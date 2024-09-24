//std
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
//sys
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
//audio
#include <ao/ao.h>
//M17
#include "m17.h"
//GTK
#include <gtk/gtk.h> 

//GTK widgets
GtkWidget* win;
GtkWidget* txt_src; GtkWidget* txt_dst; GtkWidget* txt_auddev; GtkWidget* txt_com; GtkWidget* txt_msg;
GtkWidget* btn_tx;
GtkWidget* scale_tx;
GtkWidget* sw_phase;

//COM
static struct termios oldterminfo;
int ser;

//input data
struct settings_t
{
	char* dst_raw;							//raw, unencoded destination address
	char* src_raw;							//raw, unencoded source address
	uint8_t can;							//Channel Access Number
	char* msg;								//text message
	uint8_t phase;							//baseband phase 1-normal, 0-inverted
	float aud_lvl;							//audio level (0.0-min, 100.0-max)
} settings;

//M17 stuff
lsf_t lsf;									//Link Setup Frame data

uint8_t full_packet_data[32*25]={0};		//packet payload
uint32_t pkt_sym_cnt=0;
uint16_t num_bytes=0;						//size of payload in bytes
uint8_t frame_cnt=0;						//total frame count

uint8_t rf_bits[SYM_PER_PLD*2];				//type-4 bits for transmission
float symbols[SYM_PER_FRA];					//frame symbols

//audio playback
int16_t samples[32+2][1920];				//S16 samples, fs=48kHz, enough for 40ms frames
#define SAM_PER_FRA 3840U					//samples per frame (at sps=10 and fs=48kHz)
											//the signal is stereo, therefore this value is doubled

//funcs
//filter symbols, flt is assumed to be 81 taps long
void filter_symbols(int16_t* out, const float* in, const float* flt, uint8_t phase_inv, float gain)
{
	static float last[81]; //memory for last samples

	for(uint8_t i=0; i<SYM_PER_FRA; i++)
	{
		for(uint8_t j=0; j<10; j++)
		{
			for(uint8_t k=0; k<80; k++)
				last[k]=last[k+1];

			if(j==0)
			{
				if(phase_inv)
					last[80]=-in[i];
				else
					last[80]= in[i];
			}
			else
				last[80]=0;

			float acc=0.0f;
			for(uint8_t k=0; k<81; k++)
				acc+=last[k]*flt[k];
			
			if(out!=NULL) out[i*10+j]=acc*7168.0*gain*sqrtf(10.0f);
		}
	}
}

//generate baseband samples - add args later
void generate_baseband(uint8_t phase_inv, float gain)
{
	//flush the RRC filter
	float flush[SYM_PER_FRA]={0};
	filter_symbols(NULL, flush, rrc_taps_10, phase_inv, gain);

	//generate preamble
	pkt_sym_cnt=0;
	send_preamble(symbols, &pkt_sym_cnt, PREAM_LSF);
	filter_symbols(&samples[0][0], symbols, rrc_taps_10, phase_inv, gain);

	//generate LSF
	send_frame(symbols, NULL, FRAME_LSF, &lsf, 0, 0);
	filter_symbols(&samples[1][0], symbols, rrc_taps_10, phase_inv, gain);

	//generate frames
	uint8_t cnt=0;
	uint8_t frame_payload[26];

	while(num_bytes>25)
	{
		memcpy(frame_payload, &full_packet_data[cnt*25], 25);
		frame_payload[25]=cnt<<2;
		send_frame(symbols, frame_payload, FRAME_PKT, NULL, 0, 0);
		filter_symbols(&samples[2+cnt][0], symbols, rrc_taps_10, phase_inv, gain);
		cnt++;
		num_bytes-=25;
	}

	memset(frame_payload, 0, 26);
	memcpy(frame_payload, &full_packet_data[cnt*25], num_bytes);
	frame_payload[25]=0x80|(num_bytes<<2);
	send_frame(symbols, frame_payload, FRAME_PKT, NULL, 0, 0);
	filter_symbols(&samples[2+cnt][0], symbols, rrc_taps_10, phase_inv, gain);

	//generate EOT
	pkt_sym_cnt=0;
	send_eot(symbols, &pkt_sym_cnt);
	filter_symbols(&samples[3+cnt][0], symbols, rrc_taps_10, phase_inv, gain);

	frame_cnt=4+cnt;
}

//close COM
void closeserial(int fd)
{
	tcsetattr(fd, TCSANOW, &oldterminfo);
	if(close(fd) < 0)
		perror("closeserial()");
}

//open COM
int openserial(char *devicename)
{
	int fd;
	struct termios attr;

	if((fd = open(devicename, O_RDWR)) == -1)
	{
		perror("openserial(): open()");
		return 0;
	}

	if(tcgetattr(fd, &oldterminfo) == -1)
	{
		perror("openserial(): tcgetattr()");
		return 0;
	}

	attr = oldterminfo;
	attr.c_cflag |= CRTSCTS | CLOCAL;
	attr.c_oflag = 0;

	if(tcflush(fd, TCIOFLUSH) == -1)
	{
		perror("openserial(): tcflush()");
		return 0;
	}

	if(tcsetattr(fd, TCSANOW, &attr) == -1)
	{
		perror("initserial(): tcsetattr()");
		return 0;
	}

	return fd;
}

//set RTS
int setRTS(int fd, int level)
{
    int status;

    if(ioctl(fd, TIOCMGET, &status) == -1)
	{
        perror("setRTS(): TIOCMGET");
        return 0;
    }

    if (level)
        status |= TIOCM_RTS;
    else
        status &= ~TIOCM_RTS;

    if (ioctl(fd, TIOCMSET, &status) == -1)
	{
        perror("setRTS(): TIOCMSET");
        return 0;
    }

    return 1;
}

void end_program(GtkWidget* wid, gpointer ptr)
{
	(void)wid;
	(void)ptr;
    gtk_main_quit();
}

void button_press(void)
{
    settings.src_raw = (char*)gtk_entry_get_text(GTK_ENTRY(txt_src));
    settings.dst_raw = (char*)gtk_entry_get_text(GTK_ENTRY(txt_dst));
    settings.msg = (char*)gtk_entry_get_text(GTK_ENTRY(txt_msg));

	const char* com = gtk_entry_get_text(GTK_ENTRY(txt_com));
	settings.aud_lvl = gtk_range_get_value(GTK_RANGE(scale_tx));
	settings.phase = gtk_switch_get_state(GTK_SWITCH(sw_phase));

	//obtain data and append with CRC
	memset(full_packet_data, 0, 32*25);
	full_packet_data[0]=0x05;
	memcpy((char*)&full_packet_data[1], settings.msg, strlen(settings.msg));
	num_bytes=strlen(settings.msg)+2; //0x05 and 0x00
	uint16_t packet_crc=CRC_M17(full_packet_data, num_bytes);
	full_packet_data[num_bytes]  =packet_crc>>8;
	full_packet_data[num_bytes+1]=packet_crc&0xFF;
	num_bytes+=2; //count 2-byte CRC too

	//encode dst, src for the lsf struct
	uint64_t dst_enc=0, src_enc=0;
	uint16_t type=0;
	encode_callsign_value(&dst_enc, (uint8_t*)settings.dst_raw);
	encode_callsign_value(&src_enc, (uint8_t*)settings.src_raw);
	for(int8_t i=5; i>=0; i--)
	{
		lsf.dst[5-i]=(dst_enc>>(i*8))&0xFF;
		lsf.src[5-i]=(src_enc>>(i*8))&0xFF;
	}
	
	type=((uint16_t)0x01<<1)|((uint16_t)settings.can<<7); //packet mode, content: data
	lsf.type[0]=(uint16_t)type>>8;
	lsf.type[1]=(uint16_t)type&0xFF;
	memset(&lsf.meta, 0, 112/8);

	//calculate LSF CRC
	uint16_t lsf_crc=LSF_CRC(&lsf);
	lsf.crc[0]=lsf_crc>>8;
	lsf.crc[1]=lsf_crc&0xFF;

	//print out data
    fprintf(stderr, "SRC: %s\nDST: %s\nMSG: %s\n", settings.src_raw, settings.dst_raw, settings.msg);
	fprintf(stderr, "Raw MSG (hex):\n");
	for(uint8_t i=0; i<strlen(settings.msg); i++)
		fprintf(stderr, "%02X ", (uint8_t)settings.msg[i]);
	fprintf(stderr, "\n");

	//open COM and set RTS
	ser = openserial((char*)com);
	setRTS(ser, 1); //PTT down
	usleep(40000);

	//play baseband
	ao_device* device;
	ao_sample_format format;

	//init
	ao_initialize();

	//setup
	int default_driver = ao_default_driver_id();
	memset(&format, 0, sizeof(format));
	format.bits = 16;
	format.channels = 1;
	format.rate = 48000;
	format.byte_format = AO_FMT_LITTLE;

	//open driver
	device = ao_open_live(default_driver, &format, NULL); //NULL=no options
	if(device == NULL)
	{
		fprintf(stderr, "Error opening audio device.\n");
		gtk_main_quit();
	}

	generate_baseband(settings.phase, settings.aud_lvl/100.0f);
	for(uint8_t i=0; i<frame_cnt; i++)
		ao_play(device, (char*)&samples[i][0], SAM_PER_FRA); //for a stereo signal, the buffer is LRLRLR... each sample is an int16_t, size in bytes
		//EoT frame is only 20ms - why?

	usleep(100000);
	setRTS(ser, 0); //PTT up

	//close
	ao_close(device);
	ao_shutdown();
}

//called when window is closed
void window_delete_event(void)
{
    gtk_main_quit();
}

int main(int argc, char** argv)
{
    GError* error = NULL;

    gtk_init(&argc, &argv); 

    GtkBuilder* builder = gtk_builder_new();
    if(0 == gtk_builder_add_from_file(builder, "m17-texting.glade", &error))
    {
        g_printerr("Error loading file: %s\n", error->message);
        g_clear_error(&error);
      
        return 1;
    }

    win = (GtkWidget*)gtk_builder_get_object(builder, "win");
    txt_src = (GtkWidget*)gtk_builder_get_object(builder, "txt_src");
    txt_dst = (GtkWidget*)gtk_builder_get_object(builder, "txt_dst");
	txt_auddev = (GtkWidget*)gtk_builder_get_object(builder, "txt_auddev");
	txt_com = (GtkWidget*)gtk_builder_get_object(builder, "txt_com");
    txt_msg = (GtkWidget*)gtk_builder_get_object(builder, "txt_msg");
	btn_tx = (GtkWidget*)gtk_builder_get_object(builder, "btn_tx");
	scale_tx = (GtkWidget*)gtk_builder_get_object(builder, "scale_tx");
	sw_phase = (GtkWidget*)gtk_builder_get_object(builder, "sw_phase");

    g_signal_connect(btn_tx, "clicked", G_CALLBACK(button_press), NULL);
    g_signal_connect(win, "delete_event", G_CALLBACK(window_delete_event), NULL);

    gtk_widget_show(win);

	//infinite loop while the window is opened
    gtk_main();

	//this gets executed at exit
	if(ser)
		closeserial(ser);
	fprintf(stderr, "Exiting.\n");
     
    return 0;
}
