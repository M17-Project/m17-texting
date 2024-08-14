CFLAGS = `pkg-config --cflags gtk+-3.0` -O2 -Wall -Wextra
LIBS   = `pkg-config --libs gtk+-3.0` -lm -lm17 -lao

m17-texting: m17-texting.c
	$(CC) $(CFLAGS) -o m17-texting m17-texting.c $(LIBS)
