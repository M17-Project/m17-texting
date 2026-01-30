CC			?= cc
CFLAGS 		:= $(shell pkg-config --cflags gtk+-3.0) \
        		-O2 -Wall -Wextra -Werror -Wpedantic
RES_CFLAGS	:= $(filter-out -Wpedantic -Werror,$(CFLAGS))
LIBS		:= $(shell pkg-config --libs gtk+-3.0) -lm -lm17 -lao

TARGET  := m17-texting
SRC     := m17-texting.c
RES_XML := resources.xml
RES_C   := resources.c

.PHONY: all clean install

all: $(TARGET)

$(RES_C): $(RES_XML)
	glib-compile-resources $< --generate-source --target=$@

resources.o: resources.c
	$(CC) $(RES_CFLAGS) -c $< -o $@

m17-texting: m17-texting.c resources.o
	$(CC) $(CFLAGS) -o $@ m17-texting.c resources.o $(LIBS)

install: $(TARGET)
	install -Dm755 $(TARGET) $(DESTDIR)/usr/local/bin/$(TARGET)

clean:
	rm -f $(TARGET) $(RES_C)
