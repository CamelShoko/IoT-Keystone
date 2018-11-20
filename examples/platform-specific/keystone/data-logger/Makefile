CONTIKI_PROJECT = data-logger
all: $(CONTIKI_PROJECT)

PLATFORMS_EXCLUDE = sky nrf52dk openmote-cc2538 native simplelink 

CONTIKI=../../../..

# Debugging options
CFLAGS += -g
CFLAGS += -DWATCHDOG_CONF_DISABLE=1

# Specific additional source files beyond 'CONTIKI_PROJECT'
# PROJECT_SOURCEFILES += <file.c>

include $(CONTIKI)/Makefile.include
