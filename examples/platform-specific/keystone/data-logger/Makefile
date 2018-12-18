CONTIKI_PROJECT = data-logger
all: $(CONTIKI_PROJECT)

PLATFORMS_EXCLUDE = sky nrf52dk openmote-cc2538 native simplelink 

CONTIKI=../../../..

# Debugging options -g : add debug symbols to the ELF
CFLAGS += -g

# Specific additional source files beyond 'CONTIKI_PROJECT'
# PROJECT_SOURCEFILES += <file.c>

# Enable the interactive shell console on the UART
MODULES += os/services/shell 

include $(CONTIKI)/Makefile.include
