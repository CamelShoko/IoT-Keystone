CONTIKI_PROJECT = mesh-channel-scan
all: $(CONTIKI_PROJECT)

PLATFORMS_EXCLUDE = sky nrf52dk openmote-cc2538 native simplelink 

CONTIKI=../../../..

# Debugging options -g : add debug symbols to the ELF
CFLAGS += -g

# Specific additional source files beyond 'CONTIKI_PROJECT'
# PROJECT_SOURCEFILES += <file.c>

# Enable the interactive shell console on the UART
MODULES += os/services/shell 

# Specify the MAC layer (default is CSMA - radio rx always on)
# Options:
#  MAKE_MAC_NULLMAC = 0
#  MAKE_MAC_CSMA = 1
#  MAKE_MAC_TSCH = 2
#  MAKE_MAC_BLE = 3
#  MAKE_MAC_OTHER = 4

MAKE_MAC = MAKE_MAC_CSMA

include $(CONTIKI)/Makefile.include
