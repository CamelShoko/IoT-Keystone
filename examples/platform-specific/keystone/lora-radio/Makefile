CONTIKI_PROJECT = lora-radio
all: $(CONTIKI_PROJECT)

PLATFORMS_EXCLUDE = sky nrf52dk openmote-cc2538 native simplelink 

CONTIKI=../../../..

# Debugging options -g : add debug symbols to the ELF
CFLAGS += -g

# Specific additional source files beyond 'CONTIKI_PROJECT'
# PROJECT_SOURCEFILES += <file.c>

# Enable the interactive shell console on the UART
MODULES += os/services/shell 

# User application is to specify the default active region.
# Only some regions are supported for a given board due to radio
# band support.
CFLAGS += -DACTIVE_REGION=LORAMAC_REGION_US915

# Disable Contiki native networking.
MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
CFLAGS  += -DNETSTACK_CONF_RADIO=nullradio_driver


include $(CONTIKI)/Makefile.include
