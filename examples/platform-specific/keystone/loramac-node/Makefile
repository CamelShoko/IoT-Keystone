CONTIKI_PROJECT = loramac-node
all: $(CONTIKI_PROJECT)

PLATFORMS_EXCLUDE = sky nrf52dk openmote-cc2538 native simplelink 

CONTIKI=../../../..


# Specific additional source files beyond 'CONTIKI_PROJECT'
# PROJECT_SOURCEFILES += <file.c>

#debug
CFLAGS += -g

#MAKE_MAC = MAKE_MAC_OTHER
MODULES += os/net/mac/lora


# User application is to specify the default active region.
# Only some regions are supported for a given board due to radio
# band support.
CFLAGS += -DACTIVE_REGION=LORAMAC_REGION_US915

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
CFLAGS  += -DNETSTACK_CONF_RADIO=nullradio_driver


include $(CONTIKI)/Makefile.include
