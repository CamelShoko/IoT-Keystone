CONTIKI_PROJECT = loramac-node
all: $(CONTIKI_PROJECT)

PLATFORMS_EXCLUDE = sky nrf52dk openmote-cc2538 native simplelink 

CONTIKI=../../../..


# Specific additional source files beyond 'CONTIKI_PROJECT'
# PROJECT_SOURCEFILES += <file.c>

#MAKE_MAC = MAKE_MAC_OTHER
MODULES += os/net/mac/lora


CFLAGS += "-DREGION_US915"
CFLAGS += "-DACTIVE_REGION=LORAMAC_REGION_US915"

include $(CONTIKI)/Makefile.include
