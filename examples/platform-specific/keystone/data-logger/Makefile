CONTIKI_PROJECT = data-logger
all: $(CONTIKI_PROJECT)

PLATFORMS_EXCLUDE = sky nrf52dk openmote-cc2538 native simplelink 

CONTIKI=../../../..

# Debugging options
CFLAGS += -g
CFLAGS += -DWATCHDOG_CONF_DISABLE=1

# Increase buffer size to reduce application processing requirements.
CFLAGS += -DAUDIO_SENSOR_CONF_RET_BUF_SIZE=128


# Specific additional source files beyond 'CONTIKI_PROJECT'
# PROJECT_SOURCEFILES += <file.c>

include $(CONTIKI)/Makefile.include
