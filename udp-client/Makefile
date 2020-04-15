DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI_PROJECT=udp-client-sepideh
PROJECT_SOURCEFILES += collect-common.c
all: $(CONTIKI_PROJECT)
CONTIKI=../../../../..

#linker optimizations
SMALL=1

#APPS+= battery-control 
#APPS += battery-collect
#APPS += battery-collect-view
#APPS += sepideh

APPS += serial-shell powertrace collect-view

WITH_CENTRAL_SCH=0
ifeq ($(WITH_CENTRAL_SCH),1)
APPS+=central-sch
endif

ifdef WITH_COMPOWER
APPS+=powertrace
CFLAGS+=-DCONTIKIMAC_CONF_COMPOWER=1 -DWITH_COMPOWER=1 -DQUEUEBUF_CONF_NUM=4
endif
ifdef SERVER_REPLY
CFLAGS+=-DSERVER_REPLY=$(SERVER_REPLY)
endif
ifdef PERIOD
CFLAGS+=-DPERIOD=$(PERIOD)
endif

ifeq ($(MAKE_WITH_NON_STORING),1)
CFLAGS+=-DWITH_NON_STORING=1
endif

CONTIKI_WITH_IPV6=1
include $(CONTIKI)/Makefile.include
