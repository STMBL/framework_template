OPT = -O1 -ggdb

ADDRESS ?= 0x08004000

#config size in kB
CONF_SIZE = 4
CPUDEF = STM32F103xE
CPUFLAGS = -mthumb -mcpu=cortex-m3
XTAL_FREQ = 8000000

CFLAGS += -DHAL_MAX_PINS=512
CFLAGS += -DHAL_MAX_COMPS=32
CFLAGS += -DHAL_MAX_CTX=4096

CFLAGS += -DVERSION_MAJOR=1
CFLAGS += -DVERSION_MINOR=0
CFLAGS += -DVERSION_PATCH=0

LDFLAGS += -lc
# -specs=nosys.specs
#CFLAGS += -nostdlib

LIBS = CMSIS
LIBS += ST_HAL

PERIPH = RCC

SOURCES += $(wildcard src/*.c)

# INCDIRS += ../shared/inc/
# SHAREDSOURCES += $(wildcard ../shared/src/*.c)

SRC_COMPS += $(wildcard src/comps/*.c)
SRC_COMPS += $(wildcard src/comps/*.cpp)
# SHARED_COMPS += $(wildcard ../shared/src/comps/*.c)
#LIB_COMPS += $(wildcard ../../framework/comps/*.c)
#LIB_COMPS += $(wildcard ../../framework/comps/*.cpp)

#CONFIG_TEMPLATES += $(wildcard conf/template/*.txt)

# Include the base rules
#
include ../../framework/base.mak