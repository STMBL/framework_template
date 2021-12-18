OPT = -O1 -ggdb

#config size in kB
CONF_SIZE = 4
CPUDEF = STM32F103xE
CPUFLAGS = -mthumb -mcpu=cortex-m3
XTAL_FREQ = 8000000

LDFLAGS += -lc 
#-specs=nosys.specs

LIBS = CMSIS
LIBS += ST_HAL

#PERIPH = TIM

SOURCES += $(wildcard src/*.c)
SOURCES += $(wildcard src/*.cpp)

# INCDIRS += ../shared/inc/
# SHAREDSOURCES += $(wildcard ../shared/src/*.c)

# SRC_COMPS += $(wildcard src/comps/*.c)
# SHARED_COMPS += $(wildcard ../shared/src/comps/*.c)
# LIB_COMPS += $(wildcard ../../framework/comps/curpid/*.c)
# LIB_COMPS += $(wildcard ../../framework/comps/*.c)

# CONFIG_TEMPLATES += $(wildcard conf/template/*.txt)

# Include the base rules
#
include ../../framework/base.mak