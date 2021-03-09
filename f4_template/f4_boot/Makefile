OPT = -O1 -ggdb

ADDRESS ?= 0x08000000

#config size in kB
CONF_SIZE = 4
CPUDEF = STM32F405xx
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

LIBS = ST_HAL
# LIBS += ST_USB
LIBS += CMSIS
LIBS += HAL
# LIBS += STMBLTALK
LIBS += MISC
# LIBS += CONF
LIBS += CMD

PERIPH = TIM
PERIPH += UART
# PERIPH += ADC
# PERIPH += DAC
# PERIPH += CRC
PERIPH += RCC
PERIPH += GPIO
PERIPH += PWR
PERIPH += FLASH
# PERIPH += DMA
# PERIPH += USB
# PERIPH += SPI

SOURCES += $(wildcard src/*.c)

# INCDIRS += ../shared/inc/
# SHAREDSOURCES += $(wildcard ../shared/src/*.c)

SRC_COMPS += $(wildcard src/comps/*.c)
# SHARED_COMPS += $(wildcard ../shared/src/comps/*.c)
# LIB_COMPS += $(wildcard ../../framework/comps/curpid/*.c)
# LIB_COMPS += $(wildcard ../../framework/comps/*.c)

CONFIG_TEMPLATES += $(wildcard conf/template/*.txt)

# Include the base rules
#
include ../../framework/base.mak