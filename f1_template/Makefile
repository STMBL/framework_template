#OPT = -O1

HWVERSION = V1

f1_ADDRESS = 0x08004000

#build f1 firmware
f1: force_look
	$(MAKE) -C f1/ ADDRESS=$(f1_ADDRESS)

#build f1 bootloader
f1_boot: force_look
	$(MAKE) -C f1_boot/ ADDRESS=0x08000000 APP_ADDRESS=$(f1_ADDRESS)

#flash f1 bootloader using stlink
f1_boot_flash: force_look
	$(MAKE) -C f1_boot/ ADDRESS=0x08000000 APP_ADDRESS=$(f1_ADDRESS) flash 

#flash f1 bootloader using df-util
f1_boot_btburn: force_look
	$(MAKE) -C f1_boot/ ADDRESS=0x08000000 APP_ADDRESS=$(f1_ADDRESS) btburn

#flash f1 firmware using stlink
flash: force_look
	$(MAKE) -C f1/ ADDRESS=$(f1_ADDRESS) flash

#flash f1 firmware using df-util
btburn: force_look
	$(MAKE) -C f1/ ADDRESS=$(f1_ADDRESS) btburn

all_btburn: force_look
	$(MAKE) -C f1/ ADDRESS=$(f1_ADDRESS) all_btburn

all_flash: force_look
	$(MAKE) -C f1/ ADDRESS=$(f1_ADDRESS) all_flash

all: f1 f1_boot

# Target: clean project
#
clean: force_look
	@echo Cleaning project:
	rm -rf build
	@$(MAKE) -C f1_boot/ clean
	@$(MAKE) -C f1/ clean

force_look:
	@true
