#OPT = -O1

HWVERSION = V1

f4_ADDRESS = 0x08000000

#build f4 firmware
f4: force_look
	$(MAKE) -C f4/ ADDRESS=$(f4_ADDRESS)

#build f4 bootloader
# f4_boot: force_look
# 	$(MAKE) -C f4_boot/ ADDRESS=0x08000000 APP_ADDRESS=$(f4_ADDRESS)

# #flash f4 bootloader using stlink
# f4_boot_flash: force_look
# 	$(MAKE) -C f4_boot/ ADDRESS=0x08000000 APP_ADDRESS=$(f4_ADDRESS) flash 

# #flash f4 bootloader using df-util
# f4_boot_btburn: force_look
# 	$(MAKE) -C f4_boot/ ADDRESS=0x08000000 APP_ADDRESS=$(f4_ADDRESS) btburn

#flash f4 firmware using stlink
flash: force_look
	$(MAKE) -C f4/ ADDRESS=$(f4_ADDRESS) flash

#flash f4 firmware using df-util
btburn: force_look
	$(MAKE) -C f4/ ADDRESS=$(f4_ADDRESS) btburn

all_btburn: force_look
	$(MAKE) -C f4/ ADDRESS=$(f4_ADDRESS) all_btburn

all_flash: force_look
	$(MAKE) -C f4/ ADDRESS=$(f4_ADDRESS) all_flash

all: f4 f4_boot

# Target: clean project
#
clean: force_look
	@echo Cleaning project:
	rm -rf build
	# @$(MAKE) -C f4_boot/ clean
	@$(MAKE) -C f4/ clean

force_look:
	@true
