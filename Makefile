SKETCH   := DrumLightControl
FQBN 	   := esp32:esp32:XIAO_ESP32C3:CPUFreq=160,FlashMode=qio,FlashFreq=80,FlashSize=4M,PartitionScheme=default,DebugLevel=none
SERPORT  := /dev/ttyACM0

# Provide this to build in custom path, preferably on ramfs
BUILD      := /mnt/ramdisk/arduino_build
BUILD_PATH := --build-path $(BUILD)

OPTIONS += --verbose
OPTIONS += -j $(shell nproc)

.PHONY: build upload clean

build:
	arduino-cli compile --fqbn $(FQBN) $(BUILD_PATH) $(OPTIONS) $(SKETCH)/

upload:
	arduino-cli upload --fqbn $(FQBN) $(BUILD_PATH) --port $(SERPORT) $(SKETCH)/

clean:
	rm -rf $(BUILD)
