# for use with arduino-mk https://github.com/sudar/Arduino-Makefile
#
BOARD_TAG    = leonardo

# override USB VID, set power consumption to 50mA
USB_OPTS = -DUSB_MANUFACTURER='"seife"' -DUSB_PRODUCT='"TrackMan Marble FX"' -UUSB_VID -DUSB_VID=0x1209 -DUSB_CONFIG_POWER=50
CFLAGS_STD      = -std=gnu11 -flto -fno-fat-lto-objects      $(USB_OPTS)
CXXFLAGS_STD    = -std=gnu++11 -fno-threadsafe-statics -flto $(USB_OPTS)
LDFLAGS += -flto -fuse-linker-plugin
CFLAGS = -W -Wall -Wextra
CXXFLAGS = -W -Wall -Wextra

ARDUINO_LIBS = HID Mouse

include /usr/share/arduino/Arduino.mk
