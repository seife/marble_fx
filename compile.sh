#!/bin/bash
BOARD=$(arduino-cli board listall|awk '/Arduino Leonardo  /{ print $NF }')
if [ -z "$BOARD" ]; then
	echo "Leonardo board not found :-("
	exit 1
fi

### configuration goes here
# use legacy HID code ("Mouse.h") instead of (IMHO) better HID-Project.h
# from https://github.com/NicoHood/HID
CONFIG_USE_LEGACYHID=${CONFIG_USE_LEGACYHID:-false}


declare -a props
props=( --build-property compiler.cpp.extra_flags=-DUSB_CONFIG_POWER=50 )
if $CONFIG_USE_LEGACYHID; then
	props+=( --build-property compiler.cpp.extra_flags=-DUSE_LEGACY_HID )
fi
if [ "$1" == "custom" ]; then
	shift
	props+=( --build-property build.usb_manufacturer='"seife"' )
	props+=( --build-property build.usb_product='"TrackMan Marble FX"' )
	props+=( --build-property build.vid=0x1209 )
fi

arduino-cli compile \
	-b "$BOARD" \
	"${props[@]}" \
	--warnings all \
	-v \
	"$@"
