#!/bin/bash
BOARD=$(arduino-cli board listall|awk '/Arduino Leonardo  /{ print $NF }')
if [ -z "$BOARD" ]; then
	echo "Leonardo board not found :-("
	exit 1
fi

PORT="$1"
shift
if [ -z "$PORT" ] || [ ! -c "$PORT" ]; then
	echo "usage: upload.sh <port>"
	echo "example: ./upload.sh /dev/ttyACM1"
	echo "connected boards:"
	arduino-cli board list
	exit 1
fi
arduino-cli upload \
	-b "$BOARD" \
	-p "$PORT" \
	-v \
	"$@"
