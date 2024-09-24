#!/bin/bash

FIRMWARE=firmware.bin
if [ $# -gt 0 ]; then
	FIRMWARE="$1"
fi
st-flash --reset write "$FIRMWARE" 0x8000000
