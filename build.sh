#!/bin/bash

function help() {
    echo "$0 [options] [build|upload|all]"
    echo ""
    echo "build      build the code (default action)"
    echo "upload     upload the code"
    echo "all        build and upload if build is success"
    echo ""
    echo "-h         show this help"
    echo "-b         set board (default=arduino:avr:mega:cpu=atmega2560)"
    echo "-p         set port (default=/dev/ttyARDUINO_MEGA)"
}

: ${ARDUINO_BOARD:="arduino:avr:mega:cpu=atmega2560"}
: ${ARDUINO_PORT:="/dev/ttyARDUINO_MEGA"}
: ${ARDUINO_MODE:="GT"}

board=$ARDUINO_BOARD
port=$ARDUINO_PORT
mode=$ARUDINO_MODE

while getopts "hb:p:m:" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	b)
	    borad=$OPTARG
	    ;;
	p)
	    port=$OPTARG
	    ;;
	m)
	    mode=$OPTARG
	    ;;
    esac
done

target=$1
if [ -z $target ]; then
    target=build
fi

function build() {
    echo "building..."
    arduino-cli compile -b $board . --build-properties compiler.cpp.extra_flag=-D$mode
}

function upload() {
    echo "uploading..."
    arduino-cli upload -b $board -p $port . --build-properties compiler.cpp.extra_flag=-D$mode
}    

if [ $target == "build" ]; then
    build
fi

if [ $target == "upload" ]; then
    upload
fi

if [ $target == "all" ]; then
    build && upload
fi
