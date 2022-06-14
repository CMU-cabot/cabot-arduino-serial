#!/bin/bash

function err {
    >&2 red "[ERROR] "$@
}
function red {
    echo -en "\033[31m"  ## red
    echo $@
    echo -en "\033[0m"  ## reset color
}
function help() {
    echo "$0 [options] [build|upload|all]"
    echo ""
    echo "build      build the code (default action)"
    echo "upload     upload the code"
    echo "all        build and upload if build is success"
    echo ""
    echo "-h         show this help"
    echo "-b <board> set board (default=arduino:avr:mega:cpu=atmega2560)"
    echo "-p <port>  set port (default=/dev/ttyARDUINO_MEGA)"
    echo "-m <mode>  set mode (GT/GTM) **REQUIRED** to set"
}

: ${ARDUINO_BOARD:="arduino:avr:mega:cpu=atmega2560"}
: ${ARDUINO_PORT:="/dev/ttyARDUINO_MEGA"}
: ${ARDUINO_MODE:=} # mode should be specified

board=$ARDUINO_BOARD
port=$ARDUINO_PORT
mode=$ARDUINO_MODE

while getopts "hb:p:m:" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	b)
	    board=$OPTARG
	    ;;
	p)
	    port=$OPTARG
	    ;;
	m)
	    mode=$OPTARG
	    ;;
    esac
done
shift $((OPTIND-1))

target=$1
if [ -z $target ]; then
    target=build
fi

function build() {
    echo "building..."
    echo "arduino-cli compile -b $board --build-property build.extra_flags=-D$mode ."
    arduino-cli compile -b $board --build-property build.extra_flags=-D$mode .

    if [ $? -ne 0 ]; then
	err "Please check board ($board) or mode ($mode)"
    fi
}

function upload() {
    echo "uploading..."
    arduino-cli upload -b $board -p $port .

    if [ $? -ne 0 ]; then
	err "Please check board ($board) or port ($port)"
    fi
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
