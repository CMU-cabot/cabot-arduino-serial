FROM ghcr.io/jpconstantineau/docker_arduino_cli:latest

RUN arduino-cli config init
RUN arduino-cli core update-index
RUN arduino-cli core install arduino:avr
RUN arduino-cli lib install "Rosserial Arduino Library"@0.7.9
RUN arduino-cli lib install "Adafruit BNO055"
RUN arduino-cli lib install "Adafruit BMP280 Library"
RUN arduino-cli lib install "Adafruit MPR121"
RUN arduino-cli lib install "arduino-timer"
