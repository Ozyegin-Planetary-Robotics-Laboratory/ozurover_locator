# Locator for OzU Rover

This program listens to specified serial UART port, parses NMEA data, and displays Latitude/Longitude information whenever a new data is available.

## About

This code is based on an older C class I wrote for STM32 platform. The parser was designed to be memory-efficient and has been previously tested with GYNEO6 GPS.

Processing of NMEA messages is done char-by-char. The parser holds a state that resets when `$` character is received. For now, the program only parses `GPGLL` messages and ignores any other NMEA content.

## Requirements

* CMake >= 3.16

## Building

    mkdir build # Create a build folder
    cmake --build build # Build the Locator

## Usage
    
`locator [--debug] PORT`

`PORT` is the serial port that GPS UART is linked to. Such as /dev/ttys0

`--debug` flag prints UART buffer.