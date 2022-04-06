# MaerklinMultiLocoCard

Software for usage of ms2-card emulator.
See https://github.com/GBert/railroad/tree/master/ms2-card for more details.
Forum discussion can be found at https://www.stummiforum.de/t202512f7-Lokkarten-Emulator-Interesse.html.

## Function
Copy binary files to SD card of emulator. Display should show a zero and the name of the first binary file. Insert card into MS2. 
Number should increase and show file name of current loco, that is being updated. (This is currently not properly working. Manually remove and put back in)

## Precondition
- Install rp2040 delevopment framework as described at https://github.com/earlephilhower/arduino-pico. I currently recommend to use Arduino IDE.

## Used Libraries
- https://github.com/vmilea/pico_i2c_slave
- https://github.com/mbober1/RPi-Pico-SSD1306-library
