# MaerklinMultiLocoCard

Software for usage of ms2-card emulator.
See https://github.com/GBert/railroad/tree/master/ms2-card for more details.
Forum discussion can be found at https://www.stummiforum.de/t202512f7-Lokkarten-Emulator-Interesse.html.

## Function
Copy binary files to SD card of emulator. Insert card into MS2. Display should show a zero and the name of the first binary file.
Press button to signal MS2 to read this binary file. Afterwards name of next locomotive is shown. Press button to upload it as well and so on.

## Precondition
- Install rp2040 delevopment framework as described at https://github.com/earlephilhower/arduino-pico. I currently recommend to use Arduino IDE.

## Used Libraries
- https://github.com/vmilea/pico_i2c_slave
- https://github.com/mbober1/RPi-Pico-SSD1306-library
