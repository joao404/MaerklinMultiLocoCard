# MaerklinMultiLocoCard

Software for usage of ms2-card emulator.
See https://github.com/GBert/railroad/tree/master/ms2-card for more details.
Forum discussion can be found at https://www.stummiforum.de/t202512f7-Lokkarten-Emulator-Interesse.html.

## Function
Copy binary files to SD card of emulator. Plug emulator into Mobile Station. You can start with writing locos one by one with Mobile Station to emulator.
It can happen, that first writing is not successful. Investigation is ongoing.

If you want to read locos from SD and write them to Mobile Station, press button on emulator. In case of HW2.1, transmission of locos should work automaticaly.
For MS HW2.0 card must be manually plugged and unplugged.

## Precondition
- Install rp2040 delevopment framework as described at https://github.com/earlephilhower/arduino-pico. I currently recommend to use Arduino IDE.

## Used Libraries
- https://github.com/vmilea/pico_i2c_slave
- https://github.com/mbober1/RPi-Pico-SSD1306-library
