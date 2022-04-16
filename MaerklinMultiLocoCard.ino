/*********************************************************************
 * MaerklinMultiLocoCard
 *
 * Copyright (C) 2022 Marcel Maage
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * LICENSE file for more details.
 */

 // ToDo:
 // - save .bin file to card in case that complete card was written. find with std::find search, if loco is already existing and overwrite file in that case
 // - Menu which checks for folders on card. It can be written Everything, a folder or a single loco 

// Smartcard
#include "Smartcard.h"

// SD Card
#include <SPI.h>
#include <SD.h>

// Display
#include "hardware/i2c.h"
#include "GFX.hpp"

// Led
#include "ws2812.pio.h"
#include <array>

void plugCard();
void unplugCard();
void writeDataToCardAndPlug(String& fileName);
void getLocosFromSD(File dir);


// Smartcard
const uint8_t I2C_SLAVE_ADDRESS{0x50};
const uint8_t I2C_SLAVE_SDA_PIN{28};
const uint8_t I2C_SLAVE_SCL_PIN{29};
const uint32_t I2C_BAUDRATE{400000};

const int8_t cardPin{13};
unsigned long transmissionTimeout{500};

// Display
const uint8_t I2C_DISPLAY_SDA_PIN{6};
const uint8_t I2C_DISPLAY_SCL_PIN{7};

// Button
const uint8_t BUTTON_PIN{12};

// SD Card SPI0
const uint8_t SD_CLK{2};
const uint8_t SD_MOSI{3};
const uint8_t SD_MISO{4};
const uint8_t SD_CS{5};

// Serial Flash SPI1
const uint8_t FLASH_CLK{10};
const uint8_t FLASH_MOSI{11};
const uint8_t FLASH_MISO{8};
const uint8_t FLASH_CS{9};

const uint8_t LED_PIN{16};
constexpr uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t) (r) << 8) | ((uint32_t) (g) << 16) | (uint32_t) (b);
}
inline void put_pixel(uint32_t pixel_grb);
uint8_t ledColorIndex {0};
std::array<uint32_t, 3> ledColors = {urgb_u32(0x10, 0, 0), urgb_u32(0, 0x10, 0), urgb_u32(0, 0, 0x10)};

// Smartcard class which handles data transmission
Smartcard *lococard{nullptr};

// access to filesystem on sd card
File root;

// list of files which are transferred
// only .bin currently supported
std::vector<String> files;

// index of file which is currently transmitted
uint16_t fileIndex {0};

bool buttonPressed{false};

// access to ssd1306 display
GFX* ssd1306{nullptr};

void setup()
{

  // Open serial communications and wait for port to open:
  Serial.begin(115200);

  /*
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  */

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println("Initialize Display");
  i2c_init(i2c1, 400000);
  gpio_set_function(I2C_DISPLAY_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_DISPLAY_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_DISPLAY_SDA_PIN);
  gpio_pull_up(I2C_DISPLAY_SCL_PIN);

  ssd1306 = new GFX(0x3C, size::W128xH64, i2c1);

   if (nullptr == ssd1306)
  {
    Serial.println("New error SSD1306");
    while (1)
      ;
  }

  ssd1306->rotateDisplay(0);

  ssd1306->clear();
  ssd1306->drawString(0, 0, "Init SD");
  ssd1306->display();

  Serial.print("Initializing SD card...");
  pinMode(SD_MISO, INPUT);
  pinMode(SD_MOSI, OUTPUT);
  pinMode(SD_CLK, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  SPI.setRX(SD_MISO);
  SPI.setTX(SD_MOSI);
  SPI.setSCK(SD_CLK);
  SPI.setCS(SD_CS);
  if (!SD.begin(SD_CS))
  {
    Serial.println("initialization failed!");
    // use internal memory
  }
  else
  {
    // use sd card
    Serial.println("initialization done.");
    root = SD.open("/");
    Serial.println("Getting .bin and .cs2 files");
    getLocosFromSD(root);
  }

  Serial.print("Init LED");
  PIO pio = pio0;
  int sm = 0;
  uint offset = pio_add_program(pio, &ws2812_program);
  ws2812_program_init(pio, sm, offset, LED_PIN, 800000, true);


  ssd1306->clear();
  ssd1306->drawString(0, 0, "Init CardSim");
  ssd1306->display();

  Serial.print("Init Smartcard:");
  // deactivate original i2c slave pins for smartcard
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  unplugCard();
  lococard = Smartcard::createInstance(i2c0, I2C_BAUDRATE, I2C_SLAVE_ADDRESS, I2C_SLAVE_SDA_PIN, I2C_SLAVE_SCL_PIN, &readingFinished);
  if (nullptr == lococard)
  {
    Serial.println("Failed");
    while (1)
      ;
  }
  else
  {
    Serial.println("Success");
  }
  
  if (files.size() > 0)
  {
    writeDataToCardAndPlug(files.at(fileIndex));
  }
  else
  {
    ssd1306->clear();
    ssd1306->drawString(0, 0, "No Loco");
    ssd1306->display();
    plugCard();
  }
}

void loop()
{
  unsigned long currentTime = millis();
  if(lococard->isReadingInProgress() && ((transmissionTimeout + lococard->getLastReceiveTimeINms()) < currentTime))
  {
    // Transmission ongoing and timeout reached => increase error counter
    Serial.printf("Transmissiontimeout with address:%x\n", lococard->getMemoryAddress());
    lococard->setReadingInProgress(false); 
    unplugCard();
    fileIndex++;    
    sleep_ms(100);
    if (fileIndex < files.size())
    {
      writeDataToCardAndPlug(files.at(fileIndex));
    }
    else
    {
      ssd1306->clear();
      ssd1306->drawString(0, 0, "No more loco");
      ssd1306->display();
    }
  }
}

void readingFinished()
{
 Serial.println("Reading finished");
 unplugCard();
 fileIndex++;    
 sleep_ms(100);
 if (fileIndex < files.size())
 {
   writeDataToCardAndPlug(files.at(fileIndex));
 }
 else
 {
   ssd1306->clear();
   ssd1306->drawString(0, 0, "No more loco");
   ssd1306->display();
 }
}

void plugCard()
{
  pinMode(cardPin, INPUT);
}

void unplugCard()
{
  pinMode(cardPin, OUTPUT);
  digitalWrite(cardPin, LOW);
}

inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

void writeDataToCardAndPlug(String& fileName)
{
   File file = SD.open(fileName);
        if (file)
        {
          if (lococard->getMemory())
          {
            if (fileName.endsWith(".bin")) //.bin file
            {
              if (file.size() <= 8192)
              {
                ssd1306->clear();
                ssd1306->drawString(0, 0, String(fileIndex).c_str());
                ssd1306->drawString(0, 10, fileName.c_str());
                ssd1306->display();
                Serial.printf("Next loco:%s\n", fileName.c_str());
                put_pixel(ledColors[ledColorIndex++]);
                if(ledColorIndex >= ledColors.size())
                {
                  ledColorIndex = 0;
                }
                file.read(lococard->getMemory(), file.size());
                uint16_t lastReadingAddress = getLastAddressOfLocoData(lococard->getMemory());
                Serial.printf("Last reading address:%x\n", lastReadingAddress);
                lococard->setReadingFinishedAddress(lastReadingAddress);
                plugCard();
              }
              else
              {
                Serial.print(file.name());
                Serial.print(" to large with ");
                Serial.println(file.size());
              }
            }
            else if (fileName.endsWith(".cs2")) // .cs2 file
            {
              Serial.print(file.name());
              Serial.println(" with .cs2 not supported");
            }
            else
            {
              Serial.print(file.name());
              Serial.println(" not supported");
            }
          }
          file.close();
        }
}

void getLocosFromSD(File dir)
{
  while (true)
  {
    File entry = dir.openNextFile();
    if (!entry)
    {
      // no more files
      break;
    }
    if (entry.isDirectory())
    {
      getLocosFromSD(entry);
    }
    else
    {
      String fileName = entry.name();
      if(fileName.endsWith(".bin") || fileName.endsWith(".cs2"))
      {
        Serial.println(entry.name());   
        files.emplace_back(entry.name());
      }
    }
    entry.close();
  }
}


uint16_t getLastAddressOfLocoData(uint8_t* data)
{

  if(nullptr == data)
  {
    return 0;
  }
  uint16_t index = 0;

    uint16_t length = data[0];
    if (length != 2)
    {
      Serial.printf("unknown loco card type - first byte 0x%02x should be 2\n", length);
      return 0;
    }
    uint16_t id = data[1] + (data[2] << 8);
    switch (id) {
    case 0x00E5: //PREAMBLE_MFX
    case 0x00F5: //PREAMBLE_MFX2
    case 0x0075: //PREAMBLE_MM
    case 0x0117: //PREAMBLE_MFX_F32
    case 0x00C5: //PREAMBLE_OTHER
    break;
    default:
      Serial.printf("unknown loco card type 0x%04x\n\n", id);
      return 0;
    }
    uint16_t i = 3;
    while (i < 8192) {
  index = data[i++];
  length = data[i++];

  switch (index) {

  case 0:
      length = data[i] + (data[i+1] << 8);
      i += 2;
      id = data[i++];
      while ((id != 0) && (id != 255)) {
    length = data[i++];
    switch (id) {
    case 0x05:
       i += (length + (data[i++] << 8));
       break;
    default:
      i += length;
        break;
    }
    id = data[i++];
    if (id == 0)
        id = data[i++];
      }
      break;
  default:
      i+=length;
      break;
  }
  if (index == 0)
      break;
    }
    // Serial.printf("i:%x\n", i);

    return i-1;
}
