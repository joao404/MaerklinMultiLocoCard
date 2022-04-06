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

void getLocosFromSD(File dir);
void printDirectory(File dir, int numTabs);


// Smartcard
const uint8_t I2C_SLAVE_ADDRESS{0x50};
const uint8_t I2C_SLAVE_SDA_PIN{28};
const uint8_t I2C_SLAVE_SCL_PIN{29};
const uint32_t I2C_BAUDRATE{400000};

const int8_t cardPin{13};
#define CARD_NOT_PLUGGED LOW
#define CARD_PLUGGED HIGH
#define MESSAGE_TIME_DIFF 200

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

Smartcard *lococard{nullptr};

File root;

std::vector<String> files;

uint16_t fileIndex{0};

bool buttonPressed{false};

GFX* ssd1306{nullptr};

void setup()
{

  // Open serial communications and wait for port to open:
  Serial.begin(115200);

   while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // deactivate original i2c slave pins
  pinMode(14, INPUT);
  pinMode(15, INPUT);

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
    
    printDirectory(root, 0);
    Serial.println("Getting .bin and .cs2 files");
    getLocosFromSD(root);
  }

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // todo get free sm
  PIO pio = pio0;
  int sm = 0;
  uint offset = pio_add_program(pio, &ws2812_program);

  ws2812_program_init(pio, sm, offset, LED_PIN, 800000, true);

  ssd1306->clear();
  ssd1306->drawString(0, 0, "Init CardSim");
  ssd1306->display();

  Serial.print("Init Smartcard:");
  pinMode(cardPin, OUTPUT);
  digitalWrite(cardPin, CARD_NOT_PLUGGED);
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

  
  ssd1306->clear();
  if (files.size() > 0)
  {
    ssd1306->drawString(0, 0, "0");
    ssd1306->drawString(0, 10, files.at(0).c_str());
  }
  else
  {
    ssd1306->drawString(0, 0, "No Loco");
  }
  ssd1306->display();
  
  digitalWrite(cardPin, CARD_PLUGGED);
}

void loop()
{
  put_pixel(urgb_u32(0x13, 0, 0));
  sleep_ms(250);
  put_pixel(urgb_u32(0, 0x13, 0));
  sleep_ms(250);
  unsigned long currentTime = millis();
  if(lococard->isReadingInProgress() && ((MESSAGE_TIME_DIFF + lococard->getLastReceiveTimeINms()) < currentTime))
  {
    Serial.println("Transmission finished");
    lococard->setReadingInProgress(false); 
    fileIndex++;
    digitalWrite(cardPin, CARD_NOT_PLUGGED);
    sleep_ms(100);
      if (fileIndex < files.size())
      {
        File file = SD.open(files.at(fileIndex));
        if (file)
        {
          if (lococard->getMemory())
          {
            if (files.at(fileIndex).endsWith(".bin")) //.bin file
            {
              if (file.size() <= 8192)
              {
                ssd1306->clear();
                ssd1306->drawString(0, 0, String(fileIndex).c_str());
                ssd1306->drawString(0, 10, files.at(fileIndex).c_str());
                ssd1306->display();
                file.read(lococard->getMemory(), file.size());
                digitalWrite(cardPin, CARD_PLUGGED);
              }
              else
              {
                Serial.print(file.name());
                Serial.print(" to large with ");
                Serial.println(file.size());
              }
            }
            else if (files.at(fileIndex).endsWith(".cs2")) // .cs2 file
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
 //digitalWrite(cardPin, CARD_NOT_PLUGGED);
}

static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
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

void printDirectory(File dir, int numTabs)
{
  while (true)
  {

    File entry = dir.openNextFile();
    if (!entry)
    {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++)
    {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory())
    {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    }
    else
    {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}
