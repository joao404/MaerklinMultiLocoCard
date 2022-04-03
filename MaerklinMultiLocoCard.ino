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

void printDirectory(File dir, int numTabs);

// Smartcard
const uint16_t I2C_MEMORY_SIZE{8192};
const uint8_t I2C_SLAVE_ADDRESS{0x50};
//#ifdef WAVESHARE_RP2040_ZERO
const uint8_t I2C_SLAVE_SDA_PIN{28};
const uint8_t I2C_SLAVE_SCL_PIN{29};
//#else
//const uint8_t I2C_SLAVE_SDA_PIN{20};
//const uint8_t I2C_SLAVE_SCL_PIN{21};
//#endif

const uint32_t I2C_BAUDRATE{400000};
const int8_t activationPin{13};

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

//#ifdef WAVESHARE_RP2040_ZERO
const uint8_t LED_PIN{16};
//#else
//const uint8_t LED_PIN{25};
//#endif

Smartcard *lococard{nullptr};

GFX *ssd1306{nullptr};

File root;

std::vector<String> files;

uint16_t fileIndex{0};

bool buttonPressed{false};

void setup()
{

  // Open serial communications and wait for port to open:
  Serial.begin(115200);

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

  ssd1306->clear();
  ssd1306->drawString(0, 0, "Init SD");
  ssd1306->display();

  Serial.print("Initializing SD card...");
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

    // safe name of all files on sd card
    for (File file = root.openNextFile(); !file; file = root.openNextFile())
    {
      if (!file.isDirectory())
      {
        files.emplace_back(file.name());
      }
    }

    printDirectory(root, 0);
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
  lococard = Smartcard::createInstance(I2C_MEMORY_SIZE, i2c0, I2C_BAUDRATE, I2C_SLAVE_ADDRESS, I2C_SLAVE_SDA_PIN, I2C_SLAVE_SCL_PIN, activationPin);
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
}

void loop()
{
  put_pixel(urgb_u32(0x1f, 0, 0));
  sleep_ms(250);
  put_pixel(urgb_u32(0, 0x1f, 0));
  sleep_ms(250);

  if (!digitalRead(BUTTON_PIN))
  {
    if (!lococard->isReadingInProgress()) // nothing is in progess
    {
      // writting file into smartcard simulation triggered
      if (fileIndex < files.size())
      {
        File file = SD.open(files.at(fileIndex));
        if (file)
        {
          if (lococard->getMemory())
          {
            if (files.at(fileIndex).endsWith(".bin")) //.bin file
            {
              if (file.size() <= I2C_MEMORY_SIZE)
              {
                file.read(lococard->getMemory(), file.size());
                // trigger new reading by pulling smartcard pin low
                lococard->triggerActivation();
                buttonPressed = true;
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
    }
  }

  if ((!lococard->isReadingInProgress()) && buttonPressed)
  {
    Serial.println("Reading finished");
    buttonPressed = false;
    fileIndex++;
    ssd1306->clear();
    if (fileIndex < files.size())
    {
      ssd1306->drawString(0, 0, String(fileIndex).c_str());
      ssd1306->drawString(0, 10, files.at(fileIndex).c_str());
    }
    else
    {
      ssd1306->drawString(0, 0, "No more loco");
    }
    ssd1306->display();
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

static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}
