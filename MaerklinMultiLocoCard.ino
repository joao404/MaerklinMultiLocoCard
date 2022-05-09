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

// functions for .bin and .cs2 files
#include "LocoFile.h"

void readingFinished();
void writeCallback();
bool writeDataToCardAndPlug(String& fileName);
void getLocosFromSD(File dir);
void prepareNextLoco();


// Smartcard
const uint8_t I2C_SLAVE_ADDRESS{0x50};
const uint8_t I2C_SLAVE_SDA_PIN{28};
const uint8_t I2C_SLAVE_SCL_PIN{29};
const uint32_t I2C_BAUDRATE{400000};

const int8_t cardPin{13};
unsigned long transmissionTimeout{700};

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

bool writeTriggered{false};
bool dataWritten{false};

// access to ssd1306 display
GFX* ssd1306{nullptr};

void setup()
{

  // Open serial communications and wait for port to open:
  Serial.begin(115200);

  
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  

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
    ssd1306->clear();
    ssd1306->drawString(0, 0, "NO SD Card plugged");
    ssd1306->display();
    while(1);
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
  Smartcard::Config smartCardConfig;
  smartCardConfig.i2c = i2c0;
  smartCardConfig.baud = I2C_BAUDRATE;
  smartCardConfig.address = I2C_SLAVE_ADDRESS;
  smartCardConfig.sda = I2C_SLAVE_SDA_PIN;
  smartCardConfig.scl = I2C_SLAVE_SCL_PIN;
  smartCardConfig.cardPin = cardPin;
  smartCardConfig.transmissionTimeoutINms = 500;
  smartCardConfig.delayEndOfTransmissionINms = 50;
  smartCardConfig.delayUnplugCardINms = 100;
  smartCardConfig.readingFinishedCallback = &readingFinished;
  smartCardConfig.writeCallback = &writeCallback;
  lococard = Smartcard::createInstance(smartCardConfig);
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
  
  prepareNextLoco();
  lococard->plugCard();
}

void loop()
{
  unsigned long currentTime = millis();
  if(writeTriggered && dataWritten && ((transmissionTimeout + lococard->getTransmissionTimer()) < currentTime))
  {
    Serial.println("Write finished");
    handleLocoWritingFinished();    
  }
  if((LOW == digitalRead(BUTTON_PIN)) && !writeTriggered)
  {
    // if button is pressed, switch to write mode
    writeTriggered = true;
    dataWritten = false;
    memset(lococard->getMemory(), 0, 8192);
    lococard->setReadingFinishedAddress(8192);
    Serial.println("Writemode activ");
    ssd1306->clear();
    ssd1306->drawString(0, 0, "Bereit zum Schreiben");
    ssd1306->display();
  }

  lococard->cyclic();
}

void writeCallback()
{
  if(writeTriggered)
  {
    dataWritten = true;
  }
}

void readingFinished()
{
  prepareNextLoco();
}

void handleLocoWritingFinished()
{
  String locoName;
    if(LocoFile::getLocoNameFromBin(lococard->getMemory(), locoName))
    {
      Serial.printf("Got loco:%s\n", locoName.c_str());
      locoName.replace(" ", "_");
      locoName += ".bin";
      ssd1306->clear();
      ssd1306->drawString(0, 0, locoName.c_str());
      ssd1306->drawString(0, 10, "Schreiben auf SD");

      bool success {false};
      if(SD.exists(locoName))
      {
        SD.remove(locoName);
      }
      File file = SD.open(locoName, FILE_WRITE);
      if (file)
      {
        if(8192 == file.write(lococard->getMemory(), 8192))
        {
          success = true;
        }
        file.close();
      }
      ssd1306->drawString(0, 20, success ? "erfolgreich" : "fehlgeschlagen");
      ssd1306->display();
    }
    else
    {
    ssd1306->clear();
    ssd1306->drawString(0, 0, "Kein Lokname");
    ssd1306->display();
    }
    writeTriggered = false;
    dataWritten = false;
}

void prepareNextLoco()
{
 while(1)
 {       
   if (fileIndex < files.size())
   {
     if(writeDataToCardAndPlug(files.at(fileIndex++)))
     {
       break;
     }
    }
    else
    {
      ssd1306->clear();
      ssd1306->drawString(0, 0, "Alle Lokomotiven");
      ssd1306->drawString(0, 10, "gelesen");
      ssd1306->display();
      break;
    }
  }
}

bool writeDataToCardAndPlug(String& fileName)
{
  bool result = false;
   File file = SD.open(fileName);
        if (file)
        {
          if (lococard->getMemory())
          {
            if (fileName.endsWith(".bin")) //.bin file
            {
              if (file.size() <= 8192)
              {
                Serial.printf("Next loco:%s\n", fileName.c_str());
                // set color of led
                put_pixel(ledColors[ledColorIndex++]);
                if(ledColorIndex >= ledColors.size())
                {
                  ledColorIndex = 0;
                }
                // read loco data into buffer
                file.read(lococard->getMemory(), file.size());
                uint16_t lastReadingAddress = LocoFile::getLastAddressOfLocoData(lococard->getMemory());
                Serial.printf("Last reading address:%x\n", lastReadingAddress);
                lococard->setReadingFinishedAddress(lastReadingAddress);
                lococard->getMemory()[lastReadingAddress + 1] = 0;
                lococard->getMemory()[lastReadingAddress + 2] = 0;
                lococard->getMemory()[lastReadingAddress + 3] = 0;

                // write current loco to display
                ssd1306->clear();
                String fileIndexStr = String(fileIndex);
                fileIndexStr += " von ";
                fileIndexStr += files.size();
                ssd1306->drawString(0, 0, fileIndexStr.c_str());
                ssd1306->drawString(0, 10, fileName.c_str());
                uint16_t id = lococard->getMemory()[1] + (lococard->getMemory()[2] << 8);
                switch (id) {
                  case 0x00E5: //PREAMBLE_MFX
                  ssd1306->drawString(0, 20, "MFX");
                  break;
                  case 0x00E7: //PREAMBLE_MM2
                  ssd1306->drawString(0, 20, "MM2");
                  break;
                  case 0x00F5: //PREAMBLE_MFX2
                  ssd1306->drawString(0, 20, "MFX2");
                  break;
                  case 0x0075: //PREAMBLE_MM
                  ssd1306->drawString(0, 20, "MM");
                  break;
                  case 0x0117: //PREAMBLE_MFX_F32
                  ssd1306->drawString(0, 20, "MFX_F32");
                  break;
                  case 0x00C5: //PREAMBLE_OTHER
                  ssd1306->drawString(0, 20, "DCC");
                  break;
                  default:
                  ssd1306->drawString(0, 20, "Protocol unknown");
                  break;
                }
                ssd1306->display();
                
                lococard->triggerReading();
                result = true;
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
              String cs2data = file.readString();
              uint16_t lastReadingAddress = LocoFile::createBinFromCS2((const uint8_t*)cs2data.c_str(), lococard->getMemory());
              if(lastReadingAddress > 0)
              {
                Serial.printf("Last reading address:%x\n", lastReadingAddress);
                lococard->setReadingFinishedAddress(lastReadingAddress);
                lococard->triggerReading();
                result = true;
              }
            }
            else
            {
              Serial.print(file.name());
              Serial.println(" not supported");
            }
          }
          file.close();
        }
        return result;
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

inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}
