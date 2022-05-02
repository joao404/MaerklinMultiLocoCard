/*********************************************************************
 * LocoFile
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
 
 #include "LocoFile.h"

uint16_t LocoFile::getLastAddressOfLocoData(uint8_t* data)
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


uint16_t LocoFile::createBinFromCS2(const uint8_t* cs2Data, uint8_t* binData)
{
  uint16_t index = 0;
  
  return index;
}

String LocoFile::getLocoNameFromBin(const uint8_t* binData)
{
  return "Testloco";
}
