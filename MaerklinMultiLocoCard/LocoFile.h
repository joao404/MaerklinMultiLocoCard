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


#pragma once

#include "Arduino.h"

class LocoFile
{
	public:
    //get last adress of loco data
		static uint16_t getLastAddressOfLocoData(uint8_t* data);

    // create .bin data from .cs2 file and return last address of .bin data
    static uint16_t createBinFromCS2(const uint8_t* cs2Data, uint8_t* binData);

    // get name of loco from binary data
    static bool getLocoNameFromBin(const uint8_t* data, String& locoName);
	
};
