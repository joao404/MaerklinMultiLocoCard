/*********************************************************************
 * Smartcard
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

#include "i2c_fifo.h"
#include "i2c_slave.h"
#include <pico/stdlib.h>
#include <vector>

class Smartcard
{
    
	public:
	  virtual ~Smartcard();

    static Smartcard* createInstance(uint16_t size, i2c_inst_t *i2c, uint32_t baud, uint8_t address = 0x50, uint8_t sda = PICO_DEFAULT_I2C_SDA_PIN, uint8_t scl = PICO_DEFAULT_I2C_SCL_PIN, int8_t activationPin = -1);
  
	  static void interruptHandler(i2c_inst_t *i2c, i2c_slave_event_t event);

    uint8_t* getMemory();

    void triggerActivation();

    bool isReadingInProgress();

    typedef enum
    {
      WaitingForData,
      WaitingForAddress,
      AddressReceived
    } i2cState;

	private:
    // Constructor is private to prevent creation
    Smartcard();
    
    static std::vector<Smartcard> m_instances;
    
    i2c_inst_t* m_i2c;
    uint8_t m_address;
    int8_t m_activationPin;
    i2cState m_state; 

    bool readingInProgress;

    uint8_t* m_memory;
    uint16_t m_memorySize;
    uint16_t m_memoryAddress;

    
};
