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

#include "Smartcard.h"
#include <Arduino.h>

Smartcard* Smartcard::m_instance {nullptr};

Smartcard::Smartcard()
  :m_i2c(nullptr),
   m_address(0),
   m_state(), 
   m_lastReceiveTimeINms(0),
   m_readingInProgress(false),
   m_readingFinishedFkt(nullptr),
   m_memoryAddress(0)
{
  
}

Smartcard::~Smartcard()
{
}


  
Smartcard* Smartcard::createInstance(i2c_inst_t *i2c, uint32_t baud, uint8_t address, uint8_t sda, uint8_t scl, void (*readingFinishedFkt)(void))
{
  if(nullptr == m_instance)
  {
	  m_instance = new Smartcard();
	  if(nullptr != m_instance)
    {
	    m_instance->m_i2c = i2c;
	    m_instance->m_address = address;
	    m_instance->m_memoryAddress = 0;
      m_instance->m_readingFinishedFkt = readingFinishedFkt;
      gpio_init(sda);
      gpio_set_function(sda, GPIO_FUNC_I2C);
      gpio_pull_up(sda);

      gpio_init(scl);
      gpio_set_function(scl, GPIO_FUNC_I2C);
      gpio_pull_up(scl);

      i2c_init(m_instance->m_i2c, baud);
      i2c_slave_init(m_instance->m_i2c, m_instance->m_address, &(Smartcard::interruptHandler));
    }
  }
  return m_instance;
}

uint8_t* Smartcard::getMemory()
{
  return m_memory;
}

uint16_t Smartcard::getMemoryAddress()
{
  return m_memoryAddress;
}

void Smartcard::setReadingFinishedAddress(uint16_t address)
{
  m_lastMemoryAddress = address;
}

unsigned long Smartcard::getLastReceiveTimeINms()
{
  return m_lastReceiveTimeINms;
}

void Smartcard::setReadingInProgress(bool isInProgress)
{
  m_readingInProgress = isInProgress; 
}

bool Smartcard::isReadingInProgress()
{
  return m_readingInProgress;
}

void Smartcard::interruptHandler(i2c_inst_t *i2c, i2c_slave_event_t event)
{
	if(nullptr != m_instance)
	{
    if(i2c == m_instance->m_i2c)
    { 
      switch (event) 
      {
      case I2C_SLAVE_RECEIVE: // master has written some data
        if (i2cState::WaitingForData == m_instance->m_state) {
            // writes always start with the memory address
            m_instance->m_memoryAddress = i2c_read_byte(i2c) << 8;
            m_instance->m_state = i2cState::WaitingForAddress;
        } else if (i2cState::WaitingForAddress == m_instance->m_state) {
            // writes always start with the memory address
            m_instance->m_memoryAddress += i2c_read_byte(i2c);
            m_instance->m_memoryAddress &= 0x1FFF;
            m_instance->m_state = i2cState::AddressReceived;
        } else {
            // save into memory
            m_instance->m_memory[m_instance->m_memoryAddress] = i2c_read_byte(i2c);
            m_instance->m_memoryAddress++;
            m_instance->m_memoryAddress &= 0x1FFF;
        }
        break;
      case I2C_SLAVE_REQUEST: // master is requesting data
        // load from memory
        i2c_write_byte(i2c, m_instance->m_memory[m_instance->m_memoryAddress]);
        m_instance->m_memoryAddress++;
        m_instance->m_memoryAddress &= 0x1FFF;
        m_instance->m_readingInProgress = true;
        break;
      case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        m_instance->m_state = i2cState::WaitingForData;
        if((m_instance->m_lastMemoryAddress <= m_instance->m_memoryAddress) && m_instance->m_readingInProgress)// overflow of address. Indicates complete reading of buffer
        {
          m_instance->m_readingInProgress = false;
          if(nullptr != m_instance->m_readingFinishedFkt)
          {
            m_instance->m_readingFinishedFkt();
          }
        }
        break;
      default:
        break;
      }
      m_instance->m_lastReceiveTimeINms = millis();
    }
  }
}
