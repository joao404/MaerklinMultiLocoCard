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
   m_readingFinishedCallback(nullptr),
   m_writeCallback(nullptr),
   m_memoryAddress(0)
{
  
}

Smartcard::~Smartcard()
{
}


  
Smartcard* Smartcard::createInstance(Config& config)
{
  if(nullptr == m_instance)
  {
	  m_instance = new Smartcard();
	  if(nullptr != m_instance)
    {
	    m_instance->m_i2c = config.i2c;
	    m_instance->m_address = config.address;
	    m_instance->m_memoryAddress = 0;
      m_instance->m_readingFinishedCallback = config.readingFinishedCallback;
      m_instance->m_writeCallback = config.writeCallback;
      gpio_init(config.sda);
      gpio_set_function(config.sda, GPIO_FUNC_I2C);
      gpio_pull_up(config.sda);

      gpio_init(config.scl);
      gpio_set_function(config.scl, GPIO_FUNC_I2C);
      gpio_pull_up(config.scl);

      m_instance->m_cardPin = config.cardPin;
      m_instance->unplugCard();

      i2c_init(m_instance->m_i2c, config.baud);
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

void Smartcard::plugCard()
{
  pinMode(m_cardPin, INPUT);
}

void Smartcard::unplugCard()
{
  pinMode(m_cardPin, OUTPUT);
  digitalWrite(m_cardPin, LOW);
}

void Smartcard::cyclic()
{
  
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
            // call callback for handling data
            if(nullptr != m_instance->m_writeCallback)
            {
              m_instance->m_writeCallback();
            }
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
          if(nullptr != m_instance->m_readingFinishedCallback)
          {
            m_instance->m_readingFinishedCallback();
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
