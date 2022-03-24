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

std::vector<Smartcard> Smartcard::m_instances;

Smartcard::Smartcard()
  :m_i2c(nullptr),
   m_address(0),
   m_activationPin(-1),
   m_state(), 
   readingInProgress(false),
   m_memory(nullptr),
   m_memorySize(0),
   m_memoryAddress(0)
{
  
}

Smartcard::~Smartcard()
{
  if(m_memory)
  {
    delete m_memory;
  }
}
  
Smartcard* Smartcard::createInstance(uint16_t size, i2c_inst_t *i2c, uint32_t baud, uint8_t address, uint8_t sda, uint8_t scl, int8_t activationPin)
{
  auto& card = Smartcard::m_instances.emplace_back(Smartcard());
  card.m_i2c = i2c;
  card.m_address = address;
  card.m_activationPin = activationPin;
  card.m_memory = new uint8_t(size);
  if(card.m_memory)
  {
    card.m_memorySize = size;
    card.m_memoryAddress = 0;

    if(-1 != card.m_activationPin)
    {
      pinMode(card.m_activationPin, OUTPUT);
      digitalWrite(card.m_activationPin, HIGH);// card is marked as not pluged
    }

    gpio_init(sda);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_pull_up(sda);

    gpio_init(scl);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(scl);

    i2c_init(card.m_i2c, baud);
    i2c_slave_init(card.m_i2c, card.m_address, &(Smartcard::interruptHandler));
    return &card;
  }
  return nullptr;
}



uint8_t* Smartcard::getMemory()
{
  return m_memory;
}

void Smartcard::triggerActivation()
{
  readingInProgress = true;
  if(-1 != m_activationPin)
  {
    digitalWrite(m_activationPin, LOW);// card is pluged
  }
}

bool Smartcard::isReadingInProgress()
{
  return readingInProgress;
}

void Smartcard::interruptHandler(i2c_inst_t *i2c, i2c_slave_event_t event)
{
  for(int i = 0; i < Smartcard::m_instances.size(); i++)
  //for (auto card : Smartcard::m_instances)
  {
    auto& card = Smartcard::m_instances.at(i);
    if(i2c == card.m_i2c)
    {    
      switch (event) 
      {
      case I2C_SLAVE_RECEIVE: // master has written some data
        if (i2cState::WaitingForData == card.m_state) {
            // writes always start with the memory address
            card.m_memoryAddress = i2c_read_byte(i2c) << 8;
            card.m_state = i2cState::WaitingForAddress;
        } else if (i2cState::WaitingForAddress == card.m_state) {
            // writes always start with the memory address
            card.m_memoryAddress += i2c_read_byte(i2c);
            card.m_memoryAddress &= 0x1FFF;
            card.m_state = i2cState::AddressReceived;
        } else {
            // save into memory
            card.m_memory[card.m_memoryAddress] = i2c_read_byte(i2c);
            card.m_memoryAddress++;
            card.m_memoryAddress &= 0x1FFF;
        }
        break;
      case I2C_SLAVE_REQUEST: // master is requesting data
        // load from memory
        i2c_write_byte(i2c, card.m_memory[card.m_memoryAddress]);
        card.m_memoryAddress++;
        card.m_memoryAddress &= 0x1FFF;
        if(0 == card.m_memoryAddress)// overflow of address. Indicates complete reading of buffer
        {
          if(-1 != card.m_activationPin)
          {
            digitalWrite(card.m_activationPin, HIGH);// card is not pluged anymore
            card.readingInProgress = false;
          }
        }
        break;
      case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        card.m_state = i2cState::WaitingForData;
        break;
      default:
        break;
      }
      break;
    }
  }
}
