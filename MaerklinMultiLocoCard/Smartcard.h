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

class Smartcard
{
	public:
    struct Config
    {
      i2c_inst_t *i2c{i2c0};
      uint32_t baud{400000};
      uint8_t address{0x50};
      uint8_t sda{PICO_DEFAULT_I2C_SDA_PIN};
      uint8_t scl{PICO_DEFAULT_I2C_SCL_PIN};
      uint8_t cardPin{13};
      unsigned long transmissionTimeoutINms{700};
      unsigned long delayEndOfTransmissionINms{50};
      unsigned long delayUnplugCardINms{100}; 
      void (*readingFinishedCallback)(void){nullptr};
      void (*writeCallback)(void){nullptr};     
    };
 
	  virtual ~Smartcard();

    static Smartcard* createInstance(Config& config);
  
	  static void interruptHandler(i2c_inst_t *i2c, i2c_slave_event_t event);

    uint8_t* getMemory();

    uint16_t getMemoryAddress();

    // set address which finishes reading if it is reached
    void setReadingFinishedAddress(uint16_t address);

    void triggerReading();

    unsigned long getTransmissionTimer() { return m_transmissionTimer;}
    
    typedef enum
    {
      WaitingForData,
      WaitingForAddress,
      AddressReceived
    } I2cState;

    typedef enum
    {
      Idle,
      WaitingForRequest,
      TransmittingData,
      DelayOfTransmissionEnd,
      DelayOfCardUnPlugDetected            
    } TransmissionState;

    void plugCard();
    void unplugCard();

    void cyclic();

	private:
    // Constructor is private to prevent creation
    Smartcard();
    
    static Smartcard* m_instance;

    // i2c interface
    i2c_inst_t* m_i2c;

    // slave adress of 
    uint8_t m_address;

    // internal state for getting write/read adress
    I2cState m_i2cState; 

    // state of transmission
    TransmissionState m_transmissionState;

    
    uint8_t m_cardPin;

    unsigned long m_transmissionTimer;

    unsigned long m_transmissionTimeoutINms;

    unsigned long m_delayEndOfTransmissionINms;

    unsigned long m_delayUnplugCardINms;
    
    void (*m_readingFinishedCallback)(void);

    void (*m_writeCallback)(void);

    uint8_t m_memory[8192];
    uint16_t m_memoryAddress;
    uint16_t m_lastMemoryAddress;

    
};
