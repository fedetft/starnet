/***************************************************************************
 *   Copyright (C) 2018, 2019 by Terraneo Federico                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/ 

#pragma once

#include <miosix.h>
#include <drivers/stm32_rtc.h>

// This driver can work with two IO mappings of the stm32:
// IOMAPPING=0: sck=PA5 miso=PA6 mosi=PA7 CS=PA4  DIO0=PA2,  res=PA14
// IOMAPPING=1: sck=PA5 miso=PA6 mosi=PA7 CS=PB11 DIO0=PA15, res=PA14
// anything else and you'll have to modify the driver
#ifndef IOMAPPING
#define IOMAPPING 1
#endif //IOMAPPING

// Configure legacy pins in previous board version
//#define LEGACY_PINS

// Utilities

constexpr long long seconds(int n)      { return n * 1000000000LL; }
constexpr long long milliseconds(int n) { return n * 1000000LL; }
constexpr long long microseconds(int n) { return n * 1000LL; }

/**
 * A class to abstract the underlying platform
 */
class Platform
{
public:
    /**
     * \return an instance to the platform (singleton)
     */
    static Platform& instance();
    
    // Time management
    long long getTime()              { return rtc.getValue(); }
    void sleep(long long ns)         { rtc.wait(ns); }
    void absoluteSleep(long long ns) { rtc.absoluteWait(ns); }
    void absoluteDeepSleep(long long ns);
    
    // Low-level stuff used by the transceiver implementation
    void csLow();
    void csHigh();
    unsigned int spiSendRecv(unsigned char x=0);
    void waitForDio0();
    int getDio0();
    long long getDio0Timestamp() const;
    
    // Crypto-level random number generation
    unsigned int trueRand();
    
    // ADC
    unsigned short adcReadChannel(unsigned char channel);
    
private:
    Platform();
    
    miosix::Rtc& rtc;
};
