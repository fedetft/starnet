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

#include "platform.h"
#include <miosix.h>
#include <kernel/scheduler/scheduler.h>

using namespace miosix;

// IO definitions

using tx   = Gpio<GPIOA_BASE,9>;
using rx   = Gpio<GPIOA_BASE,10>;

using sck  = Gpio<GPIOA_BASE,5>;
using miso = Gpio<GPIOA_BASE,6>;
using mosi = Gpio<GPIOA_BASE,7>;
#if IOMAPPING==0
using cs   = Gpio<GPIOA_BASE,4>;
using dio0 = Gpio<GPIOA_BASE,2>; //PacketSent/PayloadReady
using res  = Gpio<GPIOA_BASE,14>;
#elif IOMAPPING==1
using cs   = Gpio<GPIOB_BASE,11>;
using dio0 = Gpio<GPIOA_BASE,15>; //PacketSent/PayloadReady
using res  = Gpio<GPIOA_BASE,14>;
#elif IOMAPPING==2
using cs   = Gpio<GPIOA_BASE,4>;
using dio0 = Gpio<GPIOA_BASE,12>; //PacketSent/PayloadReady
using res  = Gpio<GPIOA_BASE,11>;
#else
#error IOMAPPING undefined
#endif


// SPI1 code

static inline void csDelay()
{
   //At least 20ns delay, nothing to do for a 24MHz CPU
}

static unsigned char spi1sendRecv(unsigned char x=0)
{
    SPI1->DR=x;
    while((SPI1->SR & SPI_SR_RXNE)==0) ;
    return SPI1->DR;
}

static void initSpi1()
{
    {
        FastInterruptDisableLock dLock;
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
        RCC_SYNC();
    }
    SPI1->CR1=SPI_CR1_SSM  //No HW cs
            | SPI_CR1_SSI
            | SPI_CR1_SPE  //SPI enabled
            | SPI_CR1_BR_0 //SPI clock 24/4=6 MHz
            | SPI_CR1_MSTR;//Master mode
    cs::high();
    cs::mode(Mode::OUTPUT);
    sck::mode(Mode::ALTERNATE);
    miso::mode(Mode::ALTERNATE);
    mosi::mode(Mode::ALTERNATE);
    csDelay();
}

// EXTI code

#if IOMAPPING==0

static Thread *waiting=nullptr;
static long long timestamp=0;
static Rtc *rtcPtr=nullptr;

void __attribute__((naked)) EXTI2_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z16EXTI2HandlerImplv");
    restoreContext();
}

void __attribute__((used)) EXTI2HandlerImpl()
{
    EXTI->PR=EXTI_PR_PR2;
    
    timestamp=rtcPtr->IRQgetValue();
    if(waiting==nullptr) return;
    waiting->IRQwakeup();
    if(waiting->IRQgetPriority()>Thread::IRQgetCurrentThread()->IRQgetPriority())
        Scheduler::IRQfindNextThread();
    waiting=nullptr;
}

static void extiInit(Rtc& rtc)
{
    rtcPtr=&rtc;
    dio0::mode(Mode::INPUT);
    EXTI->RTSR |= EXTI_RTSR_TR2;
    EXTI->IMR |= EXTI_IMR_MR2;
    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_SetPriority(EXTI2_IRQn,3); //High priority
}

static void waitForDio0()
{
    FastInterruptDisableLock dLock;
    if(dio0::value()) return;
    waiting=Thread::IRQgetCurrentThread();
    while(waiting)
    {
        Thread::IRQwait();
        FastInterruptEnableLock eLock(dLock);
        Thread::yield();
    }
}

#elif IOMAPPING==1 || IOMAPPING==2

static Thread *waiting=nullptr;
static long long timestamp=0;
static Rtc *rtcPtr=nullptr;

void __attribute__((naked)) EXTI15_10_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20EXTI15_10HandlerImplv");
    restoreContext();
}

void __attribute__((used)) EXTI15_10HandlerImpl()
{
#if IOMAPPING==1
    EXTI->PR=EXTI_PR_PR15;
#else
    EXTI->PR=EXTI_PR_PR12;
#endif
    
    timestamp=rtcPtr->IRQgetValue();
    if(waiting==nullptr) return;
    waiting->IRQwakeup();
    if(waiting->IRQgetPriority()>Thread::IRQgetCurrentThread()->IRQgetPriority())
        Scheduler::IRQfindNextThread();
    waiting=nullptr;
}

static void extiInit(Rtc& rtc)
{
    rtcPtr=&rtc;
    {
        FastInterruptDisableLock dLock;
        RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    }
    dio0::mode(Mode::INPUT);
#if IOMAPPING==1
    EXTI->RTSR |= EXTI_RTSR_TR15;
    EXTI->IMR |= EXTI_IMR_MR15;
#else
    EXTI->RTSR |= EXTI_RTSR_TR12;
    EXTI->IMR |= EXTI_IMR_MR12;
#endif
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI15_10_IRQn, 3); //High priority
}

static void waitForDio0()
{
    FastInterruptDisableLock dLock;
    if(dio0::value()) return;
    waiting=Thread::IRQgetCurrentThread();
    while(waiting)
    {
        Thread::IRQwait();
        FastInterruptEnableLock eLock(dLock);
        Thread::yield();
    }
}

#else
#error No low power waitForDio0() available!
#endif

static int getDio0()
{
    return dio0::value();
}

static long long getDio0Timestamp()
{
    return timestamp;
}

// ADC code

static void initAdc()
{
    {
        FastInterruptDisableLock dLock;
        const int maxAdcClk=14000000;
        int divider=(SystemCoreClock+maxAdcClk-1)/maxAdcClk;
        RCC->CFGR &= ~RCC_CFGR_ADCPRE;
        if(divider>6)      RCC->CFGR |= 0b11<<14; // /8
        else if(divider>4) RCC->CFGR |= 0b10<<14; // /4
        else if(divider>2) RCC->CFGR |= 0b01<<14; // /2
        
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
        RCC_SYNC();
    }
    ADC1->CR1=0;
    
    ADC1->CR2=ADC_CR2_ADON; //The first assignment sets the bit
    //Calibrate ADC once at powerup
    ADC1->CR2=ADC_CR2_ADON | ADC_CR2_CAL;
    while(ADC1->CR2 & ADC_CR2_CAL) ;
    ADC1->CR2=0; //Keep the ADC OFF to save power
    
    ADC1->SMPR1=0b100100100100100100100100; //Sample for 41.5 cycles
    ADC1->SMPR2=0b100100100100100100100100100100; 
    ADC1->SQR1=0; //Do only one conversion
    ADC1->SQR2=0;
}

static unsigned short adcReadChannel(unsigned char channel)
{
    ADC1->SQR3=channel & 0b11111;
    ADC1->CR2=ADC_CR2_ADON | ADC_CR2_TSVREFE;
    delayUs(5); //Wait for ADC to stabilize
    //Setting the bit while already @ 1 starts conversion
    ADC1->CR2=ADC_CR2_ADON | ADC_CR2_TSVREFE;
    while((ADC1->SR & ADC_SR_EOC)==0) ; //Wait for conversion
    unsigned short result=ADC1->DR; //Read result
    ADC1->CR2=0; //Turn ADC OFF
    return result;
}

//
// class Platform
//

Platform& Platform::instance()
{
    static Platform singleton;
    return singleton;
}

void Platform::absoluteDeepSleep(long long ns)
{
    //Do not leave this pin floating to reduce power
    miso::mode(Mode::INPUT_PULL_UP_DOWN);
    ::absoluteDeepSleep(ns);
    miso::mode(Mode::ALTERNATE);
}

void Platform::csLow()
{
    cs::low();
}

void Platform::csHigh()
{
    cs::high();
    csDelay();
}

unsigned int Platform::spiSendRecv(unsigned char x)
{
    return spi1sendRecv(x);
}

void Platform::waitForDio0()
{
    return ::waitForDio0();
}

int Platform::getDio0()
{
    return ::getDio0();
}

long long int Platform::getDio0Timestamp() const
{
    return ::getDio0Timestamp();
}

unsigned int Platform::trueRand()
{
    unsigned int result=0;
    for(int i=0;i<32;i++)
    {
        result<<=1;
        //ADC16 is temperature sensor, but only take last bit (noise)
        result|= ::adcReadChannel(16) & 1;
    }
    return result;
}

unsigned short Platform::adcReadChannel(unsigned char channel)
{
    return ::adcReadChannel(channel);
}

Platform::Platform() : rtc(Rtc::instance())
{
    {
        FastInterruptDisableLock dLock;
        RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
        AFIO->MAPR=
#if IOMAPPING!=2 || !defined(JTAG_DISABLE_SLEEP)
        //PA14 is by default used for JTAG / SWD, disable
        AFIO_MAPR_SWJ_CFG_2 |
#endif
#ifdef RUN_WITH_HSI
        //HSE is not used, remap PD0/PD1 in order to avoid leaving them floating
        AFIO_MAPR_PD01_REMAP |
#endif
        0;
    }
  
#ifdef _BOARD_STM32F103CX_GENERIC
    //All GPIOs default to input with pulldown
    GPIOA->CRL=0x88888888; GPIOA->CRH=0x88888888;
    GPIOB->CRL=0x88888888; GPIOB->CRH=0x88888888;
    GPIOC->CRH=0x88888888;
    GPIOD->CRL=0x88888888;
    
    GPIOA->ODR=0;
    GPIOB->ODR=0;
    
    //Then, reconfigure the ones we use
    tx::mode(Mode::ALTERNATE);
    rx::mode(Mode::INPUT_PULL_UP_DOWN); rx::pullup();
    //cs, sck, miso, mosi, dio0 will be set by initSpi1 and extiInit

#ifdef LEGACY_PINS
    Gpio<GPIOB_BASE,8>::mode(Mode::INPUT); //PB8 forced to vcc externally
    Gpio<GPIOB_BASE,3>::mode(Mode::INPUT); //dio1 (legacy)
#endif //LEGACY_PINS
#endif //_BOARD_STM32F103CX_GENERIC
    
    initSpi1();
    extiInit(rtc);
    initAdc();
    
    res::mode(Mode::OUTPUT);
    res::high();
    delayUs(100);
    res::low();
    sleep(10000000); //10ms
}
