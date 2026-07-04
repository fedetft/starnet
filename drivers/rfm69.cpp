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

#include "rfm69.h"
#include "platform.h"
#include <algorithm>

using namespace std;

//
// class Rfm69
//

Rfm69& Rfm69::instance()
{
    static Rfm69 singleton;
    return singleton;
}

void Rfm69::setFrequency(int freq)
{
    if(freq<minFrequency || freq>maxFrequency) return; //ISM band limit
#if FREQ_RANGE==433
    const int baseFreq=7094272; //433MHz
    const int base=433000;
#elif FREQ_RANGE==868
    const int baseFreq=14221312; //868MHz
    const int base=868000;
#else
#error No FREQ_RANGE
#endif
    int f=baseFreq+((freq-base)*16384+500)/1000;
    writeReg(0x07,(f>>16) & 0xff);
    writeReg(0x08,(f>> 8) & 0xff);
    writeReg(0x09, f      & 0xff);
}

void Rfm69::setNetworkId(const unsigned short netId)
{
    unsigned char upper=netId>>8;
    unsigned char lower=netId & 0xff;
    if(upper==0) upper=1;
    if(lower==0) lower=1;
    writeReg(0x2f,lower);
    writeReg(0x30,upper);
}

SendResult Rfm69::sendNow(const void* pkt, int size)
{
    if(size<1 || size>64) return SendResult::SIZE_ERR;
    auto *packet=reinterpret_cast<const unsigned char *>(pkt);
    
    writeReg(0x25,0x00); //DioMapping1=DIO0:PacketSent
    writeReg(0x3c,size); //FifoThresh
    
    platform.csLow();
    platform.spiSendRecv(0x80); //Write reg 0
    platform.spiSendRecv(size); //Length byte for variable length packets
    for(int i=0;i<size;i++) platform.spiSendRecv(packet[i]);
    platform.csHigh();
    
    writeReg(0x01,0x0c); //OpMode=TX
    platform.waitForDio0();
    writeReg(0x01,0x00); //OpMode=SLEEP
    return SendResult::OK;
}

SendResult Rfm69::sendAt(const void* pkt, int size, long long when)
{
    if(size<1 || size>64) return SendResult::SIZE_ERR;
    auto *packet=reinterpret_cast<const unsigned char *>(pkt);
    
    //NOTE: when starting TX mode the transmitter will emit a "pre-preamble"
    //until the start condition is met, after which it will send the programmed
    //preamble bytes and carry on sending the rest of the packet. Thus if
    //sendAt is called very early, we need to wait till txAdvance not to waste
    //power and occupy the channel
    platform.absoluteSleep(when-txAdvance);
    writeReg(0x25,0x00); //DioMapping1=DIO0:PacketSent
    writeReg(0x01,0x0c); //OpMode=TX
    writeReg(0x3c,size); //FifoThresh
    platform.csLow();
    platform.spiSendRecv(0x80); //Write reg 0
    platform.spiSendRecv(size); //Length byte for variable length packets
    for(int i=0;i<size-1;i++) platform.spiSendRecv(packet[i]); //All but the last byte
    
    platform.sleep(870000); //TS_OSC (500us) + TS_FS (150us) + TS_TR (160us)
    if(platform.getTime()>when)
    {
        platform.csHigh();
        writeReg(0x01,0x00); //OpMode=SLEEP, this clears FIFO
        return SendResult::TOO_LATE;
    }
    //NOTE: the RFM69 does not have a way to trigger TX with a GPIO, something
    //that would allow to use an output compare timer output which would allow
    //to transmit without software-induced jitter. Moreover, since when we send
    //the last byte the transceiver is already sending preamble bytes, there is
    //a jitter of around one bit time. Doing the opposite, sending the packet
    //via SPI while the transceiver is in sleep and triggering TX with the
    //OpMode is likely even worse, ans the TS_OSC + TS_FS + TS_TR are given as
    //nominal and maximum, so they are not fixed times. It appears that high
    //time determinism (less than one bit time) with RFM69 is just not possible
    //If you are using this driver in a multithreaded application, make sure
    //the thread accessing the transceiver is the highest priority, otherwise
    //you would also incur scheduling jitter
    platform.absoluteSleep(when);
    platform.spiSendRecv(packet[size-1]); //Last byte triggers TX
    platform.csHigh();
    platform.waitForDio0();
    writeReg(0x01,0x00); //OpMode=SLEEP
    return SendResult::OK;
}

RecvResult Rfm69::recv(void* pkt, int size, long long timeout)
{
    auto *packet=reinterpret_cast<unsigned char *>(pkt);
    RecvResult result;
    result.error=RecvResult::TIMEOUT;
    
    bool hasTimeout=timeout!=infiniteTimeout;
    long long start=platform.getTime()+rxAdvance;
    timeout-=start; //Make timeout a relative time
    if(hasTimeout && timeout<0) return result;
    
    writeReg(0x25,0x40); //DioMapping1=DIO0:PayloadReady
    writeReg(0x01,0x10); //OpMode=RX
    platform.absoluteSleep(start);
    
    //NOTE: what we want for the timeout is to wait until the timeout and if
    //no packet has started, return. If within the timeout a packet
    //has started, we extend the timeout up to the expected packet size.
    //The RFM69 has hardware timeout, but it doesn't quite do what we want.
    //The RxTimeout1 is supposed to end when a packet starts, but in reality
    //it doesn't, as it ends when the RSSI is above threshold, not when the
    //sync address has been received, and due to noise the RSSI goes high almost
    //immediately, making it useless. RxTimeout2 alone can't separate the packet
    //started or not condition, so the timeout would always fall at the set
    //timeout plus the expected packet length, increasing significantly waiting
    //time and power consumption. Moreover, timeouts are only 8 bits.
    //Fianlly, CrcAutoClearOff doesn't do what it says. It's supposed to just
    //try to receive the packet one time after the sync word has been received,
    //and then go to the PayloadReady state, but it doesn't. A previous version
    //of this code which waited till SyncMatch in software and then did a
    //platform.waitForDio0() would sometimes stay there for 10+ seconds in
    //an unknown state wasting power!
    //The only solution has been to reimplement the whole timeout logic in
    //software up to PayloadReady by polling the state of the transceiver every
    //byteTime. This is not power hungry due to the long byteTime.
    //So, for a 3.3ms timeout the total radio RX time is
    //- rxAdvance+timeout                            ~  5.3ms  if Rssi stays low
    //- rxAdvance+timeout+preambleSyncBytes*byteTime ~ 13.7ms  if Rssi goes high
    
    const int preambleSyncBytes=5; // Number of bytes of preamble and sync word
    if(hasTimeout)
    {
        int timeoutBytes=max<int>(1,(timeout+byteTime/2)/byteTime);
        bool packetArrived=false;
        long long wakeup=start;
        for(int i=0;i<timeoutBytes+overhead+size;i++)
        {
            wakeup+=byteTime;
            platform.absoluteSleep(wakeup);
            auto flags1=readReg(0x27);
            //No Rssi and timeout reached, no packet will arrive
            if(i>=timeoutBytes-1 && (flags1 & (1<<3))==0) break;
            //Already extended timeout but no SyncMatch, timeout
            if(i>=timeoutBytes+preambleSyncBytes-1 && (flags1 & (1<<0))==0) break;
            if(platform.getDio0()==0) continue; //PayloadReady?
            packetArrived=true;
            break;
        }
        if(packetArrived==false)
        {
            writeReg(0x01,0x00); //OpMode=SLEEP
            return result;
        }
    } else platform.waitForDio0();
    
    result.timestamp=platform.getDio0Timestamp();
    int rssiBits=readReg(0x24);
    result.rssi=-rssiBits/2;
    auto flags2=readReg(0x28);
    
    writeReg(0x01,0x00); //OpMode=SLEEP
    
    if((flags2 & (1<<1))==0)
    {
        result.error=RecvResult::CRC_FAIL;
        return result;
    } else result.error=RecvResult::OK;
    
    platform.csLow();
    platform.spiSendRecv(0x00); //Read reg 0
    int receivedSize=platform.spiSendRecv();
    const int processingTime=600000; // 600us, measured with an oscilloscope
    result.timestamp-=byteTime*(receivedSize+overhead)+processingTime;
    result.timestampValid=result.timestamp>=start;
    result.size=min(receivedSize,size);
    for(int i=0;i<result.size;i++) packet[i]=platform.spiSendRecv();
    if(receivedSize>size)
    {
        //Drain FIFO
        for(int i=0;i<receivedSize-size;i++) platform.spiSendRecv();
        result.error=RecvResult::TOO_LONG;
    }
    platform.csHigh();
    return result;
}

Rfm69::Rfm69() : platform(Platform::instance())
{
    /*
     * Default config:
     * 4800bit/s data rate (default)
     * 3 byte preamble (default)
     * 2 byte network ID (0x01 0x01)
     * Variable length packets, from 1 to 64 byte (+1byte length byte)
     * 2 byte CRC enabled
     * TX frequency 433.920MHz, power 10dBm
     */
    setChannel(34);
#ifndef RFM69_USE_PA1
    writeReg(0x11,0x80 | 28);   //PaLevel=Pa0On | 10dBm
#else
    writeReg(0x11,0x40 | 25);   //PaLevel=Pa1On | 10dBm
#endif
#ifdef RFM69_HI_SENSITIVITY_RX
    writeReg(0x58,0x2d);
#endif
    writeReg(0x18,0x88);        //Lna         (recomended)
    writeReg(0x06,0xa4);        //Fdev 10kHz
    writeReg(0x19,0x54);        //RxBw 20.8kHz dcc cutoff 4%
    writeReg(0x1a,0x8b);        //AfcBw       (recomended)
    writeReg(0x25,0x00);        //DioMapping1=DIO0:PacketSent
    writeReg(0x26,0x07);        //DioMapping2 (recomended)
#ifdef RFM69_RECV_USE_RSSI
    writeReg(0x29,90*2);        //-90dBm, below -95dBm cause spurious triggers
#else //RFM69_RECV_USE_RSSI
    writeReg(0x29,0xe4);        //RssiThresh  (recomended)
#endif //RFM69_RECV_USE_RSSI
    writeReg(0x2e,0x80 | 1<<3); //SyncConfig=SyncOn | 2 byte sync word (netId)
    setNetworkId(0x0101);       //Default netId
    writeReg(0x37,0x98);        //PacketConfig1=variable length, crc on, no nodeId
                                //stop receiving and raise PayloadReady if crc fail
    writeReg(0x38,0x41);        //PayloadLength=65 byte
    writeReg(0x6f,0x30);        //RegTestDagc (recomended)
    writeReg(0x01,0x00);        //OpMode=SLEEP
}

void Rfm69::writeReg(unsigned char reg, unsigned char value)
{
    platform.csLow();
    platform.spiSendRecv(0x80 | reg);
    platform.spiSendRecv(value);
    platform.csHigh();
}

unsigned char Rfm69::readReg(unsigned char reg)
{
    platform.csLow();
    platform.spiSendRecv(reg & 0x7f);
    auto result=platform.spiSendRecv();
    platform.csHigh();
    return result;
}
