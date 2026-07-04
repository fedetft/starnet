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

#include <limits>

// Select frequency range
#define FREQ_RANGE 433
// #define FREQ_RANGE 868

// NOTE: although not written in the datasheet, RFM69W only has the power
// amplifier PA0, while RFM69HW doesn't have it, and instead has PA1 and PA2.
// So if you're using RFM69W comment this. If configured improperly, the
// outcome is very poor radio range
#define RFM69_USE_PA1

// Enable high sensitivity RX, may saturate if nodes are too close
//#define RFM69_HI_SENSITIVITY_RX

// Use a conservative RSSI threshold to return timeout early in recv, saving
// power at the expense of sensitivity
#define RFM69_RECV_USE_RSSI

//Forward decls
class Platform;

/// Pass this to recv() to disable timeout
const long long infiniteTimeout=std::numeric_limits<long long>::max();

/**
 * This class is returned by the recv member function of the transceiver
 */
class RecvResult
{
public:
    /**
     * Possible outcomes of a receive operation
     */
    enum ErrorCode
    {
        OK,             ///< Receive succeeded
        TIMEOUT,        ///< Receive timed out
        TOO_LONG,       ///< Packet was too long for the given buffer
        CRC_FAIL,       ///< Packet failed CRC check
        UNINITIALIZED   ///< RecvResult not valid
    };
    
    RecvResult() : timestamp(0), rssi(-128), size(0), error(UNINITIALIZED), timestampValid(false) {}
    
    long long timestamp; ///< Packet timestamp. It is the time point when the
                         ///< first bit of the packet preamble is received
    signed char rssi;    ///< RSSI of received packet
    unsigned char size;  ///< Packet payload size in bytes
    ErrorCode error;     ///< Possible outcomes of the receive operation
    bool timestampValid; ///< True if timestamp is valid
};

/**
 * Possible outcomes of a send operation
 */
enum class SendResult
{
    OK,       ///< Send succeded
    SIZE_ERR, ///< Packet size is not within range 
    TOO_LATE  ///< sendAt called too late
};

/**
 * This singleton class allows to interact with the radio transceiver
 * This class is not safe to be accessed by multiple threads simultaneously.
 */
class Rfm69
{
public:
    
#if FREQ_RANGE==433
    static const int minFrequency=433070; ///< Minimum supported frequency (kHz)
    static const int maxFrequency=434770; ///< Maximum supported frequency (kHz)
    static const int numChannels=69;      ///< Number of available radio channels
#elif FREQ_RANGE==868
    static const int minFrequency=868600; ///< Minimum supported frequency (kHz)
    static const int maxFrequency=869700; ///< Maximum supported frequency (kHz)
    static const int numChannels=45;      ///< Number of available radio channels
#else
#error No FREQ_RANGE
#endif
    
    /// TS_OSC(500us)+TS_FS(150us)+TS_TR(160us)+slack(190us) 
    static const int txAdvance=1000000;
    /// TS_OSC(500us)+TS_FS(150us)+(Tana+Tcf+Tdcc)[RxBw=20.8kHz](1100us)+slack(250us)
    static const int rxAdvance=2000000;
    
    /// Overhead in bytes to send a packet.
    /// 3 byte preamble 2 byte netId 1 byte len 2 byte CRC
    static const int overhead=8;

    /// Time in nanoseconds to send a byte (8/4800*1e9)
    static const long long byteTime=1666667;
    
    /**
     * \return an instance of the transceiver (singleton)
     */
    static Rfm69& instance();
    
    /**
     * Set TX/RX frequency. You can use setChannel() instead.
     * \param freq frequency in kHz, must be in the range
     * [minFrequency, maxFrequency]
     */
    void setFrequency(int freq);
    
    /**
     * Set TX/RX channel. You can use setFrequency() instead.
     * \param ch channel, from 0 to 68 in the 433MHz range,
     *                and from 0 to 44 in the 868MHz range.
     */
    void setChannel(int ch) { setFrequency(minFrequency+ch*25); }
    
    /**
     * Set network id. Only packets with matching network ID will be received.
     * \param netId network id.
     * 
     * NOTE: Due to a transceiver limitation, neither the upper or lower byte
     * can be 0
     */
    void setNetworkId(const unsigned short netId);
    
    /**
     * Send a packet without checking for a clear channel. Packet transmission
     * time point is managed in software and as such is not very time
     * deterministic
     * \param pkt pointer to the packet bytes
     * \param size packet size in bytes. If CRC is enabled the maximum size is
     * 125 bytes (the packet must not contain the CRC, which is appended
     * by this class). If CRC is disabled, maximum length is 127 bytes
     * \returns SendResult
     * 
     * NOTE: transmit current consumption at 3.3V (STM32@24MHz+RFM69) is ~55mA
     */
    SendResult sendNow(const void *pkt, int size);

    /**
     * Send a packet at a precise time point. This function needs to be called
     * at least txAdvance before the time when the packet needs to be sent, because
     * the transceiver takes this time to transition to the TX state.
     * \param pkt pointer to the packet payload bytes
     * \param size packet payload size in bytes. From 1 to 64 bytes
     * \param when the time point when the first bit of the preamble of the
     * packet is to be transmitted on the wireless channel
     * \returns SendResult
     * 
     * NOTE: transmit current consumption at 3.3V (STM32@24MHz+RFM69) is ~55mA
     */
    SendResult sendAt(const void *pkt, int size, long long when);
    
    /**
     * \param pkt pointer to a buffer where the packet payload will be stored
     * \param size size of the buffer
     * \param timeout absolute time after which the function returns
     * NOTE: the timeout is not strict, if the timeout occurs while a packet
     * is being received, the transceiver will carry on receiving.
     * \return a RecvResult class with the information about the operation
     * outcome
     * 
     * Starting the receiver takes rxAdvance. After which the RSSI timeout starts,
     * which is the timeout parameter. If no packets starts by then, timeout is
     * returned.
     * 
     * NOTE: timestamp jitter seems to be +/-100us
     * NOTE: receive current consumption at 3.3V (STM32@24MHz+RFM69) is
     * ~21mA without RFM69_HI_SENSITIVITY_RX
     * ~23mA with RFM69_HI_SENSITIVITY_RX
     */
    RecvResult recv(void *pkt, int size, long long timeout);

    // NOTE: when not transmitting nor receiving the transceiver draws just a
    // few uA. If the STM32 is also put in deep sleep, the current consumption
    // can be ~21uA (HT7333+STM32+RFM69)
    
private:
    /**
     * Constructor
     */
    Rfm69();
    Rfm69(const Rfm69&)=delete;
    Rfm69& operator= (const Rfm69&)=delete;
    
    /**
     * Write to an RFM69 register
     * \param reg register number
     * \param value value to write
     */
    void writeReg(unsigned char reg, unsigned char value);
    
    /**
     * Read from an RFM69 register
     * \param reg register number
     * \return register content
     */
    unsigned char readReg(unsigned char reg);

    Platform& platform;
};
