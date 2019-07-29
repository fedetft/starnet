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

#include <list>
#include <functional>
#include "drivers/platform.h"
#include "drivers/transceiver.h"
#include "libs/flopsync2.h"
#include "libs/aes.hpp"

/**
 * Enables debug prints, may break the timing if the slot duration is short
 */
#define DEBUG_PRINT 0

/**
 * The received packet callback will provide this struct
 */
struct ReceivedPacket
{
    ReceivedPacket(const unsigned char *packet, int length,
                   unsigned char sender, signed char rssi)
        : packet(packet), length(length), sender(sender), rssi(rssi) {}
    
    const unsigned char *packet; ///< Pointer to the packet bytes payload
    int length;                  ///< Packet payload length
    unsigned char sender;        ///< nodeId of the sender who published this
    signed char rssi;            ///< RSSI of the packet
};

/**
 * StarNet main class.
 * 
 * StarNet is a low-level publish-subscribe network.
 * 
 * The network master (node 0) takes care of periodically sending clock
 * synchronization packets to synchronize all nodes.
 * Time is divided in slots and every node has its transmission turn in a round
 * robin configuration.
 * The transmission period of each node is slotPeriod * maxNumNodes.
 * All nodes can receive messages of 1 to maxPayloadBytes bytes from the master
 * node, as well as from other nodes they decide to subscribe to.
 * No acknowledge is sent when receiving packets, as nodes sending packets don't
 * know if and how many nodes are listening.
 * 
 * Interaction with StarNet is callback based. Before running StarNet,
 * the application can register to a number of callbacks, that are called
 * whenever certain events occur.
 * Note that callbacks are called in the same thread calling run(), which is
 * also the thread running the network, so keep them short especially for
 * network with small slot period, or the network will not work.
 */
class StarNet
{
public:
    static const int maxPayloadBytes=10; ///< StarNet packets carry 1..10 bytes
    static const int packetLength=16;    ///< Length of all packets transmitted
    
    /**
     * Constructor
     * 
     * For proper operation, all nodes of the same network must have exactly the
     * same parameter except nodeId, which must be unique for each node and less
     * than maxNumNodes.
     * 
     * \param slotDurationNs the duration of one slot of the network in
     * nanoseconds. The minimum and maximum value depend on the transceiver.
     * \param syncPeriodNs the clock sync period in number of slots.
     * Select a period between 10 and 60 seconds for best performance
     * \param maxNumNodes the maximum number of nodes in the network, from 2 to 32
     * \param netId the network ID, broadcast in every packet.
     * Due to some transceivers limitations the first and second byte may not
     * be 0, so 0x1234 is ok, but 0x0034 or 0x1200 is not.
     * \param key the AES key used to encrypt packets
     * \param channel the channel on which we'll transmit.
     * The range goes from 0 to a number that depends on the transceiver.
     * See Transceiver::numChannels.
     * \param nodeId our node ID in the network. If it is 0 we are the network
     * master. Each network must have a network master to connect to.
     * \param sendQueueSize how many packets can be queued for sending before
     * send refuses to accept packets
     */
    StarNet(long long slotDurationNs, long long syncPeriodNs,
            unsigned char maxNumNodes, unsigned short netId,
            const unsigned char key[AES_KEYLEN],
            int channel, unsigned char nodeId, unsigned char sendQueueSize=4);
    
    /**
     * Register a callback to be called when the node connects to the master.
     * If this is the master node, this callback is called as soon as run is called.
     */
    void setConnectCallback(std::function<void ()> cb)    { onConnect=cb; }
    
    /**
     * Register a callback to be called when the node disconnects from the master.
     * If this is the master node, this callback is never called.
     */
    void setDisconnectCallback(std::function<void ()> cb) { onDisconnect=cb; }
    
    /**
     * Register a callback to be called once every slotPeriodNs * maxNumNodes
     * when connected to the network. The callback will be called also when
     * not connected, but the period will be inconsistent.
     */
    void setPeriodicCallback(std::function<void ()> cb)   { onPeriodic=cb; }
    
    /**
     * Register a callback to be called when a packet is received
     */
    void setPacketReceivedCallback(std::function<void (const ReceivedPacket)> cb)
    {
        onReceived=cb;
    }
    
    /**
     * Register a callback to be called when a packet is sent
     */
    void setPacketSentCallback(std::function<void ()> cb) { onSent=cb; }
    
    /**
     * Register a packet to be sent. The packet is not sent immediately,
     * but when it is this node's turn.
     * \param packet packet bytes, that is copied in an internal buffer
     * \param length from 1 to maxPayloadBytes bytes
     * \return 0 on success, -1 if wrong length, -2 if tx queue full
     */
    int sendPacket(const void *packet, int length);
    
    /**
     * \return the total number of entries of the send queue
     */
    int sendQueueSize() const { return config.sendQueueSize; }
    
    /**
     * \return the number of available entries of the send queue
     */
    int sendQueueAvailable() const
    {
        return config.sendQueueSize-sendQueue.size();
    }
    
    /**
     * Subscribe to receive packets from nodeId.
     * Every node is subscribed at least to the master node (node 0).
     */
    void subscribe(unsigned char nodeId);
    
    /**
     * Unsubscribe to receive packets from nodeId.
     * Can't unsubscribe from the master node (node 0).
     */
    void unsubscribe(unsigned char nodeId);
    
    /**
     * \return true if we're subscribed to nodeId
     */
    bool subscribedTo(unsigned char nodeId) const;
    
    /**
     * \return the duration of one slot of the network in nanoseconds.
     */
    long long getSlotDurationNs() const { return config.slotDurationNs; }
    
    /**
     * \return the clock sync period in number of slots.
     */
    long long getSyncPeriodSlots() const{ return config.syncPeriodRounds; }
    
    /**
     * \return the maximum number of nodes in the network, from 2 to 32
     */
    unsigned char getMaxNumNodes() const { return config.maxNumNodes; }
    
    /**
     * \return the network ID, broadcast in every packet.
     */
    unsigned short getNetId() const { return config.netId; }
    
    /**
     * \return the channel on which we'll transmit.
     */
    int getChannel() const { return config.channel; }
    
    /**
     * \return our node ID in the network. If it is 0 we are the network master.
     */
    unsigned char getNodeId() const { return config.nodeId; }
    
    /**
     * \return true if connected to the network
     */
    bool connected() const;
    
    /**
     * \return the clock sync error
     */
    int getSyncError() const { return flopsync2.getSyncError(); }
    
    /**
     * \return the clock sync correction
     */
    int getClockCorrection() const { return flopsync2.getClockCorrection(); }
    
    /**
     * Called to temporarily prevent going in deep sleep between slots (nestable)
     */
    void preventDeepSleep()
    {
        deepSleep++;
    }
    
    /**
     * Celloed to allow back going in deep sleep between slots
     */
    void allowDeepSleep()
    {
        if(deepSleep>0) deepSleep--;
    }
    
    /**
     * Blocking call to run the StarNet
     */
    void run();
    
private:
    StarNet(const StarNet&) = delete;
    StarNet& operator=(const StarNet&) = delete;
    
    struct StarNetConfig
    {
        long long slotDurationNs;
        int channel;
        unsigned short netId, syncPeriodRounds;
        unsigned char maxNumNodes, nodeId, sendQueueSize;
        AES_ctx aesCtx;
    };
    
    struct __attribute__((packed)) Packet
    {
        unsigned char payload[maxPayloadBytes];
        unsigned int payloadLength:4;
        unsigned int isSync:1;
        unsigned int nonce:27;
        unsigned short crc;
    };
    static_assert(sizeof(Packet)==packetLength,"");
    
    StarNetConfig getConfigHelper(long long slotDurationNs, long long syncPeriodNs,
                                  unsigned char maxNumNodes, unsigned short netId,
                                  const unsigned char key[AES_KEYLEN], int channel,
                                  unsigned char nodeId, unsigned char sendQueueSize);
    
    void masterNodeSlot();
    
    void slot(unsigned char slotNumber);
    
    void send();
    
    RecvResult recv(unsigned char slotNumber);
    
    void connect();
    
    bool tryDecodeSyncPacket(const RecvResult& rr);
    
    void sync(const RecvResult& rr);
    
    void miss();
    
    const StarNetConfig config;
    /// Which nodes we are interested in receiving packets from.
    /// For example, 0b1101 means node 0, 2 and 3. Bit 0 should always be set,
    /// as every node needs to listen to the network master.
    unsigned int subscribeMask=0b1;
    unsigned int nonce=0;              ///< Used to prevent replay attacks
    long long correctedSlotDuration=0; ///< slotDurationNs corrected for clock errors
    long long currentSlotTime=0;       ///< time in ns of the current slot start
    unsigned short slotPhase=0;        ///< when we need to sync clock
    unsigned char missedPackets=0;     ///< sync packet we've missed in a row
    unsigned char deepSleep=0;         ///< allow going in deep sleep
    static const unsigned char maxMissedPackets=3;
    
    std::list<Packet> sendQueue;
    Packet rxPacket;
    
    Flopsync2 flopsync2;
    
    std::function<void ()> onConnect;
    std::function<void ()> onDisconnect;
    std::function<void ()> onPeriodic;
    std::function<void (const ReceivedPacket)> onReceived;
    std::function<void ()> onSent;
    
    Platform& platform;
    Transceiver& rtx;
};
