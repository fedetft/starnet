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

#include "starnet.h"
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <util/crc16.h>

using namespace std;

//
// class StarNet
//

StarNet::StarNet(long long slotDurationNs, long long syncPeriodNs,
                 unsigned char maxNumNodes, unsigned short netId,
                 const unsigned char key[AES_KEYLEN],
                 int channel, unsigned char nodeId, unsigned char sendQueueSize)
    : config(getConfigHelper(slotDurationNs,syncPeriodNs,maxNumNodes,
                             netId,key,channel,nodeId,sendQueueSize)),
      platform(Platform::instance()),
      rtx(Transceiver::instance())
{
    rtx.setChannel(config.channel);
    rtx.setNetworkId(config.netId);
}

int StarNet::sendPacket(const void *packet, int length)
{
    if(length==0 || length>maxPayloadBytes) return -1;
    if(sendQueue.size()>=config.sendQueueSize) return -2;
    Packet txPacket;
    txPacket.payloadLength=length;
    memset(txPacket.payload,0,maxPayloadBytes);
    memcpy(txPacket.payload,packet,length);
    sendQueue.push_back(txPacket);
    return 0;
}

void StarNet::subscribe(unsigned char nodeId)
{
    if(nodeId==0 || nodeId>=config.maxNumNodes) return;
    subscribeMask |= 1<<nodeId;
}

void StarNet::unsubscribe(unsigned char nodeId)
{
    if(nodeId==0 || nodeId>=config.maxNumNodes) return;
    subscribeMask &= ~(1<<nodeId);
}

bool StarNet::subscribedTo(unsigned char nodeId) const
{
    if(nodeId>=config.maxNumNodes) return false;
    return (subscribeMask & (1<<nodeId)) ? true : false;
}

bool StarNet::connected() const
{
    return missedPackets<maxMissedPackets;
}

void StarNet::run()
{
#if DEBUG_PRINT > 0
    iprintf("Starting StarNet with:\n"
            "slotDuration = %lldns\n"
            "syncPeriod   = %lldns\n"
            "netId        = 0x%x\n"
            "channel      = %d\n"
            "maxNumNodes  = %d\n"
            "nodeId       = %d\n",
            config.slotDurationNs,
            config.syncPeriodRounds*config.slotDurationNs*config.maxNumNodes,
            config.netId,config.channel,
            config.maxNumNodes,config.nodeId);
#endif //DEBUG_PRINT

    correctedSlotDuration=config.slotDurationNs;
    currentSlotTime=platform.getTime()+correctedSlotDuration;
    slotPhase=config.syncPeriodRounds-1;
    
    if(config.nodeId==0)
    {
        nonce=platform.trueRand() & 0x07ffffff;
        missedPackets=0;
        if(onConnect) onConnect();
    } else {
        missedPackets=maxMissedPackets;
        //Connect and do the first round without masterNodeSlot()
        connect();
        nonce=(nonce+1) & 0x07ffffff;
        currentSlotTime+=correctedSlotDuration;
        for(unsigned char i=1;i<config.maxNumNodes;i++) slot(i);
        if(onPeriodic) onPeriodic();
    }
    
    for(;;)
    {
        masterNodeSlot();
        for(unsigned char i=1;i<config.maxNumNodes;i++) slot(i);
        if(onPeriodic) onPeriodic();
    }
}

StarNet::StarNetConfig StarNet::getConfigHelper(long long slotDurationNs,
    long long syncPeriodNs, unsigned char maxNumNodes, unsigned short netId,
    const unsigned char key[AES_KEYLEN], int channel, unsigned char nodeId,
    unsigned char sendQueueSize)
{
    StarNet::StarNetConfig config;
    config.sendQueueSize=sendQueueSize;
    config.netId=netId;
    config.channel=max(0,min(Transceiver::numChannels,channel));
    config.maxNumNodes=max<unsigned char>(2,min<unsigned char>(32,maxNumNodes));
    config.nodeId=min<unsigned char>(config.maxNumNodes,nodeId);
    
    // Cap the slot period to the minimum allowed by the transceiver
    long long minimumPeriod=0;
    minimumPeriod+=std::max(Transceiver::txAdvance,Transceiver::rxAdvance);
    minimumPeriod+=(Transceiver::overhead+packetLength)*Transceiver::byteTime;
    config.slotDurationNs=std::max(minimumPeriod,slotDurationNs);
    
    // The sync period must be a multiple of slotPeriodNs * maxNumNodes,
    // find the closest one that matches the requested sync period
    long long minimumSyncPeriod=config.slotDurationNs*config.maxNumNodes;
    syncPeriodNs=(syncPeriodNs+minimumSyncPeriod/2)/minimumSyncPeriod*
        minimumSyncPeriod;
    config.syncPeriodRounds=syncPeriodNs/config.slotDurationNs/config.maxNumNodes;
    
    AES_init_ctx(&config.aesCtx,key);
    return config;
}

void StarNet::masterNodeSlot()
{
    bool thisSlotIsSync=false;
    if(++slotPhase>=config.syncPeriodRounds)
    {
        slotPhase=0;
        thisSlotIsSync=true;
    }
    
    if(config.nodeId==0)
    {
        if(thisSlotIsSync && sendQueue.empty())
        {
            // We need to send a packet to synchronize the network, but we don't
            // have anything to send. This is the only case we send with 0 payload
            Packet txPacket;
            txPacket.payloadLength=0;
            memset(txPacket.payload,0,maxPayloadBytes);
            sendQueue.push_back(txPacket);
        }
        if(sendQueue.empty()==false)
        {
            //Cache len as send() deletes the packet from the queue
            unsigned char len=sendQueue.front().payloadLength;
            sendQueue.front().isSync=thisSlotIsSync;
            send();
            // Don't call the callback if it's the empty packet used for sync
            if(onSent && len>0) onSent();
        }

#if DEBUG_PRINT > 1
        if(thisSlotIsSync) iprintf("sync T=%lldns\n",currentSlotTime);
#elif DEBUG_PRINT > 0
        if(thisSlotIsSync) puts("sync");
#endif //DEBUG_PRINT

    } else {
        auto result=recv(0);
        if(result.error==RecvResult::OK)
        {
            if(thisSlotIsSync) sync(result);
            // Don't call the callback if it's the empty packet used for sync
            if(rxPacket.payloadLength>0 && onReceived)
            {
                onReceived(ReceivedPacket(
                    rxPacket.payload,rxPacket.payloadLength,0,result.rssi));
            }
        } else {
            // Missed sync packet
            if(thisSlotIsSync) miss();
        }
    }
    nonce=(nonce+1) & 0x07ffffff;
    currentSlotTime+=correctedSlotDuration;
}

void StarNet::slot(unsigned char slotNumber)
{
    if(slotNumber==config.nodeId)
    {
        if(sendQueue.empty()==false)
        {
            sendQueue.front().isSync=0;
            send();
            if(onSent) onSent();
        }
    } else {
        if(subscribedTo(slotNumber))
        {
            auto result=recv(slotNumber);
            if(result.error==RecvResult::OK && rxPacket.payloadLength>0 && onReceived)
            {
                onReceived(ReceivedPacket(
                    rxPacket.payload,rxPacket.payloadLength,slotNumber,result.rssi));
            }
        }
    }
    nonce=(nonce+1) & 0x07ffffff;
    currentSlotTime+=correctedSlotDuration;
}

void StarNet::send()
{
    Packet* txPacket=&sendQueue.front();
    txPacket->nonce=nonce;
    txPacket->crc=miosix::crc16(txPacket,packetLength-2);
    AES_ECB_encrypt(&config.aesCtx,reinterpret_cast<unsigned char*>(txPacket));
    if(deepSleep==0)
        platform.absoluteDeepSleep(currentSlotTime-Transceiver::txAdvance);
    else platform.absoluteSleep(currentSlotTime-Transceiver::txAdvance);
    if(onSendRecv) onSendRecv(true);
    auto result=rtx.sendAt(txPacket,packetLength,currentSlotTime);
    if(onSendRecv) onSendRecv(false);
    (void)result;
    sendQueue.pop_front();

#if DEBUG_PRINT > 2
    if(result!=SendResult::TOO_LATE) iprintf("send T=%lldns\n",currentSlotTime);
    else iprintf("send too late T=%lldns\n",currentSlotTime);
#elif DEBUG_PRINT > 0
    if(result==SendResult::TOO_LATE) puts("send too late");
#endif //DEBUG_PRINT
}

RecvResult StarNet::recv(unsigned char slotNumber)
{
    long long w=config.nodeId==0 ? Flopsync2::wMin : flopsync2.getReceiverWindow();
    if(deepSleep==0)
        platform.absoluteDeepSleep(currentSlotTime-Transceiver::rxAdvance-w);
    else platform.absoluteSleep(currentSlotTime-Transceiver::rxAdvance-w);
#if DEBUG_PRINT > 2
    auto b=platform.getTime();
#endif //DEBUG_PRINT
    if(onSendRecv) onSendRecv(true);
    auto result=rtx.recv(&rxPacket,packetLength,currentSlotTime+w);
    if(onSendRecv) onSendRecv(false);
#if DEBUG_PRINT > 2
    auto a=platform.getTime();
    iprintf("recv err=%d rssi=%ddBm len=%d rcvt=%lldns T=%lldns\n",
            result.error,result.rssi,result.size,a-b,
            result.error==RecvResult::OK ? result.timestamp : currentSlotTime);
#endif //DEBUG_PRINT

    if(result.error!=RecvResult::OK) return result;
    // Validate packet
    if(result.size!=packetLength || !result.timestampValid) return RecvResult();
    AES_ECB_decrypt(&config.aesCtx,reinterpret_cast<unsigned char*>(&rxPacket));
    if(rxPacket.nonce!=nonce) return RecvResult();
    if(rxPacket.crc!=miosix::crc16(&rxPacket,packetLength-2)) return RecvResult();
    if(rxPacket.payloadLength>maxPayloadBytes) return RecvResult();
    if(rxPacket.isSync && (slotNumber!=0)) return RecvResult();
    return result;
}

void StarNet::connect()
{
    // When connecting the first time or disconnecting, receive continuously
    // for up to (quickConnectSyncPeriods+0.1) sync periods to (re)connect fast
    const int quickConnectSyncPeriods=3;
    // If we don't connect within quickConnectSyncPeriods, switch to low
    // power connect mode, where we listen for 1.1 sync periods every
    // 1.1*lowPowerConnectRatio sync periods to save power
    const int lowPowerConnectRatio=40;
    
    if(onDisconnect) onDisconnect();
    flopsync2.reset();
    correctedSlotDuration=config.slotDurationNs;
    RecvResult rr;
    bool syncFound=false;
    
    // Quick connect
    auto syncPeriod=config.syncPeriodRounds*config.slotDurationNs*config.maxNumNodes;
    auto timeout=platform.getTime()+syncPeriod*quickConnectSyncPeriods+syncPeriod/10;
#if DEBUG_PRINT > 2
    iprintf("connecting timeout=%lldns T=%lldns\n",timeout,platform.getTime());
#elif DEBUG_PRINT > 0
    puts("connecting");
#endif //DEBUG_PRINT
    do {
        rr=rtx.recv(&rxPacket,packetLength,timeout);
        syncFound=tryDecodeSyncPacket(rr);
#if DEBUG_PRINT > 2
        iprintf("recv good=%d err=%d rssi=%ddBm len=%d T=%lldns\n",
                syncFound,rr.error,rr.rssi,rr.size,
                rr.timestamp==0 ? platform.getTime() : rr.timestamp);
#endif //DEBUG_PRINT
    } while(syncFound==false && rr.error!=RecvResult::TIMEOUT);
    
    // Low power connect
    auto plusDuty=syncPeriod+syncPeriod/10;
    auto minusDuty=plusDuty*(lowPowerConnectRatio-1);
    while(syncFound==false)
    {
        auto timeout=platform.getTime()+minusDuty;
        if(onPeriodic) onPeriodic();
        if(deepSleep==0)
        {
#if DEBUG_PRINT > 0
            iprintf("lowPowerConnect: sleeping for %lldns\n",minusDuty);
#endif //DEBUG_PRINT
            platform.absoluteDeepSleep(timeout);
#if DEBUG_PRINT > 0
            puts("lowPowerConnect: waking up");
#endif //DEBUG_PRINT
        }
        
        do {
            rr=rtx.recv(&rxPacket,packetLength,timeout+plusDuty);
            syncFound=tryDecodeSyncPacket(rr);
#if DEBUG_PRINT > 2
            iprintf("recv good=%d err=%d rssi=%ddBm len=%d T=%lldns\n",
                    syncFound,rr.error,rr.rssi,rr.size,rr.timestamp);
#endif //DEBUG_PRINT
        } while(syncFound==false && rr.error!=RecvResult::TIMEOUT);
    }
    
    nonce=rxPacket.nonce;
    currentSlotTime=rr.timestamp;
    slotPhase=0;
    missedPackets=0;
    if(onConnect) onConnect();
    if(rxPacket.payloadLength>0 && onReceived)
    {
        onReceived(ReceivedPacket(rxPacket.payload,rxPacket.payloadLength,0,rr.rssi));
    }
#if DEBUG_PRINT > 1
    iprintf("connected rssi=%ddBm T=%lldns\n",rr.rssi,currentSlotTime);
#elif DEBUG_PRINT > 0
    iprintf("connected rssi=%ddBm\n",rr.rssi);
#endif //DEBUG_PRINT
}

bool StarNet::tryDecodeSyncPacket(const RecvResult& rr)
{
    if(rr.error!=RecvResult::OK || rr.size!=packetLength || !rr.timestampValid)
        return false;
    AES_ECB_decrypt(&config.aesCtx,reinterpret_cast<unsigned char*>(&rxPacket));
    if(rxPacket.crc!=miosix::crc16(&rxPacket,packetLength-2)) return false;
    return rxPacket.isSync;
}

void StarNet::sync(const RecvResult& rr)
{
    int error=rr.timestamp-currentSlotTime;
    auto result=flopsync2.computeCorrection(error);
    correctedSlotDuration=config.slotDurationNs+
        result.first/config.maxNumNodes/config.syncPeriodRounds;
    missedPackets=0;

#if DEBUG_PRINT > 1
    iprintf("sync e=%dns u=%dns w=%dns rssi=%ddBm T=%lldns\n",
            error,result.first,result.second,rr.rssi,currentSlotTime);
#elif DEBUG_PRINT > 0
    iprintf("sync e=%dns w=%dns rssi=%ddBm\n",
            error,result.second,rr.rssi);
#endif //DEBUG_PRINT
}

void StarNet::miss()
{
    auto result=flopsync2.lostPacket();
    correctedSlotDuration=config.slotDurationNs+
        result.first/config.maxNumNodes/config.syncPeriodRounds;

#if DEBUG_PRINT > 1
    iprintf("sync miss u=%dns w=%dns T=%lldns\n",
            result.first,result.second,currentSlotTime);
#elif DEBUG_PRINT > 0
    iprintf("sync miss w=%dns\n",result.second);
#endif //DEBUG_PRINT
    
    if(++missedPackets>=maxMissedPackets) connect();
}
