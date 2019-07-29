
#include <cstdio>
#include "starnet/starnet.h"

const unsigned char key[AES_KEYLEN]={0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5};
StarNet starNet(
    milliseconds(1000),// slotDurationNs
    seconds(30),       // syncPeriodNs
    10,                // maxNumNodes
    0x1234,            // netId
    key,               // key
    34,                // channel
    0                  // nodeId
);

void onPacketReceived(ReceivedPacket rp)
{
    iprintf("packet received sender=%d len=%d rssi=%ddBm\n",
            rp.sender,rp.length,rp.rssi);
}

void periodicCallback()
{
    puts("---");
}

int main()
{
    starNet.subscribe(1);
    starNet.setPacketReceivedCallback(onPacketReceived);
    starNet.setPeriodicCallback(periodicCallback);
    starNet.run();
}
