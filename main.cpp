
#include <cstdio>
#include "miosix.h"
#include "starnet/drivers/platform.h"
#include "starnet/drivers/transceiver.h"

using namespace std;
using namespace miosix;

int main()
{
    auto& platform=Platform::instance();
    auto& rtx=Transceiver::instance();
    rtx.setChannel(34);
    rtx.setNetworkId(0x0101);
    
    for(;;)
    {
        const unsigned char data[]={1,2,3,4};
        rtx.sendNow(data,sizeof(data));
        iprintf("Packet transmitted\n");
        platform.sleep(1000000000); //1s
    }
}
