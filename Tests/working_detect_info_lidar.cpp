#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;

int mainBIS(int argc, char** argv)
{
    // Create a communication channel instance
    Result<IChannel*> channelRes = createSerialPortChannel("/dev/ttyUSB0", 115200);
    if (!channelRes) {
        fprintf(stderr, "Failed to create serial channel\n");
        return -1;
    }
    IChannel* channel = *channelRes;

    // Create a LIDAR driver instance
    Result<ILidarDriver*> drv = createLidarDriver();
    if (!drv) {
        fprintf(stderr, "Failed to create LIDAR driver\n");
        return -1;
    }
    ILidarDriver* lidar = *drv;

    auto res = lidar->connect(channel);
    if (SL_IS_OK(res)) {
        sl_lidar_response_device_info_t deviceInfo;
        res = lidar->getDeviceInfo(deviceInfo);
        if (SL_IS_OK(res)) {
            printf("Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
                   deviceInfo.model,
                   deviceInfo.firmware_version >> 8,
                   deviceInfo.firmware_version & 0xffu,
                   deviceInfo.hardware_version);
        } else {
            fprintf(stderr, "Failed to get device information from LIDAR %08x\r\n", res);
        }
    } else {
        fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
    }

    // Clean up
    delete lidar;
    delete channel;

    return 0;
}
