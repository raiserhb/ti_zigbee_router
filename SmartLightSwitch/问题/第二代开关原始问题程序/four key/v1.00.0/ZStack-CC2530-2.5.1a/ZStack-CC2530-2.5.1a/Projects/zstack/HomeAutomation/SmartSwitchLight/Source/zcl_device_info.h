#ifndef __ZCL_DEVICE_INFO_H__
#define __ZCL_DEVICE_INFO_H__

#define ZCL_DEVICE_INFO_SEND_EVENT 0x0100
#define ZCL_DEVICE_HEARTBEAT_EVENT 0x0200
#define ZCL_DEVICE_RESET_EVENT     0x0400
#define ZCL_DEVICE_REJOIN_EVENT    0x0800

#define ZCL_HEARTBEAT_PERIOD (60000+(osal_rand() & 0x00FF))
#define ZCL_DEVICE_INFO_TIME (500+(osal_rand() & 0x00FF))
#define ZCL_HEARTBEAT_ENDPOINT 0xEF
#define ZCL_HEARTBEAT_CLUSTERID 0xFF00

#define ZCL_FACTORY_RESET_TIMECOUNT 10
#define ZCL_FACTORY_RESET_HINT_TIMECOUNT (ZCL_FACTORY_RESET_TIMECOUNT + 6)
#define ZCL_FACTORY_RESET_HINT 500
#define ZCL_REJOIN_ATTEMPTS 6
#define ZCL_REJOIN_TIMEOUT 40000

extern void zclDeviceInfoInit(void);
extern void zclSendDeviceInfo(void);
extern afStatus_t zclSendHeartbeat(void);
extern void zclFactoryReset(uint8 DataCleanFlag);
extern afAddrType_t zcl_Coord_nwkAddr;

#endif