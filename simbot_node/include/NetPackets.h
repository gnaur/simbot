#ifndef NETPACKETS_H
#define NETPACKETS_H
#pragma once
/// Network packet header, used in all network packets
typedef  struct net_packet_header_t
{  
  uint8_t seq_num;    ///< 8 bit sequence number
  uint8_t reserved1;  ///< pad byte
  uint8_t reserved2;  ///< pad byte
  uint8_t reserved3;  ///< pad byte

} __attribute__((packed)) NetPacketHeader;


typedef struct {
    NetPacketHeader header;
    float acc[3];
    float gyro[3];
    uint32_t accTS;
    uint32_t gyroTS;
} __attribute__((packed)) IMU_PACKET;

typedef enum MainComsInCmd_t : uint32_t {
	CMD_NULL=0,
	CMD_PING=1,
	CMD_MTR_EN=2,
	CMD_MTR_DISABLE=4
} MainComsInCmd;

typedef enum MainComsOutEvent_t : uint32_t {
	EVT_NONE=0,
	EVT_ESTOP=1
} MainComsOutEvent;

typedef struct {
    NetPacketHeader header;
    MainComsInCmd cmd;
    uint32_t arg;

} __attribute__((packed)) MAIN_COMS_IN_PACKET;

typedef struct {
    NetPacketHeader header;
    MainComsOutEvent event;
    uint32_t arg;

} __attribute__((packed)) MAIN_COMS_OUT_PACKET;

#endif // NETPACKETS_H

