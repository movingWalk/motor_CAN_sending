#ifndef __CANOPEN_BATCH_H__
#define __CANOPEN_BATCH_H__

#include "CANOpen.h"

// 0x22 PDO Objects
extern CO_PDOStruct readPDO_0x22;
extern CO_PDOStruct sendPDO_0x22;
extern int32_t OD_0x22_target_position; // Position Control target value
extern int32_t OD_0x22_actual_position; // Position Control actual value

// 0x23 PDO Objects
extern CO_PDOStruct readPDO_0x23;
extern CO_PDOStruct sendPDO_0x23;
extern int32_t OD_0x23_target_position; // Position Control target value
extern int32_t OD_0x23_actual_position; // Position Control actual value

// 0x31 PDO Objects
extern CO_PDOStruct readPDO_0x31;
extern CO_PDOStruct sendPDO_0x31;
extern int32_t OD_0x31_actual_speed; // Speed Control actual value
extern int32_t OD_0x31_target_speed; // Speed Control target value

extern void CANOpen_batch_Init();
extern void CANOpen_batch_0x22();
extern void CANOpen_batch_0x23();
extern void CANOpen_batch_0x31();
extern void CANOpen_batch_Start();

#endif