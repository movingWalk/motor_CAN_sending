#ifndef __CANOPEN_HW_APPL_H_
#define __CANOPEN_HW_APPL_H_

#include <stdint.h>

#define CO_BUFLEN            40
#define CO_RX_TIMEOUT        1000

extern void CANOpen_sendFrame(uint16_t cobID, uint8_t* data, uint8_t len);

#endif