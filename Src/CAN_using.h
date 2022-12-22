#include <stdint.h>

typedef enum CO_Status{
  CO_OK,
  CO_TIMEOUT,
  CO_ERROR,
}CO_Status;

extern void addRxBuffer(uint16_t ID, uint8_t* data);
extern void timerLoop();
#endif

#ifndef __CANOPEN_HW_APPL_H_
#define __CANOPEN_HW_APPL_H_

#include <stdint.h>

#define CO_BUFLEN            40
#define CO_RX_TIMEOUT        1000

extern void sendFrame_std(uint16_t ID, uint8_t* data, uint8_t len);

#endif