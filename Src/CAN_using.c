#include <string.h>


#include "stm32f4xx_hal.h"

extern CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef   TxHeader;
uint32_t              TxMailbox;
typedef struct CO_BUFFER{
  uint16_t ID;
  uint8_t data[8];
  uint8_t valid;
  uint16_t timeout;
}CO_BUFFER;
static CO_BUFFER rxBuffer[CO_BUFLEN];

static uint16_t ID;
static uint8_t txData[8];
static volatile uint16_t tx_timeout;


void addRxBuffer(uint16_t ID, uint8_t* data){

  uint16_t i;
  for(i = 0; i < CO_BUFLEN; i++){
    if(rxBuffer[i].valid == 0){
      rxBuffer[i].timeout = CO_RX_TIMEOUT;
      rxBuffer[i].ID = ID;
      memcpy(rxBuffer[i].data, data, 8);
      rxBuffer[i].valid = 1;
      return;
    }
  }
  
}

void timerLoop(){ // Should be call every 1ms
  
  uint16_t i;
  if(tx_timeout != 0) tx_timeout --;
  for(i = 0; i < CO_BUFLEN; i++){
    if(rxBuffer[i].valid == 1){
      if(rxBuffer[i].timeout != 0) rxBuffer[i].timeout --;
      if(rxBuffer[i].timeout == 0) rxBuffer[i].valid = 0;
    }
  }
  
}

void sendFrame_std(uint16_t ID, uint8_t* data, uint8_t len){
  
  TxHeader.StdId = ID;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = len;
  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox);
  while(HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox) == 1);
}

