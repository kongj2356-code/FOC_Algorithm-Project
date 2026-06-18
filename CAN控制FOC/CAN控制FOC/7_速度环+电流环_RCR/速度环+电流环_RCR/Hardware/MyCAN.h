#ifndef __MYCAN_H
#define __MYCAN_H

void MyCAN_Init(void);
void MyCAN_Transmit(CanTxMsg *TxMessage);
uint8_t MyCAN_ReceiveFlag(void);
void MyCAN_Receive(CanRxMsg *RxMessage);

#endif
