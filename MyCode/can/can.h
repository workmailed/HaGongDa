#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
 
//CAN����RX0�ж�ʹ��
#define CAN_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.								    
										 							 				    
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
 
u8 Can_Send_Msg(int* msg,u8 len);						//��������

u8 Can_Receive_Msg(u8 *buf);							//��������

void can_send(void);
typedef struct
{
	u8 ShiNeng_flag;
	u8 Can_flag;
	u8 usart_flag;
	u8 change_flag;
	u8 guzhang_flag;
}deflag;
extern deflag flag;
extern int cansend[4];
#endif
