/*****************************************************************************
  *Ten Tep          :     user_delay.h
  *Ngay             :     16/06/2014
  *Tac Gia          :     MinhHa R&D Team
  *Cong Ty          :     MinhHaGroup
  *Webside          :     mcu.banlinhkien.vn
  *Phien Ban        :     V1.0
  *Tom Tat          :     Khai bao cac ham tao delay
  ******************************************************************************/
	
#ifndef __USER_DELAY_H
#define __USER_DELAY_H    16062014

#include "main.h"
#include "stm32f1xx_hal.h"	

/*******************************************************************************
Noi Dung    : Khoi tao Systick.
Tham Bien   : SYSCLK: Tan so hoat dong cua he thong.
Tra Ve      : Khong.
********************************************************************************/

void Delay_init(uint8_t SYSCLK);

/*******************************************************************************
Noi Dung    : Tao dinh thoi theo don vi ms.
Tham Bien   : nms: Thoi gian can dinh thoi.
Tra Ve      : Khong.
********************************************************************************/

void Delay_ms(uint16_t nms);

/*******************************************************************************
Noi Dung    : Tao dinh thoi theo don vi us.
Tham Bien   : nus: Thoi gian can dinh thoi.
Tra Ve      : Khong.
********************************************************************************/

void Delay_us(uint32_t nus);

#endif
/*------------------------------KET THUC FILE-------------------------------
 ______________________________MinhHa R&D Team______________________________*/

