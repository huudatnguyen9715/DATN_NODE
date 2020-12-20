/*****************************************************************************
  *Ten Tep          :     user_delay.c
  *Ngay             :     16/06/2014
  *Tac Gia          :     MinhHa R&D Team
  *Cong Ty          :     MinhHaGroup
  *Webside          :     mcu.banlinhkien.vn
  *Phien Ban        :     V1.0
  *Tom Tat          :     Dinh nghia cac ham tao delay
  ******************************************************************************/

#include "user_delay.h"
#if (__USER_DELAY_H !=16062014)
    #error "Include Sai #File user_delay.h"
#endif

static uint8_t  fac_us=0;
static uint16_t fac_ms=0;

/*******************************************************************************
Noi Dung    : Khoi tao Systick.
Tham Bien   : SYSCLK: Tan so hoat dong cua he thong.
Tra Ve      : Khong.
********************************************************************************/

void Delay_init(uint8_t SYSCLK)
{
	SysTick->CTRL&=0xfffffffb;
	fac_us=SYSCLK/8;		    
	fac_ms=(uint16_t)fac_us*1000;
}		

/*******************************************************************************
Noi Dung    : Tao dinh thoi theo don vi ms.
Tham Bien   : nms: Thoi gian can dinh thoi.
Tra Ve      : Khong.
********************************************************************************/

void Delay_ms(uint16_t nms)
{	 		  	  
	uint32_t temp;		   
	SysTick->LOAD=(uint32_t)nms*fac_ms;
	SysTick->VAL =0x00;           
	SysTick->CTRL=0x01 ;         
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16))); 
	SysTick->CTRL=0x00;       
	SysTick->VAL =0X00;       	    
}  

/*******************************************************************************
Noi Dung    : Tao dinh thoi theo don vi us.
Tham Bien   : nus: Thoi gian can dinh thoi.
Tra Ve      : Khong.
********************************************************************************/

void Delay_us(uint32_t nus)
{		
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us;   		 
	SysTick->VAL=0x00;        
	SysTick->CTRL=0x01 ;      	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));   
	SysTick->CTRL=0x00;       
	SysTick->VAL =0X00;        
}

/*------------------------------KET THUC FILE-------------------------------
 ______________________________MinhHa R&D Team______________________________*/




































