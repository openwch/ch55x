/********************************** (C) COPYRIGHT ******************************
* File Name          : USBDEV.C
* Author             : WCH
* Version            : V1.0
* Date               : 2014/12/18
* Description        : ISP下载主程序
*                      1，支持串口下载，串口号为0，波特率为4800，由于采用内部晶振，晶振存在误差，所以增加串口累加和，累加和错误进行重发
*                      2，支持USB下载，USB为全速设备
					   3，支持EEPROM编程
					   4，支持芯片型号判断
					   5，下载时需要将P4.6引脚拉低，默认使用内部晶振进行下载
*******************************************************************************/

#include <CH559.H>
#include <string.h>
#include <intrins.h>


 sbit EnableIAP            = P1^3;         // external count input or clock output for timer2


#pragma NOAREGS


/*********************************************************************
 * @fn     ：mDelayus
 *
 * @brief  ：延时函数
 *
 * @param  ：n——延时时间
 *
 * @return ：none
 */
void mDelay20us( UINT16 n )
{
	for( n <<= 3;n;--n ){
		_nop_( );
	}
}

/*******************************************************************************
* Function Name  : main
* Description    : 主函数
*                ：
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
typedef void( *pTaskFn)( void );
pTaskFn tasksArr[1];  

void main( void ) 
{
	UINT16 i=0;

	P4_DIR |= 1<<1;       		// 设置上拉
	P4_OUT |= 1<<1;				// 设置方向输出
	tasksArr[0] = (pTaskFn)(0xC800+0x00);
	
	while(1){
		if( EnableIAP == 0 ){	  // K4
			P4_OUT |= (1<<1);
			mDelay20us(60000);
	 		(tasksArr[0])();
		}
	    i++;
		if( i == 200 ) i = 0;
		if( i == 0 )   P4_OUT |= (1<<1);
		if( i == 100 ) P4_OUT &= ~(1<<1);
		mDelay20us(1000);
	}
}

