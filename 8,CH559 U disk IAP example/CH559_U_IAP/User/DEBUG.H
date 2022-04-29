/* 调试 */
/* 提供printf子程序和延时函数 */

#ifndef	__DEBUG_H__
#define __DEBUG_H__

//定义函数返回值
#ifndef  SUCESS
#define  SUCESS  1
#endif
#ifndef  FAIL
#define  FAIL    0
#endif

//定义定时器起始
#ifndef  START
#define  START  1
#endif
#ifndef  STOP
#define  STOP    0
#endif

extern  void	CfgFsys( );                        //CH559时钟选择和配置
extern  void	mDelayuS( UINT16 n );              // 以uS为单位延时
extern  void	mDelaymS( UINT16 n );              // 以mS为单位延时
extern  void  	CH559UART0Alter();                 //CH559串口0引脚映射到P0.2/P0.3
extern  void	mInitSTDIO( );                      //T1作为波特率发生器
extern  UINT8   CH559UART0RcvByte( );              //CH559 UART0查询方式接收一个字节
extern  void  	CH559UART0SendByte(UINT8 SendDat); //CH559UART0发送一个字节
#endif
