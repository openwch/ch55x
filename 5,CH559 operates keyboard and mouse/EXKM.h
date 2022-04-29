/********************************** (C) COPYRIGHT *******************************
* File Name          : EXKM.H
* Author             : ZFL
* Version            : V1.0
* Date               : 2014/11/10
* Description        : FOR lianhongtai.  
*******************************************************************************/



/******************************************************************************/
#ifndef        TRUE
#define        TRUE        1
#define        FALSE       0
#endif
#ifndef        NULL
#define        NULL        0
#endif
#ifndef        BOOL
#define        BOOL        unsigned char
#endif
#ifndef        u8C
#define        u8C         unsigned char  code
#endif
#ifndef        u8XV
#define        u8XV        unsigned char volatile xdata
#endif
#ifndef        u8
#define        u8          unsigned char
#define        u16         unsigned short
#endif
#ifndef        u8D
#define        u8D         unsigned char  data 
#define        u16D        unsigned short data
#define        u32D        unsigned long  data
#endif
#ifndef        u8I
#define        u8I         unsigned char  idata 
#define        u16I        unsigned short idata 
#endif
#ifndef        u8X
#define        u8X         unsigned char  xdata
#define        u16X        unsigned short xdata
#endif

// type of initialize device
#ifndef DEC_KEY
#define DEC_KEY               		0x30           // initialize key success(bios)
#define DEC_MOUSE             		0x31           // initialize mouse success(bios)
#define DEC_MOUSE_BIOS         		0x32           // initialize mouse success(bios)
#endif


// 库初始化函数
void KM_LibInit( void );
// 库程序调用函数，主要查询设备连接，设备断开等
u8	 KM_ProcessSystem( );
// 用于键盘点灯操作。
void KM_HostSetReport( u8 report );
// 用于获取键值函数
// index - 端口索引号（0/1）
// GetDatBuf - 用于存放键值的缓冲区
// *len - 获取键值的数据长度，键盘为8，鼠标为4
u8   KM_HostGetData( u8 index,u8 *GetDatBuf, u8 *len ,u8 *SourceDatBuf,u8 *SourceLen);
// 查询端口设备类型
// index - 端口索引号（0/1）
// GetDatBuf - 用于存放类型的缓冲区 接口(B)+类型(B)
// *len - 获取的数据长度
void KM_TypeQuery( u8 RootHubIndex,u8 *GetDatBuf,u8 *len );