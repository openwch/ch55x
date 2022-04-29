
/* 2015.10.12
****************************************
**  Copyright  (C)  W.ch  1999-2015   **
**  Web:  http://www.wch.cn  **
****************************************
*/

/* 支持: FAT32 */

/* CH559 C语言的U盘升级例程: 
1.升级文件名固定"/DOWNLOARD.BIN"
2.当前文件设定地址0xCC00，下载文件指定地址0xCB00
3.进入U盘升级程序后，P4.3口输出低电平，灯点亮；下载完成输出高电平灯灭并回到新的应用程序	
4.跟踪程序打开DEBUG_NOW宏定义即可
*/

#include <CH559.H>
#define	DEBUG_NOW
#ifdef DEBUG_NOW
#include <stdio.h>
#endif
#include "DEBUG.H"
#include "DEBUG.C"
#include <intrins.h>
// #include <string.h>

#include "CH559UFI.H"


// 各子程序返回状态码
#define	ERR_SUCCESS			0x00	// 操作成功
#define	ERR_USB_CONNECT		0x15	/* 检测到USB设备连接事件,已经连接 */
#define	ERR_USB_DISCON		0x16	/* 检测到USB设备断开事件,已经断开 */
#define	ERR_USB_BUF_OVER	0x17	/* USB传输的数据有误或者数据太多缓冲区溢出 */
#define	ERR_USB_DISK_ERR	0x1F	/* USB存储器操作失败,在初始化时可能是USB存储器不支持,在读写操作中可能是磁盘损坏或者已经断开 */
#define	ERR_USB_TRANSFER	0x20	/* NAK/STALL等更多错误码在0x20~0x2F */
#define	ERR_USB_UNSUPPORT	0xFB
#define	ERR_USB_UNKNOWN		0xFE

#define	WAIT_USB_TOUT_200US		200  // 等待USB中断超时时间200uS@Fsys=12MHz
#define	SetUsbSpeed( x )			//操作U盘设备，设为全速，正常需要选择

// 获取设备描述符
UINT8C	SetupGetDevDescr[] = { USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_DEVICE, 0x00, 0x00, sizeof( USB_DEV_DESCR ), 0x00 };
// 获取配置描述符
UINT8C	SetupGetCfgDescr[] = { USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_CONFIG, 0x00, 0x00, 0x04, 0x00 };
// 设置USB地址
UINT8C	SetupSetUsbAddr[] = { USB_REQ_TYP_OUT, USB_SET_ADDRESS, USB_DEVICE_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00 };
// 设置USB配置
UINT8C	SetupSetUsbConfig[] = { USB_REQ_TYP_OUT, USB_SET_CONFIGURATION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// 清除端点STALL
UINT8C	SetupClrEndpStall[] = { USB_REQ_TYP_OUT | USB_REQ_RECIP_ENDP, USB_CLEAR_FEATURE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//获取逻辑单元数LUN
UINT8C	GetMaxLUN[] = {0xA1, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};
//Mass Storage Reset
UINT8C	SetReset[] = {0xA1, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

UINT8X	UsbDevEndp0Size;	/* USB设备的端点0的最大包尺寸 */
UINT8X  DevEndpOUTAddr;
UINT8X  DevEndpINAddr;

//USB设备相关信息表,CH559最多支持2个设备
#define	ROOT_DEV_DISCONNECT		0
#define	ROOT_DEV_CONNECTED		1
#define	ROOT_DEV_FAILED			2
#define	ROOT_DEV_SUCCESS		3

UINT8X	RxBuffer[ MAX_PACKET_SIZE ] _at_ 0x0000 ;  // IN, must even address
UINT8X	TxBuffer[ 512 ] 			_at_ 0x0040 ;  // OUT, must even address
UINT8X	TMPDataBuf[512]       _at_ 0x0250 ; 
UINT8X	TTMPDataBuf[512]       _at_ 0x0460 ; 
#define	pSetupReq	((PXUSB_SETUP_REQ)TxBuffer)
bit		FoundNewDev;

#pragma NOAREGS

void	mDelayuS( UINT16 n );  // 以uS为单位延时
void	mDelaymS( UINT16 n );  // 以mS为单位延时

void	DisableRootHubPort( void );  // 关闭端口,实际上硬件已经自动关闭,此处只是清除一些结构状态
UINT8	AnalyzeRootHub( void );   // 分析端口状态,处理ROOT-HUB端口的设备插拔事件
// 返回ERR_SUCCESS为没有情况,返回ERR_USB_CONNECT为检测到新连接,返回ERR_USB_DISCON为检测到断开
void	SetHostUsbAddr( UINT8 addr );  // 设置USB主机当前操作的USB设备地址
void	ResetRootHubPort( void );  // 检测到设备后,复位总线,为枚举设备准备,设置为默认为全速
UINT8	EnableRootHubPort( void );  // 使能端口,相应的bUH_PORT_EN置1开启端口,设备断开可能导致返回失败
UINT8	WaitUSB_Interrupt( void );  // 等待USB中断

// CH559传输事务,输入目的端点地址/PID令牌,同步标志,以20uS为单位的NAK重试总时间(0则不重试,0xFFFF无限重试),返回0成功,超时/出错重试
UINT8	USBHostTransact( UINT8 endp_pid, UINT8 tog, UINT16 timeout );  // endp_pid: 高4位是token_pid令牌, 低4位是端点地址
UINT8	HostCtrlTransfer( PUINT8X DataBuf, PUINT8I RetLen );  // 执行控制传输,8字节请求码在pSetupReq中,DataBuf为可选的收发缓冲区
// 如果需要接收和发送数据,那么DataBuf需指向有效缓冲区用于存放后续数据,实际成功收发的总长度返回保存在ReqLen指向的字节变量中
void	CopySetupReqPkg( PUINT8C pReqPkt );  // 复制控制传输的请求包

UINT8	CtrlGetDeviceDescr( void );  // 获取设备描述符,返回在TxBuffer中
UINT8	CtrlGetConfigDescr( void );  // 获取配置描述符,返回在TxBuffer中
UINT8	CtrlSetUsbAddress( UINT8 addr );  // 设置USB设备地址
UINT8	CtrlSetUsbConfig( UINT8 cfg );  // 设置USB设备配置
UINT8	CtrlClearEndpStall( UINT8 endp );  // 清除端点STALL

UINT8	InitRootDevice( void );  // 初始化USB设备

/* U盘操作命令 */
UINT8	CtrlSetUsbReset( void );  //设置U盘复位,类命令
UINT8	CtrlGetMaxLUN( void );  // 获取U盘逻辑单元数量,类命令
UINT8	CH559BulkOnlyCmd( PUINT8X DataBuf );
UINT8	CH559ReadDiskInfo(PUINT8X DataBuf );    /* 索取器件信息 */
UINT8	CH559ReadDiskCap(PUINT8X DataBuf );  	/* 索取设备容量 */
UINT8	CH559ReadOneSector(PUINT8X DataBuf, UINT32 Addr );  /* 读取1个扇区数据 */
UINT8	CH559ReadDiskDBR(PUINT8X DataBuf );     			 /* 索取设备DBR */
UINT8	CH559FileOpen( PUINT8X DataBuf );					 /* 打开文件或者枚举文件 */
UINT8	CH559ReadFile( PUINT8X DataBuf );					 /* 读取文件完整内容 */
UINT8 ComString(PUINT8 source, PUINT8 target, UINT8 l);

void IAP_ERASE(void);//擦除
void IAP_PROM( PUINT8X DataBuf, UINT16 l );//下载
void IAP_END(void);//复位

/* 为printf和getkey输入输出初始化串口 */
void	mInitSTDIO( void );
void	InitUSB_Host( void );  // 初始化USB主机


void	DisableRootHubPort( void )  // 关闭端口,实际上硬件已经自动关闭,此处只是清除一些结构状态
{
	CH559DiskStatus = DISK_DISCONNECT;
}

UINT8	AnalyzeRootHub( void )   // 分析端口状态,处理端口的设备插拔事件
// 返回ERR_SUCCESS为没有情况,返回ERR_USB_CONNECT为检测到新连接,返回ERR_USB_DISCON为检测到断开
{ //处理端口的插拔事件,如果设备拔出,函数中调用DisableRootHubPort()函数,将端口关闭,插入事件,置相应端口的状态位
	UINT8	s;
	s = ERR_SUCCESS;
	if ( USB_HUB_ST & bUHS_H0_ATTACH ) {  // 设备存在
		if ( CH559DiskStatus == DISK_DISCONNECT || ( UHUB0_CTRL & bUH_PORT_EN ) == 0x00 ) {  // 检测到有设备插入,但尚未允许,说明是刚插入
			DisableRootHubPort( );  // 关闭端口
			CH559DiskStatus = DISK_CONNECT;
#ifdef DEBUG_NOW			
			printf( "USB dev in\n" );
#endif
			s = ERR_USB_CONNECT;
		}
	}
	else if ( CH559DiskStatus >= DISK_CONNECT ) {
		DisableRootHubPort( );  // 关闭端口
#ifdef DEBUG_NOW
		printf( "USB dev out\n" );
#endif
		if ( s == ERR_SUCCESS ) s = ERR_USB_DISCON;
	}
	return( s );
}

void	SetHostUsbAddr( UINT8 addr )  // 设置USB主机当前操作的USB设备地址
{
	USB_DEV_AD = (addr&0x7F);
}

void	ResetRootHubPort( void )  // 检测到设备后,复位总线,为枚举设备准备,设置为默认为全速
{
	UsbDevEndp0Size = DEFAULT_ENDP0_SIZE;  /* USB设备的端点0的最大包尺寸 */
	SetHostUsbAddr( 0x00 );
	SetUsbSpeed( 1 );  // 默认为全速
	UHUB0_CTRL = UHUB0_CTRL & ~ bUH_LOW_SPEED | bUH_BUS_RESET;  // 默认为全速,开始复位
	mDelaymS( 15 );  // 复位时间10mS到20mS
	UHUB0_CTRL = UHUB0_CTRL & ~ bUH_BUS_RESET;  // 结束复位
	mDelayuS( 250 );
	UIF_DETECT = 0;  // 清中断标志
}

UINT8	EnableRootHubPort( void )  // 使能端口,相应的bUH_PORT_EN置1开启端口,设备断开可能导致返回失败
{
	if ( CH559DiskStatus < DISK_CONNECT ) CH559DiskStatus = DISK_CONNECT;
	if ( USB_HUB_ST & bUHS_H0_ATTACH ) {  // 有设备
		UHUB0_CTRL |= bUH_PORT_EN;  //使能HUB端口
		return( ERR_SUCCESS );
	}
	return( ERR_USB_DISCON );
}

#if 0
UINT8	WaitUSB_Interrupt( void )  // 等待USB中断
{
	UINT16	i;
	for ( i = WAIT_USB_TOUT_200US; i != 0 && UIF_TRANSFER == 0; i -- );
	return( UIF_TRANSFER ? ERR_SUCCESS : ERR_USB_UNKNOWN );
}
#endif

// CH559传输事务,输入目的端点地址/PID令牌,同步标志,以20uS为单位的NAK重试总时间(0则不重试,0xFFFF无限重试),返回0成功,超时/出错重试
UINT8	USBHostTransact( UINT8 endp_pid, UINT8 tog, UINT16 timeout )  // endp_pid: 高4位是token_pid令牌, 低4位是端点地址
{  // 本子程序着重于易理解,而在实际应用中,为了提供运行速度,应该对本子程序代码进行优化
//	UINT8	TransRetry;
#define	TransRetry	UEP0_T_LEN	// 节约内存
	UINT8	s, r;
	UINT16	i;
	UH_RX_CTRL = UH_TX_CTRL = tog;
	TransRetry = 0;
	do {
		UH_EP_PID = endp_pid;  // 指定令牌PID和目的端点号
		UIF_TRANSFER = 0;  // 允许传输
//		s = WaitUSB_Interrupt( );
		for ( i = WAIT_USB_TOUT_200US; i != 0 && UIF_TRANSFER == 0; i -- );
		UH_EP_PID = 0x00;  // 停止USB传输
		if ( UIF_TRANSFER == 0 ) return( ERR_USB_UNKNOWN );
		if ( UIF_DETECT ) {  // USB设备插拔事件
//			mDelayuS( 200 );  // 等待传输完成
			UIF_DETECT = 0;  // 清中断标志
			s = AnalyzeRootHub( );   // 分析ROOT-HUB状态
			if ( s == ERR_USB_CONNECT ) FoundNewDev = 1;
			if ( CH559DiskStatus == DISK_DISCONNECT ) return( ERR_USB_DISCON );  // USB设备断开事件
			if ( CH559DiskStatus == DISK_CONNECT ) return( ERR_USB_CONNECT );  // USB设备连接事件
//			if ( ( USB_HUB_ST & bUHS_H0_ATTACH ) == 0x00 ) return( ERR_USB_DISCON );  // USB设备断开事件
			mDelayuS( 200 );  // 等待传输完成
		}
		if ( UIF_TRANSFER ) {  // 传输完成
			if ( U_TOG_OK ) return( ERR_SUCCESS );
			r = USB_INT_ST & MASK_UIS_H_RES;  // USB设备应答状态
			if ( r == USB_PID_STALL ) return( r | ERR_USB_TRANSFER );
			if ( r == USB_PID_NAK ) {
				if ( timeout == 0 ) return( r | ERR_USB_TRANSFER );
				if ( timeout < 0xFFFF ) timeout --;
				-- TransRetry;
			}
			else switch ( endp_pid >> 4 ) {
				case USB_PID_SETUP:
				case USB_PID_OUT:
//					if ( U_TOG_OK ) return( ERR_SUCCESS );
//					if ( r == USB_PID_ACK ) return( ERR_SUCCESS );
//					if ( r == USB_PID_STALL || r == USB_PID_NAK ) return( r | ERR_USB_TRANSFER );
					if ( r ) return( r | ERR_USB_TRANSFER );  // 不是超时/出错,意外应答
					break;  // 超时重试
				case USB_PID_IN:
//					if ( U_TOG_OK ) return( ERR_SUCCESS );
//					if ( tog ? r == USB_PID_DATA1 : r == USB_PID_DATA0 ) return( ERR_SUCCESS );
//					if ( r == USB_PID_STALL || r == USB_PID_NAK ) return( r | ERR_USB_TRANSFER );
					if ( r == USB_PID_DATA0 && r == USB_PID_DATA1 ) {  // 不同步则需丢弃后重试
					}  // 不同步重试
					else if ( r ) return( r | ERR_USB_TRANSFER );  // 不是超时/出错,意外应答
					break;  // 超时重试
				default:
					return( ERR_USB_UNKNOWN );  // 不可能的情况
					break;
			}
		}
		else {  // 其它中断,不应该发生的情况
			USB_INT_FG = 0xFF;  /* 清中断标志 */
		}
		mDelayuS( 15 );
	} while ( ++ TransRetry < 3 );
	return( ERR_USB_TRANSFER );  // 应答超时
}

UINT8	HostCtrlTransfer( PUINT8X DataBuf, PUINT8 RetLen )  // 执行控制传输,8字节请求码在pSetupReq中,DataBuf为可选的收发缓冲区
// 如果需要接收和发送数据,那么DataBuf需指向有效缓冲区用于存放后续数据,实际成功收发的总长度保存在ReqLen指向的字节变量中
{
	UINT8	 s, RemLen, RxLen, RxCnt, TxCnt;

	mDelayuS( 200 );
	if ( RetLen ) *RetLen = 0;  // 实际成功收发的总长度
	UH_TX_LEN = sizeof( USB_SETUP_REQ );
	s = USBHostTransact( USB_PID_SETUP << 4 | 0x00, 0x00, 200000/20 );  // SETUP阶段,200mS超时
	if ( s != ERR_SUCCESS ) return( s );
	UH_RX_CTRL = UH_TX_CTRL = bUH_R_TOG | bUH_R_AUTO_TOG | bUH_T_TOG | bUH_T_AUTO_TOG;  // 默认DATA1
	UH_TX_LEN = 0x01;  // 默认无数据故状态阶段为IN
	RemLen = pSetupReq -> wLengthH ? 0xFF : pSetupReq -> wLengthL;
	if ( RemLen && DataBuf ) {  // 需要收发数据
		if ( pSetupReq -> bRequestType & USB_REQ_TYP_IN ) {  // 收
			while ( RemLen ) {
				mDelayuS( 200 );
				s = USBHostTransact( USB_PID_IN << 4 | 0x00, UH_RX_CTRL, 200000/20 );  // IN数据
				if ( s != ERR_SUCCESS ) return( s );
				RxLen = USB_RX_LEN < RemLen ? USB_RX_LEN : RemLen;
				RemLen -= RxLen;
				if ( RetLen ) *RetLen += RxLen;  // 实际成功收发的总长度
				for ( RxCnt = 0; RxCnt != RxLen; RxCnt ++ ) {
					*DataBuf = RxBuffer[ RxCnt ];
					DataBuf ++;
				}
				if ( USB_RX_LEN == 0 || ( USB_RX_LEN & ( UsbDevEndp0Size - 1 ) ) ) break;  // 短包
			}
			UH_TX_LEN = 0x00;  // 状态阶段为OUT
		}
		else {  // 发
			while ( RemLen ) {
				mDelayuS( 200 );
				UH_TX_LEN = RemLen >= UsbDevEndp0Size ? UsbDevEndp0Size : RemLen;
				for ( TxCnt = 0; TxCnt != UH_TX_LEN; TxCnt ++ ) {
					TxBuffer[ TxCnt ] = *DataBuf;
					DataBuf ++;
				}
				s = USBHostTransact( USB_PID_OUT << 4 | 0x00, UH_TX_CTRL, 200000/20 );  // OUT数据
				if ( s != ERR_SUCCESS ) return( s );
				RemLen -= UH_TX_LEN;
				if ( RetLen ) *RetLen += UH_TX_LEN;  // 实际成功收发的总长度
			}
			UH_TX_LEN = 0x01;  // 状态阶段为IN
		}
	}
	mDelayuS( 200 );
	s = USBHostTransact( ( UH_TX_LEN ? USB_PID_IN << 4 | 0x00: USB_PID_OUT << 4 | 0x00 ), bUH_R_TOG | bUH_T_TOG, 200000/20 );  // 状态阶段,DATA1
	if ( s != ERR_SUCCESS ) return( s );
	if ( UH_TX_LEN == 0 ) return( ERR_SUCCESS );  // 状态OUT
	if ( USB_RX_LEN == 0 ) return( ERR_SUCCESS );  // 状态IN,检查IN状态返回数据长度
	return( ERR_USB_BUF_OVER );  // IN状态阶段错误
}

void	CopySetupReqPkg( PUINT8C pReqPkt )  // 复制控制传输的请求包
{
	UINT8	i;
	for ( i = 0; i != sizeof( USB_SETUP_REQ ); i ++ ) {
		((PUINT8X)pSetupReq)[ i ] = *pReqPkt;
		pReqPkt ++;
	}
}

UINT8	CtrlGetDeviceDescr( void )  // 获取设备描述符,返回在TxBuffer中
{
	UINT8	s;
	UINT8D	len;
	
	UsbDevEndp0Size = DEFAULT_ENDP0_SIZE;
	CopySetupReqPkg( SetupGetDevDescr );	     //写入发送缓存区
	s = HostCtrlTransfer( TxBuffer, &len );      // 执行控制传输
	if ( s != ERR_SUCCESS ) return( s );
	UsbDevEndp0Size = ( (PXUSB_DEV_DESCR)TxBuffer ) -> bMaxPacketSize0;  // 端点0最大包长度,这是简化处理,正常应该先获取前8字节后立即更新UsbDevEndp0Size再继续
	if ( len < ( (PUSB_SETUP_REQ)SetupGetDevDescr ) -> wLengthL ) return( ERR_USB_BUF_OVER );  // 描述符长度错误
	return( ERR_SUCCESS );
}

UINT8	CtrlGetConfigDescr( void )  // 获取配置描述符,返回在TxBuffer中
{
	UINT8	s;
	UINT8D	len;
	CopySetupReqPkg( SetupGetCfgDescr );
	s = HostCtrlTransfer( TxBuffer, &len );  // 执行控制传输
	if ( s != ERR_SUCCESS ) return( s );
	if ( len < ( (PUSB_SETUP_REQ)SetupGetCfgDescr ) -> wLengthL ) return( ERR_USB_BUF_OVER );  // 返回长度错误
	len = ( (PXUSB_CFG_DESCR)TxBuffer ) -> wTotalLengthL;
	if ( len > MAX_PACKET_SIZE ) return( ERR_USB_BUF_OVER );  // 返回长度错误
	
	CopySetupReqPkg( SetupGetCfgDescr );
	pSetupReq -> wLengthL = len;  // 完整配置描述符的总长度
	s = HostCtrlTransfer( TxBuffer, &len );  // 执行控制传输
	if ( s != ERR_SUCCESS ) return( s );
	if ( len < ( (PUSB_SETUP_REQ)SetupGetCfgDescr ) -> wLengthL || len < ( (PXUSB_CFG_DESCR)TxBuffer ) -> wTotalLengthL ) return( ERR_USB_BUF_OVER );  // 描述符长度错误
	
	CtrlSetUsbConfig(( (PXUSB_CFG_DESCR)TxBuffer ) -> bConfigurationValue);
	if((((PXUSB_CFG_DESCR_LONG)TxBuffer) -> endp_descr[0].bEndpointAddress)>0x80)
	{
		DevEndpOUTAddr = (((PXUSB_CFG_DESCR_LONG)TxBuffer) -> endp_descr[1].bEndpointAddress)&0x0f;
		DevEndpINAddr  = (((PXUSB_CFG_DESCR_LONG)TxBuffer) -> endp_descr[0].bEndpointAddress)&0x0f;
	}
	else
	{
		DevEndpOUTAddr = (((PXUSB_CFG_DESCR_LONG)TxBuffer) -> endp_descr[0].bEndpointAddress)&0x0f;
		DevEndpINAddr  = (((PXUSB_CFG_DESCR_LONG)TxBuffer) -> endp_descr[1].bEndpointAddress)&0x0f;
	}

	return( ERR_SUCCESS );
}

UINT8	CtrlSetUsbAddress( UINT8 addr )  // 设置USB设备地址
{
	UINT8	s;
	CopySetupReqPkg( SetupSetUsbAddr );
	pSetupReq -> wValueL = addr;  // USB设备地址
	s = HostCtrlTransfer( NULL, NULL );  // 执行控制传输
	if ( s != ERR_SUCCESS ) return( s );
	SetHostUsbAddr( addr );  // 设置USB主机当前操作的USB设备地址
	mDelaymS( 10 );  // 等待USB设备完成操作
	return( ERR_SUCCESS );
}

UINT8	CtrlSetUsbConfig( UINT8 cfg )  // 设置USB设备配置
{
#ifdef DEBUG_NOW
	printf( "SetUsbConfig: \n" );
#endif
	CopySetupReqPkg( SetupSetUsbConfig );
	pSetupReq -> wValueL = cfg;  // USB设备配置
	return( HostCtrlTransfer( NULL, NULL ) );  // 执行控制传输
}

#if 0
UINT8	CtrlClearEndpStall( UINT8 endp )  // 清除端点STALL
{
	CopySetupReqPkg( SetupClrEndpStall );  // 清除端点的错误
	pSetupReq -> wIndexL = endp;  // 端点地址
	return( HostCtrlTransfer( NULL, NULL ) );  /* 执行控制传输 */
}
#endif

/* ************************************************************************************** */
UINT8	CtrlSetUsbReset( void )
{
	UINT8	s;
	UINT8D	len;
	
	CopySetupReqPkg( SetReset );	     		 //写入发送缓存区
	s = HostCtrlTransfer( TxBuffer, &len );      // 执行控制传输
	if ( s != ERR_SUCCESS ) return( s );
	if ( len < ( (PUSB_SETUP_REQ)SetReset ) -> wLengthL ) return( ERR_USB_BUF_OVER );  // 描述符长度错误
	return( ERR_SUCCESS );
}

UINT8	CtrlGetMaxLUN( void )
{
	UINT8	s;
	UINT8D	len;
	
	CopySetupReqPkg( GetMaxLUN );	     		 //写入发送缓存区
	s = HostCtrlTransfer( TxBuffer, &len );      // 执行控制传输
	if ( s != ERR_SUCCESS ) return( s );
	if ( len < ( (PUSB_SETUP_REQ)GetMaxLUN ) -> wLengthL ) return( ERR_USB_BUF_OVER );  // 描述符长度错误
	CH559vCurrentLun = TxBuffer[0];
	UH_RX_CTRL = UH_TX_CTRL = bUH_R_TOG | bUH_T_TOG;
	return( ERR_SUCCESS );
}

/* 执行基于BulkOnly协议的命令 */
UINT8	CH559BulkOnlyCmd( PUINT8X DataBuf )
{	
	UINT8 s, i, RxLen=0;
	UINT32	RemLen;
	
	pCBW -> mCBW_Sig0 = 0x55;  
	pCBW -> mCBW_Sig1 = 0x53;
	pCBW -> mCBW_Sig2 = 0x42;
	pCBW -> mCBW_Sig3 = 0x43;
	pCBW -> mCBW_Tag0 = 0x00;  
	pCBW -> mCBW_Tag1 = 0x00;
	pCBW -> mCBW_Tag2 = 0x01;
	pCBW -> mCBW_Tag3 = 0x00;
	pCBW -> mCBW_CB_Buf[ 10 ] = 0x07;  
	pCBW -> mCBW_CB_Buf[ 11 ] = 0x05;
	pCBW -> mCBW_CB_Buf[ 12 ] = 0x81;
	pCBW -> mCBW_CB_Buf[ 13 ] = 0x02;
	pCBW -> mCBW_CB_Buf[ 14 ] = 0x40;  
	pCBW -> mCBW_CB_Buf[ 15 ] = 0x00;	
	
	UH_TX_LEN = 31;
	RemLen = (((pCBW -> mCBW_DataLen1)<<8) |pCBW -> mCBW_DataLen0);	//获取CBW与CSW直接的传输字节数
	UH_TX_CTRL ^= bUH_T_TOG;
	s = USBHostTransact( USB_PID_OUT << 4 | DevEndpOUTAddr, UH_TX_CTRL, 200000/20 );  // OUT数据  CBW
	if ( s != ERR_SUCCESS ) return( s );
	if(RemLen)		//有传输字节
	{		
		if(pCBW -> mCBW_Flag)		//获取数据，IN传输
		{
			while(RemLen)
			{
				mDelayuS( 200 );
				UH_RX_CTRL ^= bUH_R_TOG;
				s = USBHostTransact( USB_PID_IN << 4 | DevEndpINAddr, UH_RX_CTRL, 65535 );  	// IN数据
				if ( s != ERR_SUCCESS ) return( s );
				RxLen = USB_RX_LEN < RemLen ? USB_RX_LEN : RemLen; 
				RemLen -= RxLen;
				for ( i = 0; i != RxLen; i ++ ) {
					*DataBuf = RxBuffer[ i ];
					DataBuf ++;
				}
			}
		}
		else						//发送数据，OUT传输
		{
			mDelayuS( 200 );
			UH_TX_LEN = RemLen >= 64 ? 64 : RemLen;
			for ( i = 0; i != UH_TX_LEN; i ++ ) {
				TxBuffer[ i ] = *DataBuf;
				DataBuf ++;
			}
			s = USBHostTransact( USB_PID_OUT << 4 | DevEndpOUTAddr, UH_TX_CTRL, 200000/20 );   // OUT数据
			if ( s != ERR_SUCCESS ) return( s );
			RemLen -= UH_TX_LEN;
		}
	}
	
	mDelayuS( 200 );
	UH_RX_CTRL ^= bUH_R_TOG;
	s = USBHostTransact( USB_PID_IN << 4 | DevEndpINAddr, UH_RX_CTRL, 200000/20 );  			// IN数据 CSW
	if ( s != ERR_SUCCESS ) return( s );
	return (pCSW -> mCSW_Status);
}

#if 1
UINT8	CH559ReadDiskInfo(PUINT8X DataBuf )  /* 索取器件信息 */
{
	UINT8	retry, s;

	for( retry = 0; retry < 3; retry ++ ) {  /* 错误重试 */
		pCBW -> mCBW_DataLen0 = 0x24;/* 数据传输长度 */  			
		pCBW -> mCBW_DataLen1 = 0x00;
		pCBW -> mCBW_DataLen2 = 0x00;
		pCBW -> mCBW_DataLen3 = 0x00;
		pCBW -> mCBW_Flag = 0x80;			/* 上传 */
		pCBW -> mCBW_LUN = CH559vCurrentLun;
		pCBW -> mCBW_CB_Len = 0x06;
		pCBW -> mCBW_CB_Buf[ 0 ] = SPC_CMD_INQUIRY;/* 操作命令代码 */		
		pCBW -> mCBW_CB_Buf[ 1 ] = CH559vCurrentLun<<4;
		pCBW -> mCBW_CB_Buf[ 2 ] = 0x00;/* 逻辑块地址 */		
		pCBW -> mCBW_CB_Buf[ 3 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 4 ] = 0x24;
		pCBW -> mCBW_CB_Buf[ 5 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 6 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 7 ] = 0x00;/* 传输长度 */
		pCBW -> mCBW_CB_Buf[ 8 ] = 0x01;		
		pCBW -> mCBW_CB_Buf[ 9 ] = 0x00;
		s = CH559BulkOnlyCmd( DataBuf );  /* 执行基于BulkOnly协议的命令 */
		if(s == ERR_SUCCESS)	break;
	}
	return( s );  /* 磁盘操作错误 */
}
#endif

#if 0
UINT8	CH559ReadDiskCap(PUINT8X DataBuf )  /* 索取设备容量 */
{
	UINT8	retry, s;

	for( retry = 0; retry < 3; retry ++ ) {  /* 错误重试 */
		pCBW -> mCBW_DataLen0 = 0x08;/* 数据传输长度 */  			
		pCBW -> mCBW_DataLen1 = 0x00;
		pCBW -> mCBW_DataLen2 = 0x00;
		pCBW -> mCBW_DataLen3 = 0x00;
		pCBW -> mCBW_Flag = 0x80;			/* 上传 */
		pCBW -> mCBW_LUN = CH559vCurrentLun;
		pCBW -> mCBW_CB_Len = 0x0a;
		pCBW -> mCBW_CB_Buf[ 0 ] = SPC_CMD_READ_CAPACITY;/* 操作命令代码 */		
		pCBW -> mCBW_CB_Buf[ 1 ] = CH559vCurrentLun<<4;
		pCBW -> mCBW_CB_Buf[ 2 ] = 0x00;/* 逻辑块地址 */		
		pCBW -> mCBW_CB_Buf[ 3 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 4 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 5 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 6 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 7 ] = 0x00;/* 传输长度 */
		pCBW -> mCBW_CB_Buf[ 8 ] = 0x00;		
		pCBW -> mCBW_CB_Buf[ 9 ] = 0x00;
		s = CH559BulkOnlyCmd( DataBuf );  /* 执行基于BulkOnly协议的命令 */
		if(s == ERR_SUCCESS)	break;
	}
	return( s );  /* 磁盘操作错误 */
}
#endif

UINT8	CH559ReadOneSector(PUINT8X DataBuf, UINT32 Addr )  /* 读取1个扇区数据 */
{
	UINT8	retry, s;

	for( retry = 0; retry < 3; retry ++ ) {  /* 错误重试 */
		pCBW -> mCBW_DataLen0 = 0x00;/* 数据传输长度 */  			
		pCBW -> mCBW_DataLen1 = 0x02;
		pCBW -> mCBW_DataLen2 = 0x00;
		pCBW -> mCBW_DataLen3 = 0x00;
		pCBW -> mCBW_Flag = 0x80;			/* 上传 */
		pCBW -> mCBW_LUN = CH559vCurrentLun;
		pCBW -> mCBW_CB_Len = 0x0a;
		pCBW -> mCBW_CB_Buf[ 0 ] = SPC_CMD_READ10;/* 操作命令代码 */		
		pCBW -> mCBW_CB_Buf[ 1 ] = CH559vCurrentLun<<4;
		pCBW -> mCBW_CB_Buf[ 2 ] = (UINT8)(Addr>>24);/* 逻辑块地址扇区号 */		
		pCBW -> mCBW_CB_Buf[ 3 ] = (UINT8)(Addr>>16);
		pCBW -> mCBW_CB_Buf[ 4 ] = (UINT8)(Addr>>8);
		pCBW -> mCBW_CB_Buf[ 5 ] = (UINT8)Addr;
		pCBW -> mCBW_CB_Buf[ 6 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 7 ] = 0x00;/* 传输长度:1个扇区大小 */
		pCBW -> mCBW_CB_Buf[ 8 ] = 0x01;		
		pCBW -> mCBW_CB_Buf[ 9 ] = 0x00;
		s = CH559BulkOnlyCmd( DataBuf );  /* 执行基于BulkOnly协议的命令 */
		if(s == ERR_SUCCESS)	break;
	}
	return( s );  /* 磁盘操作错误 */
}

UINT8	CH559ReadDiskDBR(PUINT8X DataBuf )  /* 索取设备DBR */
{
	UINT8   s,j;
	UINT16  i,ss =0;
  j = 4;
	CH559vSecMBR = 0;
	s = CH559ReadOneSector(DataBuf, 0x00000000);
//   memcpy(TTMPDataBuf,DataBuf,512);
  while(ss < 512){
    TTMPDataBuf[ss] = DataBuf[ss];
    ss = ss +1;
  }
#ifdef DEBUG_NOW
      ss = 0;
      while(ss<512){
        if((ss%16) == 0)printf("\n");				
        printf("  %02x",(UINT16)TTMPDataBuf[ss]);
		   		ss = ss+1;
      }
#endif	
	i = 0x01BE;
	if(s == ERR_SUCCESS)	
	{
      while(j){
			CH559vSecMBR = ((UINT32)TTMPDataBuf[i+11]<<24)|((UINT32)TTMPDataBuf[i+10]<<16)|((UINT16)TTMPDataBuf[i+9]<<8)|TTMPDataBuf[i+8];  // DBR扇区号	
      if(CH559vSecMBR < 0x20000){//小于64G				
      s = CH559ReadOneSector(TMPDataBuf, CH559vSecMBR);
#ifdef DEBUG_NOW
      ss = 0;
      while(ss<512){
        if((ss%16) == 0)printf("\n");				
        printf("  %02x",(UINT16)TMPDataBuf[ss]);
					ss = ss+1;
      }
      printf("sssssssssssssssss\n");				
#endif
			if((TMPDataBuf[0x00]==0xEB)&&(TMPDataBuf[0x02]==0x90))	//DBR
		  {	
			  break;	
			}	
		  }	
      j--;			
			i+=16;					
    }	
    if(j == 0){ 
//       memcpy(DataBuf,TTMPDataBuf,512);	
      ss = 0;
			while(ss < 512){
				TTMPDataBuf[ss] = DataBuf[ss];
        ss = ss +1;
			}
      CH559vSecMBR = 0;
#ifdef DEBUG_NOW
      printf("num 0 %02x\n",(UINT16)j);				
#endif			
    }
	  else{ 
//       memcpy(DataBuf,TMPDataBuf,512);
      ss = 0;
			while(ss < 512){
				TTMPDataBuf[ss] = DataBuf[ss];
        ss = ss +1;				
			}			
#ifdef DEBUG_NOW
      printf("num n %02x\n",(UINT16)j);						
#endif	
    }
#ifdef DEBUG_NOW
      ss = 0;
      while(ss<512){
        if((ss%16) == 0)printf("\n");				
        printf("  %02x",(UINT16)DataBuf[ss]);
		   		ss = ss+1;
      }
#endif		
		CH559vSectorSizeH = DataBuf[0x0c];//扇区大小
		CH559vSecPerClus = DataBuf[0x0d]; //逻辑盘的每簇扇区数
		CH559vDiskRoot = ((UINT32)DataBuf[0x2f]<<24)|((UINT32)DataBuf[0x2e]<<16)|((UINT16)DataBuf[0x2d]<<8)|DataBuf[0x2c];//对于FAT32盘为根目录起始簇号
		CH559vSecFdt = ((UINT32)DataBuf[0x27]<<24)|((UINT32)DataBuf[0x26]<<16)|((UINT16)DataBuf[0x25]<<8)|DataBuf[0x24];  // FAT表占用扇区数
		CH559vNumFdt = DataBuf[0x10];     //FAT个数
		CH559vSecResv = ((UINT16)DataBuf[0x0F]<<8)|DataBuf[0x0E];//保留扇区数	
		CH559vDataStart	= CH559vSecMBR+CH559vSecResv+CH559vNumFdt*CH559vSecFdt;//目录项首扇区
#ifdef DEBUG_NOW
		switch(DataBuf[0x01])
		{
			case 0x3C:
				printf("It is fat16!\n");return(0xFA);//系统不对
			break;
			case 0x58:
				printf("It is fat32!\n");
			break;
			case 0x52:
				printf("It is NTFS!\n"); return(0xFA); //系统不对
			break;
			default:
				printf("system unknown! %02x  \n",(UINT16)DataBuf[0x01]);return(0xFA);//系统不对
			break;
		}
		printf("CH559vSectorSizeH: %02d \n",(UINT16)CH559vSectorSizeH);
		printf("CH559vSecPerClus: %02d \n",(UINT16)CH559vSecPerClus);
		printf("CH559vDiskRoot: %ld \n",CH559vDiskRoot);
		printf("CH559vSecFdt: %ld \n",CH559vSecFdt);
		printf("CH559vNumFdt: %02d \n",(UINT16)CH559vNumFdt);
		printf("CH559vSecResv: %02d \n",(UINT16)CH559vSecResv);
		printf("CH559vDataStart: %ld \n",CH559vDataStart);
#endif
	}
	return( s );  /* 磁盘操作错误 */
}

UINT32X	CH559ReadDiskFAT(PUINT8X DataBuf, UINT32X ClusNum)  /* 索取设备FAT表指定位置值 */
{
	UINT8   s; 
	UINT16  j=0;
	UINT32X  NextClusNum;

	s = CH559ReadOneSector(DataBuf, CH559vSecMBR+CH559vSecResv+(ClusNum<<2)/(CH559vSectorSizeH<<8));
	if(s == ERR_SUCCESS)
	{
		j = (ClusNum*4)%(CH559vSectorSizeH<<8);
		NextClusNum = ((UINT32)DataBuf[j+3]<<24)|((UINT32)DataBuf[j+2]<<16)|((UINT16)DataBuf[j+1]<<8)|(UINT32)DataBuf[j];
	}
	if( NextClusNum == 0x0fffffff)	return( 0 );  /* 文件已经结束于当前簇 */
	else 				  return( NextClusNum );  /* 返回下一簇号 */	

}

UINT8 ComString(PUINT8 source, PUINT8 target, UINT8 l)
{
	UINT8 i;
	for(i=0;i<l;i++)
	{
		if(*source++ != *target++)	return 1;
	}
	return 0;
}

UINT8	CH559FileOpen( PUINT8X DataBuf )
{
	UINT8	 s, j;
	UINT8	 mPathName[12] = "DOWNLOADBIN";
	UINT16   i=0;
	UINT32X	 NextClusNum, secnum;
	
	NextClusNum = CH559vDiskRoot;
	do{
		secnum = CH559vDataStart+CH559vSecPerClus*(NextClusNum-CH559vDiskRoot);//计算准备读取簇的扇区号
		
		for(j=0; j<CH559vSecPerClus; j++)				//读取一个簇的数据，查找文件
		{				
			s = CH559ReadOneSector(DataBuf,secnum+j);//读取目录项中1个扇区大小
			if(s == ERR_SUCCESS)	
			{
				i = 0;
				while(ComString(mPathName, &DataBuf[i],11)&&(i<512))
				{
					i += 32;
				}
				if(i<512)   
				{
					s = ERR_SUCCESS;
					CH559vFdtLba = (CH559vDataStart+j)*((UINT16)CH559vSectorSizeH<<8)+i;  		//文件目录项当前的逻辑地址		
					CH559vFileSize = ((UINT32)DataBuf[i+0x1f]<<24)|((UINT32)DataBuf[i+0x1e]<<16)|((UINT16)DataBuf[i+0x1d]<<8)|DataBuf[i+0x1c];	  //文件大小	
					CH559vStartCluster = ((UINT32)DataBuf[i+0x15]<<24)|((UINT32)DataBuf[i+0x14]<<16)|((UINT16)DataBuf[i+0x1b]<<8)|DataBuf[i+0x1a];//文件起始簇号				
					CH559vCurrentOffset = (CH559vDataStart+CH559vSecPerClus*(CH559vStartCluster-CH559vDiskRoot)); //文件内容当前指针扇区号
#ifdef DEBUG_NOW
					printf("CH559vFdtLba: %ld \n",CH559vFdtLba);
					printf("CH559vFileSize: %ld \n",CH559vFileSize);
					printf("CH559vStartCluster: %ld \n",CH559vStartCluster);
					printf("CH559vCurrentOffset: %ld \n",CH559vCurrentOffset);
#endif
					return( s ); 
				}
				else	s = ERR_MISS_FILE;
			}
		}
		
		NextClusNum = CH559ReadDiskFAT(DataBuf,NextClusNum);//计算下一簇号
		
#ifdef DEBUG_NOW
		printf("NextClusNum: x%lx \n",NextClusNum);
#endif
	}while(NextClusNum);
	
	return( s );  /* 磁盘操作错误 */
}

UINT8	CH559ReadFile( PUINT8X DataBuf )		/* 读取文件完整内容 */
{
	UINT8	 s, j;
	UINT16   len;
	UINT32X	 NextClusNum, secnum, resvsize;
	
	NextClusNum = CH559vStartCluster;	
	resvsize = CH559vFileSize;
	do{
		secnum = CH559vDataStart+CH559vSecPerClus*(NextClusNum-CH559vDiskRoot);//计算准备读取簇的扇区号
		for(j=0; j<CH559vSecPerClus; j++)				//读取一个簇的数据
		{				
			s = CH559ReadOneSector(DataBuf,secnum+j);   //读取目录项中1个扇区大小
			if(s == ERR_SUCCESS)
			{		
				len = (resvsize>((UINT16)CH559vSectorSizeH<<8))?((UINT16)CH559vSectorSizeH<<8):resvsize;
				IAP_PROM(DataBuf, len);
				resvsize -= len;
#ifdef DEBUG_NOW
				printf("resvsize: x%lx \n",resvsize);
#endif
				if(!resvsize)		return s;			//文件读取完成
			}
		}		
		NextClusNum = CH559ReadDiskFAT(DataBuf,NextClusNum);//计算下一簇号
		
#ifdef DEBUG_NOW
		printf("NextClusNum: x%lx \n",NextClusNum);
#endif
	}while(NextClusNum);
	
	return( s );  /* 磁盘操作错误 */
}


/* ************************************************************************************** */

UINT8	InitRootDevice( void )  // 初始化USB设备
{
	UINT8	i, s, dv_cls, if_cls;
	
#ifdef DEBUG_NOW
	printf( "Reset host port\n" );
#endif
	ResetRootHubPort( );  // 检测到设备后,复位相应端口的USB总线
	for ( i = 0, s = 0; i < 100; i ++ ) {  // 等待USB设备复位后重新连接,100mS超时
		mDelaymS( 1 );
		if ( EnableRootHubPort( ) == ERR_SUCCESS ) {  // 使能端口
			i = 0;
			s ++;  // 计时等待USB设备连接后稳定
			if ( s > 100 ) break;  // 已经稳定连接100mS
		}
	}
	if ( i ) {  // 复位后设备没有连接
		DisableRootHubPort( );
#ifdef DEBUG_NOW
		printf( "Disable host port because of disconnect\n" );
#endif
		return( ERR_USB_DISCON );
	}
	SetUsbSpeed( 1 );  // 设置当前USB速度全速
#ifdef DEBUG_NOW
	printf( "GetDevDescr: " );
#endif
	s = CtrlGetDeviceDescr( );  // 获取设备描述符
	if ( s == ERR_SUCCESS ) {
#ifdef DEBUG_NOW
		for ( i = 0; i < ( (PUSB_SETUP_REQ)SetupGetDevDescr ) -> wLengthL; i ++ ) printf( "x%02X ", (UINT16)( TxBuffer[i] ) );
		printf( "\n" ); // 显示出描述符
#endif
		
		dv_cls = ( (PXUSB_DEV_DESCR)TxBuffer ) -> bDeviceClass;  // 设备类代码
		s = CtrlSetUsbAddress( ( (PUSB_SETUP_REQ)SetupSetUsbAddr ) -> wValueL );  // 设置USB设备地址
		if ( s == ERR_SUCCESS ) {
#ifdef DEBUG_NOW
			printf( "GetCfgDescr: \n" );
#endif
			s = CtrlGetConfigDescr( );  // 获取配置描述符
			if ( s == ERR_SUCCESS ) {
/* 分析配置描述符,获取端点数据/各端点地址/各端点大小等,更新变量endp_addr和endp_size等 */
				if_cls = ( (PXUSB_CFG_DESCR_LONG)TxBuffer ) -> itf_descr.bInterfaceClass;  // 接口类代码
				if ( dv_cls == 0x00 && if_cls == USB_DEV_CLASS_STORAGE ) {  // 是USB存储类设备,基本上确认是U盘
					CH559DiskStatus = DISK_USB_ADDR;
					return( ERR_SUCCESS );
				}
				else {
					return( ERR_USB_UNSUPPORT );
				}
			}
		}
	}
#ifdef DEBUG_NOW
	printf( "InitRootDev Err = %02X\n", (UINT16)s );
#endif
	CH559DiskStatus = DISK_CONNECT;
	SetUsbSpeed( 1 );  // 默认为全速
	return( s );
}

void	InitUSB_Host( void )  // 初始化USB主机
{
	IE_USB = 0;					//禁止USB中断

	USB_CTRL = bUC_HOST_MODE;  	// 先设定模式
	USB_DEV_AD = 0x00;			// 连接设备地址为0x00
	
	UHUB1_CTRL &= ~bUH1_DISABLE; //打开HUB1
	UH_EP_MOD = bUH_EP_TX_EN | bUH_EP_RX_EN ;
	UH_RX_DMA = RxBuffer;
	UH_TX_DMA = TxBuffer;
	UH_RX_CTRL = 0x00;
	UH_TX_CTRL = 0x00;
	
	USB_CTRL = bUC_HOST_MODE | bUC_INT_BUSY | bUC_DMA_EN;  // 启动USB主机及DMA,在中断标志未清除前自动暂停
	UH_SETUP = bUH_SOF_EN;
	USB_INT_FG = 0xFF;  	// 清中断标志
	USB_INT_EN = bUIE_TRANSFER | bUIE_DETECT;
}

#if 0
/* 检查操作状态,如果错误则显示错误代码并停机 */
void	mStopIfError( UINT8 iError )
{
	if ( iError == ERR_SUCCESS ) return;  /* 操作成功 */
	printf( "Error: %02X\n", (UINT16)iError );  /* 显示错误 */
/* 遇到错误后,应该分析错误码以及CH559DiskStatus状态,例如调用CH559DiskReady查询当前U盘是否连接,如果U盘已断开那么就重新等待U盘插上再操作,
   建议出错后的处理步骤:
   1、调用一次CH559DiskReady,成功则继续操作,例如Open,Read/Write等
   2、如果CH559DiskReady不成功,那么强行将从头开始操作(等待U盘连接，CH559DiskReady等) */
	while ( 1 ) {
//		LED_TMP=0;  /* LED闪烁 */
//		mDelaymS( 100 );
//		LED_TMP=1;
//		mDelaymS( 100 );
	}
}
#endif

#define IAP_CODE_ADDR        (0x7700-1024)	  //1k的整

UINT8	EraseBlock( UINT16 Addr ) 
{
	ROM_ADDR = Addr;
	if ( ROM_STATUS & bROM_ADDR_OK ) {  /* 操作地址有效 */
		if ( (UINT8)EraseBlock & 0x01 ) {
			ROM_CTRL = ROM_CMD_ERASE;
			_nop_( );
		}
		else {
			_nop_( );
			ROM_CTRL = ROM_CMD_ERASE;																				
		}
		return( ( ROM_STATUS ^ bROM_ADDR_OK ) & 0x7F );  /* 返回状态,0x00=success, 0x01=time out(bROM_CMD_TOUT), 0x02=unknown command(bROM_CMD_ERR) */
	}
	else return( 0x40 );
}

UINT8 ProgWord( UINT16 Addr, UINT16 Data )
{
	ROM_ADDR = Addr;
	ROM_DATA = Data;
	if ( ROM_STATUS & bROM_ADDR_OK ) {  /* 操作地址有效 */
		if ( (UINT8)ProgWord & 0x01 ) {
			ROM_CTRL = ROM_CMD_PROG;
			_nop_( );
		}
		else {
			_nop_( );
			ROM_CTRL = ROM_CMD_PROG;
		}
		return( ( ROM_STATUS ^ bROM_ADDR_OK ) & 0x7F );  /* 返回状态,0x00=success, 0x01=time out(bROM_CMD_TOUT), 0x02=unknown command(bROM_CMD_ERR) */
	}
	else return( 0x40 );
}

void IAP_ERASE(void)
{
	UINT8  s;
	UINT16 i;
	UINT32 addr;
	
	SAFE_MOD = 0x55;
	SAFE_MOD = 0xAA;
	GLOBAL_CFG |= bCODE_WE | bDATA_WE;
	addr = 0x00000000;
	for( i=0; addr <= IAP_CODE_ADDR;i++ ){
		s = EraseBlock( addr );
		addr+=1024;
		if( s != 0 )break;
	}
	SAFE_MOD = 0x55;
	SAFE_MOD = 0xAA;
	GLOBAL_CFG &= ~ ( bCODE_WE | bDATA_WE );
}

void IAP_PROM( PUINT8X DataBuf, UINT16 l )
{
	UINT8  s;
	UINT16 i;
	UINT16 len,Data;
	UINT32 addr;
	
	len = (l+1)>>1;		//必须为2的整数倍，按照半字进行操作
	addr = 0x00000000;
	SAFE_MOD = 0x55;
	SAFE_MOD = 0xAA;
	GLOBAL_CFG |= bCODE_WE | bDATA_WE;
	for( i=0;i!=len;i++ ){							/* 一次写入一个扇区大小 */
		Data = (DataBuf[2*i]| (UINT16)DataBuf[2*i+1]<<8 ); 
		s = ProgWord( addr,Data );
		addr+=2;
		if( s != 0x00 )break;
	}
	SAFE_MOD = 0x55;
	SAFE_MOD = 0xAA;
	GLOBAL_CFG &= ~ ( bCODE_WE | bDATA_WE );
}

void IAP_END(void)
{
	SAFE_MOD = 0x55;					   
	SAFE_MOD = 0xAA;
	GLOBAL_CFG |= bSW_RESET;				/* 复位单片机,进入用户程序 */
}


void main( ) {
	UINT8	s;
	UINT16	i;
  EA = 0;
  ES = 0; 	
  PIN_FUNC &= ~bUART0_PIN_X;                                                 //串口映射到P0.2和P0.3
	mDelaymS(30);                                                              //上电延时,等待内部晶振稳定,必加 
//	CfgFsys();
#ifdef DEBUG_NOW	
	mInitSTDIO(); 
	printf( "Start @ChipID=%02X\n", (UINT16)CHIP_ID );
#endif
	
	InitUSB_Host( );
#ifdef DEBUG_NOW
	printf( "Wait Device In\n" );
#endif

	P4_DIR |= 1<<3;       		// 设置上拉
	P4_OUT |= 1<<3;				// 设置方向输出
	P4_OUT &= ~(1<<3);			// 点亮

	
	while ( 1 ) 
	{
		if(UIF_DETECT)
		{
			UIF_DETECT = 0;
			s = AnalyzeRootHub( );  			    // 分析ROOT-HUB状态;
			if ( s == ERR_USB_CONNECT ) 
			{  										// 有新的USB设备插入
				mDelaymS( 200 );  					// 由于USB设备刚插入尚未稳定,故等待USB设备数百毫秒,消除插拔抖动
				s = InitRootDevice( );  			// 初始化USB设备			
				if ( s == ERR_SUCCESS ) 
				{
/* U盘操作流程：USB总线复位、U盘连接、获取设备描述符和设置USB地址、可选的获取配置描述符，之后到达此处，由CH559子程序库继续完成后续工作 */
					CH559DiskStatus = DISK_USB_ADDR;
					for ( i = 0; i != 10; i ++ ) {
#ifdef DEBUG_NOW
						printf( "Wait DiskReady\n" );
#endif
						s = CtrlGetMaxLUN();
//						if(s == ERR_SUCCESS)
//						{
//							s = CH559ReadDiskInfo(TxBuffer );
							if(s == ERR_SUCCESS)		break;
//						}	
					}
/* 查找文件并获取文件大小 */	
#ifdef DEBUG_NOW
					printf( "DBR \n" );
#endif
					s = CH559ReadDiskDBR(TxBuffer);
#ifdef DEBUG_NOW
					printf( "CHECK \n" );
#endif
					if( s != ERR_SUCCESS) 	IAP_END();
					else					s = CH559FileOpen(TxBuffer);
/* 读取文件内容并下载 */		
#ifdef DEBUG_NOW					
					printf("Read file: %02x\n",(UINT16)s);
#endif
					if(s != ERR_SUCCESS){
#ifdef DEBUG_NOW
						printf("There is no file! \n");
#endif
						IAP_END();						
					}
					else{
						IAP_ERASE()	;								
						s = CH559ReadFile(TxBuffer);		//读取文件内容并下载
						if(s == ERR_SUCCESS){
							P4_OUT |= (1<<3);
							IAP_END();

						}
					}	
				}
				else
				{
#ifdef DEBUG_NOW
					printf("初始化U盘失败，请拔下U盘重试\n");
#endif
				}
			}
		}
	}
}

