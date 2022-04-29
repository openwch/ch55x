
/********************************** (C) COPYRIGHT *******************************
* File Name          : SPI0Flash.C
* Author             : WCH
* Version            : V1.2
* Date               : 2016/1/15
* Description        : CH559 SPI0 读写外部Flash
*******************************************************************************/
#include ".\DEBUG.C"                                                          //调试信息打印
#include ".\DEBUG.H"
#include "string.h"
#pragma  NOAREGS

#define THIS_ENDP0_SIZE         DEFAULT_ENDP0_SIZE
UINT8X  Ep0Buffer[THIS_ENDP0_SIZE] _at_ 0x0000;                                //端点0 OUT&IN缓冲区，必须是偶地址
UINT8X  Ep1Buffer[2*MAX_PACKET_SIZE] _at_ 0x0008;                                //端点1 IN缓冲区,必须是偶地址
UINT8   UsbConfig = 0; 

#define UsbSetupBuf     ((PUSB_SETUP_REQ)Ep0Buffer)

/*设备描述符*/
UINT8C  MyDevDescr[] = { 0x12, 0x01, 0x10, 0x01,
                         0x00, 0x00, 0x00, THIS_ENDP0_SIZE,
                         0x44, 0x33, 0x33, 0x35,                              // 厂商ID和产品ID
                         0x00, 0x01, 0x01, 0x02,
                         0x00, 0x01
                       };
/*配置描述符*/
UINT8C  MyCfgDescr[] = { 0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
                         0x09, 0x04, 0x00, 0x00, 0x02, 0x08, 0x06, 0x50, 0x00,                     
                         0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00,
                         0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00
                       };
/*语言描述符*/
UINT8C  MyLangDescr[] = { 0x04, 0x03, 0x09, 0x04 };
/*厂家信息*/
UINT8C  MyManuInfo[] = { 0x0E, 0x03, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0 };
/*产品信息*/
UINT8C  MyProdInfo[] = { 0x0C, 0x03, 'C', 0, 'H', 0, '5', 0, '5', 0, '9', 0 };

UINT8C  MAX_LUN[] = {0};

//INQUIRY inform
UINT8C	DBINQUITY[]={
						0x00,             //Peripheral Device Type
						0x80, 			//
						0x02 ,			//ISO/ECMA
						0x02 ,			//
						0x1f ,			//Additional Length

						00 ,			//Reserved
						00 ,			//Reserved
						00 ,				//Reserved


					    'w' ,			//Vendor Information
						'c' ,			//
						'h' ,			//
						'.' ,			//
						'c' ,			//
						'n' ,			//
						' ' ,			//
						' ' ,			//


        			    0xc7,			//Product Identification
						0xdf, 			//
						0xba,			//
						0xe3,			//
						0xb5,			//
						0xe7,			//
						0xd7,			//
						0xd3,			//
						0x55,			//
						0xc5,			//
						0xcc,			//
						0xb7,			//
						0xbd,			//
						0xb0,			//
						0xb8,			//
						0x00,          //

               			'1' ,			//Product Revision Level
						'.' ,			//
						'1' ,			//
						'0'  			//
						  };

//修改恒定义的时候注意修改擦除函数，当前为擦除4k函数。需要相应修改这个函数即可。
						  
#define DISK_SEC_NUM   0x00000200       //总扇区数  共512物理扇区
#define DISK_SEC_LAST  DISK_SEC_NUM- 1  //最后一个逻辑扇区地址				  
#define DISK_SEC_LEN   0x00001000       //扇区大小  每个扇区4096字节大小
						  					  
						  
UINT8C DBCAPACITY[]={ (DISK_SEC_LAST>>24)&0xFF, (DISK_SEC_LAST>>16)&0xFF, (DISK_SEC_LAST>>8)&0xFF, DISK_SEC_LAST&0xFF , (DISK_SEC_LEN>>24)&0xFF, (DISK_SEC_LEN>>16)&0xFF, (DISK_SEC_LEN>>8)&0xFF, DISK_SEC_LEN&0xFF};    //last logic addr//block lenth

UINT8C  modesense3F[]={
				0x0b, 0x00, 0x00, 0x08, (DISK_SEC_NUM>>24)&0xFF, (DISK_SEC_NUM>>16)&0xFF, (DISK_SEC_NUM>>8)&0xFF, DISK_SEC_NUM&0xFF, 00, 00, 02, 00 };   //物理扇区数

UINT8C  mode5sense3F[]={
				0x00, 0x06, 0x00, 0x00, 0x08, 0x00, 0x00, 0x08,(DISK_SEC_NUM>>24)&0xFF, (DISK_SEC_NUM>>16)&0xFF, (DISK_SEC_NUM>>8)&0xFF, DISK_SEC_NUM&0xFF, 00, 00, 02, 00 };  //物理扇区数


sbit CHIP_SELECT = P1^4;
#define SENDBYTE_SPI( d )    {  SPI0_DATA = d;while(S0_FREE == 0); }
#define RECVBYTE_SPI( d )    { SPI0_DATA = 0xff;while(S0_FREE == 0);d = SPI0_DATA;}

UINT8 buffer[64]; 

typedef union _CBWCB{
		unsigned char buf1[16];
}CBWCB;

typedef  union _MASS_PARA {
		unsigned char buf[64];
		struct  _SENSE{
			unsigned char ErrorCode;
			unsigned char Reserved1;
			unsigned char SenseKey;
			unsigned char Information[4];
			unsigned char AddSenseLength;
			unsigned char Reserved2[4];
			unsigned char AddSenseCode;
			unsigned char AddSenseCodeQua;
			unsigned char Reserved3[4];
		}Sense;
	//	unsigned char SenseData[18];
		struct  _CBW{
		unsigned char dCBWsig[4];
		unsigned char dCBWTag[4];
		unsigned long dCBWDatL;
		unsigned char bmCBWFlags;
		unsigned char bCBWLUN;
		unsigned char bCBWCBLength;
		CBWCB        cbwcb;
	}cbw;
		struct _CSW{

		unsigned char buf2[13];
	}csw;


}MASS_PARA;

enum _HOST_DEV_DISAGREE {
CASEOK = 0,
CASE1,
CASE2,
CASE3,
CASE4,
CASE5,
CASE6,
CASE7,
CASE8,
CASE9,
CASE10,
CASE11,
CASE12,
CASE13,
CASECBW,
CASECMDFAIL
};

#define FORMAT_UNIT 	0x04
#define INQUIRY 		0x12
#define MODE_SELECT 	0x15
#define MODE_SENSE5 	0x5A
#define MODE_SENSE 		0x1A
#define PER_RES_IN 		0x5E
#define PER_RES_OUT 	0x5F
#define PRE_OR_MED 		0x1E
#define READ 			0x28
#define READ_CAPACITY 	0x25
#define RELEASE 		0x17
#define REQUEST_SENSE 	0x03
#define RESERVE 		0x16
#define STA_STO_UNIT 	0x1B
#define SYN_CACHE 		0x35
#define TES_UNIT 		0x00
#define VERIFY 			0x2F
#define WRITE 			0x2A
#define WRITE_BUFFER 	0x3B


union {
unsigned long mDataLength;							//数据长度
unsigned char mdataLen[4];							//
} LEN;


unsigned char mdCBWTag[4];						//dCBWTag
MASS_PARA  MassPara;
bit CH375BULKUP;									//数据上传
bit CH375BULKDOWN;									//数据下传
bit CH375CSW;										//CSW上传标志
unsigned char	BcswStatus;							//CSW状态
unsigned char mSenseKey;
unsigned char mASC;
bit FSTALL;											//数据错误标志
bit lastFSTALL;
bit pBuf_ReSelect = 0;

UINT32 Locate_Addr;

unsigned char *pBuf;

/*******************************************************************************
* Function Name  : USBDeviceCfg()
* Description    : USB设备模式配置
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void    InitUSB_Device( void )                                                      // 初始化USB设备
{
    IE_USB = 0;
    USB_CTRL = 0x00;                                                                // 先设定模式
    UEP4_1_MOD = bUEP1_TX_EN | bUEP1_RX_EN;                                         // 端点1收发模式
    UEP0_DMA = Ep0Buffer;
    UEP1_DMA = Ep1Buffer;
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
    USB_DEV_AD = 0x00;
    UDEV_CTRL = bUD_DP_PD_DIS | bUD_DM_PD_DIS;                                      // 禁止DP/DM下拉电阻
    USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;                           // 启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
    UDEV_CTRL |= bUD_PORT_EN;                                                       // 允许USB端口
    USB_INT_FG = 0xFF;                                                              // 清中断标志
    USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
    IE_USB = 1;
	UEP1_T_LEN = 0;                                                       //预使用发送长度一定要清空 
	
	EA = 1;
}


/*******************************************************************************
* Function Name  : InitHostSPI0( void )
* Description    : SPI0主机模式初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void    InitHostSPI0( void )
{
    SPI0_SETUP &=~(bS0_MODE_SLV | bS0_BIT_ORDER);                              /*设置成主机模式*/
    SPI0_CTRL = bS0_SCK_OE | bS0_MOSI_OE;                                      /*主机写，默认不启动写传输，如果使能bS0_DATA_DIR*/
                                                                               /*那么发送数据后自动产生一个字节的时钟，用于快速数据收发*/
    P1_DIR |= (bMOSI | bSCK | bSCS| bPWM3 );                                   /*bMOSI 、bSCK 、bSCS置为输出方向*/
    P1_DIR &= ~bMISO;
    SPI0_CK_SE = 0x02; //0x02                                                        /*分频为12M*/
//  SPI0_STAT = 0xFF;                                                          /*清中断标志*/
//  IE_SPI0 = 1;
}

/*******************************************************************************
* Function Name  : ReadExternalFlashStatusReg_SPI
* Description    : 用来读取状态寄存器,并返回状态寄存器的值
* Input          : None
* Output         : None
* Return         : ExFlashRegStatus
*******************************************************************************/
UINT8 ReadExternalFlashStatusReg_SPI( void )
{
    UINT8 ExFlashRegStatus;
    CHIP_SELECT = 0;
    SENDBYTE_SPI(0x05);                                                        /*发送读状态寄存器的命令 */
    RECVBYTE_SPI(ExFlashRegStatus);                                            /*读取状态寄存器*/
    CHIP_SELECT = 1 ;
    return ExFlashRegStatus;
}
   
/*******************************************************************************
* Function Name  : WaitExternalFlashIfBusy
* Description    : 等待芯片空闲(在执行Byte-Program, Sector-Erase, Block-Erase, Chip-Erase操作后)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WaitExternalFlashIfBusy( void )
{
    while ((ReadExternalFlashStatusReg_SPI())&0x01 == 0x01 )                   /*等待直到Flash空闲*/
    {
        ReadExternalFlashStatusReg_SPI( );
    }
}

/*******************************************************************************
* Function Name  : WriteExternalFlashStatusReg_SPI
* Description    : 往状态寄存器里写一个字节
* Input          : status -写入的数据
* Output         : None
* Return         : None
*******************************************************************************/
void WriteExternalFlashStatusReg_SPI( UINT8 status )
{
    CHIP_SELECT = 0 ;
    SENDBYTE_SPI(0x01);                                                       /*发送写状态寄存器*/
    SENDBYTE_SPI(status);                                                     /*改变寄存器里BPx或者BPL (只有2,3,4,5,7位可以改写)*/
    CHIP_SELECT = 1 ;
}
 /*********************************************************
*函数名：  static void st25vf016b_WRDI(void)
*函数功能：写禁止
*参数：    无
*返回值：  无                                                                                                    
***************************************************************/
void WriteFlashForbidden(void)
{
	CHIP_SELECT = 0 ;
	SENDBYTE_SPI(0x04);
	CHIP_SELECT = 1 ;   
}
/*******************************************************************************
* Function Name  : WriteExternalFlashEnable_SPI
* Description    : 写使能,同样可以用于使能写状态寄存器
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WriteExternalFlashEnable_SPI( void )
{
    CHIP_SELECT = 0 ;
    SENDBYTE_SPI(0x06);                                                       /*发送写使能命令*/
    CHIP_SELECT = 1 ;
}

/*******************************************************************************
* Function Name  : CheckExternalFlashWriteEnable_SPI
* Description    : 检查擦写操作前WEL位是否为1
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CheckExternalFlashWriteEnable_SPI( void )
{
    UINT8 WRENStatus;
    WRENStatus = ReadExternalFlashStatusReg_SPI();                            /*读取状态register*/
    if ((WRENStatus&0x02) != 0x02)                                            /*检查WEL位置位*/
    {
        WriteExternalFlashEnable_SPI( );                                      /*如果未置1进行相应处理,如对其进行写使能操作*/
    }
}

/*******************************************************************************
* Function Name  : EraseExternalFlash_SPI
* Description    : 擦除4K Flash  擦除一个扇区
* Input          : Dst_Addr 0-1 ffff ffff ,清除任意地址所在的扇区。
* Output         : None
* Return         : None
*******************************************************************************/
void EraseExternalFlash_SPI( UINT32 Dst_Addr )
{
//	WriteExternalFlashStatusReg_SPI( 0x00 ); //设置芯片为无保护
//	WriteExternalFlashEnable_SPI();//写使能
    CheckExternalFlashWriteEnable_SPI();
    CHIP_SELECT = 0 ;
    SENDBYTE_SPI(0x20);    //扇区擦除命令         //0x60擦整片；0x20擦除一个扇区
    SENDBYTE_SPI(((Dst_Addr & 0xFFFFFF) >> 16));                                //send 3 address bytes  
    SENDBYTE_SPI(((Dst_Addr & 0xFFFF) >> 8));
    SENDBYTE_SPI(Dst_Addr & 0xFF);	
	
    CHIP_SELECT = 1 ;
    WaitExternalFlashIfBusy();
}

/*******************************************************************************
* Function Name  : ByteReadExternalFlash_SPI
* Description    : 读取一个地址内一个字节的数据.返回读取的数据 
* Input          : UINT32 StarAddr -Destination Address 000000H - 1FFFFFH
* Output         : None
* Return         : byte -读取的数据
*******************************************************************************/
UINT8 ByteReadExternalFlash_SPI(UINT32 StarAddr)    
{   
    UINT8 dat = 0;    
    CHIP_SELECT = 0 ;                                                           //enable device  
    SENDBYTE_SPI(0x03);                                                         //read command 
    SENDBYTE_SPI(((StarAddr & 0xFFFFFF) >> 16));                                //send 3 address bytes  
    SENDBYTE_SPI(((StarAddr & 0xFFFF) >> 8));
    SENDBYTE_SPI(StarAddr & 0xFF);
    RECVBYTE_SPI(dat);   
    CHIP_SELECT = 1 ;                                                           //disable device   
    return dat;                                                                 //return one byte read
} 

/*******************************************************************************
* Function Name  : ByteWriteExternalFlash_SPI
* Description    : 写数据
* Input          : StarAddr  -Destination Address 000000H - 1FFFFFH
*                  dat -要写入的数据
* Output         : None
* Return         : None
*******************************************************************************/
void ByteWriteExternalFlash_SPI(UINT32 StarAddr, UINT8 dat)
{
    WriteExternalFlashEnable_SPI();
    CHIP_SELECT = 0 ;                                                          //芯片使能 
    SENDBYTE_SPI(0x02);                                                        //发送写操作指令 
    SENDBYTE_SPI(((StarAddr & 0xFFFFFF) >> 16));                                    //发送3字节地址 
    SENDBYTE_SPI(((StarAddr & 0xFFFF) >> 8));
    SENDBYTE_SPI(StarAddr & 0xFF);
    SENDBYTE_SPI(dat);                                                         //发送要写的数据
    CHIP_SELECT = 1 ;
    WaitExternalFlashIfBusy();
}

/*******************************************************************************
* Function Name  : BlukReadExternalFlash_SPI
* Description    : 读取起始地址(StarAddr)内多个字节(Len)的数据.存入缓冲区RcvBuffer中
* Input          : StarAddr -Destination Address 000000H - 1FFFFFH
                   Len 读取数据长度
                   RcvBuffer 接收缓冲区起始地址
* Output         : None
* Return         : None
*******************************************************************************/
void BlukReadExternalFlash_SPI(UINT32 StarAddr,UINT16 Len,PUINT8 RcvBuffer)
{
	UINT16 i;	
    CHIP_SELECT = 0 ;                                                           //enable device  
    SENDBYTE_SPI(0x0b);                                                         //read command 
    SENDBYTE_SPI(((StarAddr & 0xFFFFFF) >> 16));                                //send 3 address bytes  
    SENDBYTE_SPI(((StarAddr & 0xFFFF) >> 8));
    SENDBYTE_SPI(StarAddr & 0xFF);
	SENDBYTE_SPI(0x00);
	for(i=0; i<Len; i++)                                                        /*接收数据*/
    {
		RECVBYTE_SPI(RcvBuffer[i]);                                                            /*读取下一地址*/
    }
    CHIP_SELECT = 1 ;                                                           //disable device   
}

/*******************************************************************************
* Function Name  : BlukWriteExternalFlash_SPI
* Description    : 将数据写入外部Flash
* Input          : StarAddr  -Destination Address 000000H - 1FFFFFH
                   Len 发送数据长度
*                  SendBuffer -发送数据缓冲区
* Output         : None
* Return         : None
*******************************************************************************/
void BlukWriteExternalFlash_SPI(UINT32 StarAddr,UINT16 Len,PUINT8 SendBuffer)
{
	unsigned int timeout = 0;
	unsigned int i = 0;
	
    WriteExternalFlashStatusReg_SPI( 0x00 ); //设置芯片为无保护
	WriteExternalFlashEnable_SPI();//写使能
	
	CHIP_SELECT = 0 ;
	SENDBYTE_SPI(0xad);             //自动地址增加字(2byte)编程模式
	
    SENDBYTE_SPI(((StarAddr & 0xFFFFFF) >> 16));                                    //发送3字节地址 
    SENDBYTE_SPI(((StarAddr & 0xFFFF) >> 8));
    SENDBYTE_SPI(StarAddr & 0xFF);
	
	SENDBYTE_SPI(SendBuffer[i]);
	SENDBYTE_SPI(SendBuffer[i+1]);	
	CHIP_SELECT = 1 ; 	
	
	WaitExternalFlashIfBusy();	  //等待字编程完成

	for(i = 2; i < Len-1; i++)
	{
		CHIP_SELECT = 0 ;
		SENDBYTE_SPI(0xad); //自动地址增加字(2byte)编程模式
		SENDBYTE_SPI(SendBuffer[i++]);
	    SENDBYTE_SPI(SendBuffer[i]);	
	    CHIP_SELECT = 1 ;
		
		WaitExternalFlashIfBusy();	  //等待字编程完成
    }	
	
	WriteFlashForbidden();//写禁止
	WaitExternalFlashIfBusy();	  //等待字编程完成
	
	
}

void main( ) 
{
    mDelaymS(30);                                                                  //上电延时,等待内部晶振稳定,必加 
    mInitSTDIO( );                                                                 /* 为了让计算机通过串口监控演示过程 */
    printf( "Start SPI FLASH @ChipID=%02X\n", (UINT16)CHIP_ID );

    InitHostSPI0( );
    WriteExternalFlashEnable_SPI( );                                               //FLASH写使能
    WriteExternalFlashStatusReg_SPI( 0x00 );                                       //写寄存器 
	
	InitUSB_Device();
	
	while(1);
}

//BLOCK ONLY  The Thirteen Cases

void  BulkThirteen(unsigned char Case)
{
	switch(Case)
	{
	case CASEOK:
	case CASE1:     									/* Hn=Dn*/
	case CASE6:     									/* Hi=Di*/
		BcswStatus = 0;
		break;
	case CASE12:    									/* Ho=Do*/
		BcswStatus = 0;
		break;

	case CASE2:     									/* Hn<Di*/
	case CASE3:     									/* Hn<Do*/

		UEP1_CTRL = UEP1_CTRL | MASK_UEP_T_RES ;
		FSTALL=1;
														//这里上传端点设置一个STALL，待主机清掉 // may or may-not
		BcswStatus =2;
		break;

	case CASE4:     									/* Hi>Dn*/
	case CASE5:     									/* Hi>Di*/


		UEP1_CTRL = UEP1_CTRL | MASK_UEP_T_RES ;
		FSTALL=1;
														//这里上传端点设置一个STALL，待主机清掉
		BcswStatus= 1;									//CSW_GOOD or CSW_FAIL
		break;


	case CASE7:    										 /* Hi<Di*/
	case CASE8:     									/* Hi<>Do */

		UEP1_CTRL = UEP1_CTRL | MASK_UEP_T_RES ;
			FSTALL=1;
													//这里上传端点设置一个STALL，待主机清掉
		BcswStatus = 2;
		break;

	case CASE9:     										/* Ho>Dn*/
	case CASE11:    									/* Ho>Do*/

		UEP1_CTRL = UEP1_CTRL | MASK_UEP_R_RES ;
		FSTALL=1;
													//这里上传端点设置一个STALL，待主机清掉
	BcswStatus =1;									//CSW_GOOD or CSW_FAIL
		break;

	case CASE10:    								/* Ho<>Di */
	case CASE13:    								/* Ho<Do*/

		UEP1_CTRL = UEP1_CTRL | MASK_UEP_T_RES ;
		FSTALL=1;
													//这里上传端点设置一个STALL，待主机清掉
		UEP1_CTRL = UEP1_CTRL | MASK_UEP_R_RES ;
													//这里上传端点设置一个STALL，待主机清掉
		BcswStatus = 2;
		break;

	case CASECBW:   								/* invalid CBW */

	    UEP1_CTRL = UEP1_CTRL | MASK_UEP_T_RES ;
		FSTALL=1;
		UEP1_CTRL = UEP1_CTRL | MASK_UEP_R_RES ;
											//这里端点设置一个STALL，待主机清掉
		BcswStatus = 2;
		break;

	case CASECMDFAIL:

		UEP1_CTRL = UEP1_CTRL | MASK_UEP_T_RES ;
		FSTALL=1;
												//这里上传端点设置一个STALL，待主机清掉
		BcswStatus= 1;
		break;

	default:
		break;
	}
}



				
void  UFI_readCapacity(void ){
	
		if ( LEN.mDataLength > sizeof(DBCAPACITY) ) LEN.mDataLength = sizeof(DBCAPACITY);
		pBuf=(unsigned char*)DBCAPACITY;	
		BcswStatus=0;
		mSenseKey=0;
		mASC=0;
}

void  UFI_inquiry(void ){
		pBuf = DBINQUITY;					////查询U盘信息
		if(LEN.mDataLength>36) LEN.mDataLength=36;
		BcswStatus=0;
		mSenseKey=0;
		mASC=0;
	
}
void  UFI_read10(void){
													//读取数据
		LEN.mDataLength=(((UINT32)MassPara.cbw.cbwcb.buf1[7]<<8) | (UINT32)MassPara.cbw.cbwcb.buf1[8])*DISK_SEC_LEN;
		Locate_Addr = ((UINT32)MassPara.cbw.cbwcb.buf1[2]<<24) | ((UINT32)MassPara.cbw.cbwcb.buf1[3]<<16) | ((UINT32)MassPara.cbw.cbwcb.buf1[4]<<8) | (UINT32)MassPara.cbw.cbwcb.buf1[5];
	
		Locate_Addr = Locate_Addr*DISK_SEC_LEN;
	
	//	printf("Read addr:%ld  len:%ld \n",Locate_Addr,LEN.mDataLength);
	
		BcswStatus=0;
		mSenseKey=0;
		mASC=0;
	
		pBuf_ReSelect = 1;

}

void  UFI_modeSense(void ){
										//模式认识
		if(MassPara.cbw.cbwcb.buf1[2]==0x3F){
			if ( LEN.mDataLength > sizeof(modesense3F) ) LEN.mDataLength = sizeof(modesense3F);
			pBuf=modesense3F;
			BcswStatus=0;
			mSenseKey=0;
			mASC=0;
		}
		else {
			CH375BULKUP=0;
			mSenseKey=5;
			mASC=0x20;

			BcswStatus=1;
			BulkThirteen(CASECMDFAIL);
	   	}
}

void  UFI_requestSense(void ){
											//请求认识
	if ( FSTALL | lastFSTALL ) {
		lastFSTALL=FSTALL;
		FSTALL=0;
		MassPara.Sense.ErrorCode=0x70;
		MassPara.Sense.Reserved1=0;
		MassPara.Sense.SenseKey=mSenseKey;
		MassPara.Sense.Information[0]=0;
		MassPara.Sense.Information[1]=0;
		MassPara.Sense.Information[2]=0;
		MassPara.Sense.Information[3]=0;
		MassPara.Sense.AddSenseLength=0x0a;
		MassPara.Sense.Reserved2[0]=0;
		MassPara.Sense.Reserved2[1]=0;
		MassPara.Sense.Reserved2[2]=0;
		MassPara.Sense.Reserved2[3]=0;
		MassPara.Sense.AddSenseCode=mASC;
		MassPara.Sense.AddSenseCodeQua=00;
		MassPara.Sense.Reserved3[0]=0;
		MassPara.Sense.Reserved3[1]=0;
		MassPara.Sense.Reserved3[2]=0;
		MassPara.Sense.Reserved3[3]=0;
		pBuf=MassPara.buf;
		BcswStatus=0;
	}
	else {
		lastFSTALL=FSTALL;
		FSTALL=0;
		MassPara.Sense.ErrorCode=0x70;
		MassPara.Sense.Reserved1=0;
		MassPara.Sense.SenseKey=0x00;
		MassPara.Sense.Information[0]=0;
		MassPara.Sense.Information[1]=0;
		MassPara.Sense.Information[2]=0;
		MassPara.Sense.Information[3]=0;
		MassPara.Sense.AddSenseLength=0x0a;
		MassPara.Sense.Reserved2[0]=0;
		MassPara.Sense.Reserved2[1]=0;
		MassPara.Sense.Reserved2[2]=0;
		MassPara.Sense.Reserved2[3]=0;
		MassPara.Sense.AddSenseCode=0x00;
		MassPara.Sense.AddSenseCodeQua=00;
		MassPara.Sense.Reserved3[0]=0;
		MassPara.Sense.Reserved3[1]=0;
		MassPara.Sense.Reserved3[2]=0;
		MassPara.Sense.Reserved3[3]=0;
		pBuf=MassPara.buf;
		BcswStatus=0;
	}
}
void  UFI_testUnit(void ){
		CH375BULKDOWN=0;
		CH375BULKUP=0;
		BcswStatus=0;			//测试U盘是否准备好
		mSenseKey=0;
		mASC=0;
}
void  UFI_perOrMed(void ){				//允许移出磁盘
		BcswStatus=0;
		mSenseKey=0;
		mASC=0;
}
void  UFI_write(void ){
		UINT8 i,num;
		LEN.mDataLength=(((UINT32)MassPara.cbw.cbwcb.buf1[7]<<8) | (UINT32)MassPara.cbw.cbwcb.buf1[8])*DISK_SEC_LEN;		//写数据长度
		Locate_Addr = ((UINT32)MassPara.cbw.cbwcb.buf1[2]<<24) | ((UINT32)MassPara.cbw.cbwcb.buf1[3]<<16) | ((UINT32)MassPara.cbw.cbwcb.buf1[4]<<8) | (UINT32)MassPara.cbw.cbwcb.buf1[5];
		Locate_Addr = Locate_Addr*DISK_SEC_LEN;
	//擦除
		num = MassPara.cbw.cbwcb.buf1[8];   //待写扇区数
		for(i=0;i<num;i++)
			EraseExternalFlash_SPI(Locate_Addr + i*DISK_SEC_LEN);  

		BcswStatus=0;
		mSenseKey=0;
		mASC=0;
	
}
void  UFI_staStoUnit(void ){     //请求装载卸载设备
		CH375BULKDOWN=0;
		CH375BULKUP=0;
		BcswStatus=0;
			mSenseKey=0;
			mASC=0;
}
void  UFI_verify(void ){
		BcswStatus=0;		//校验存储器空间
		mSenseKey=0;
		mASC=0;
					//这里这里只是作为演示所以没有真正检测物理存储器
					//但实际上这一步一定要处理
}
void  UFI_modeSense5(void ){

		if(MassPara.cbw.cbwcb.buf1[2]==0x3F){
			if ( LEN.mDataLength > sizeof(mode5sense3F) ) LEN.mDataLength = sizeof(mode5sense3F);
			pBuf=mode5sense3F;
			BcswStatus=0;
			mSenseKey=0;
			mASC=0;
		}
		else {
			CH375BULKUP=0;
			mSenseKey=5;
			mASC=0x20;
			BcswStatus=1;
			BulkThirteen(CASECMDFAIL);
	   	}
}
//UFI  CMD
void UFI_Hunding(void ){		
		switch(MassPara.cbw.cbwcb.buf1[0]){
			case INQUIRY:
				UFI_inquiry();
			break;
			case WRITE:
				
				//printf("%2x\n",(UINT16)MassPara.cbw.cbwcb.buf1[0]);			
               UFI_write();
			break;
			case TES_UNIT:
				UFI_testUnit();
			break;
			case READ:
				UFI_read10();
			break;
			case REQUEST_SENSE:
				UFI_requestSense();
			break;
			case READ_CAPACITY:
                UFI_readCapacity();
			break;
			case VERIFY:
				UFI_verify();
				break;
//		//	case 0x23:

//		//	break;
//		//	case MODE_SELECT:
//			//	UFI_modeSlect();
//		//	break;
			case MODE_SENSE:
				UFI_modeSense();
			break;
			case MODE_SENSE5:
				UFI_modeSense5();
			break;
//		//	case WRITE_BUFFER:
//		//		UFI_writeBuf();
//	//		break;
//	//		case PREVENT:
//		//	break;
//	//		case FORMAT_UNIT:
//		//		UFI_format();
//	//		break;
//	//		case RELEASE:
//	//		break;
			case STA_STO_UNIT:
				UFI_staStoUnit();
			break;
			case PRE_OR_MED:
				UFI_perOrMed();
			break;
			default:
				
			//printf("%2x\n",(UINT16)MassPara.cbw.cbwcb.buf1[0]);
			
				mSenseKey=5;
				mASC=0x20;
				BcswStatus=1;
				CH375BULKUP=0;
				BulkThirteen(CASECBW);
				break;
			}
}

void mCH375BulkOnly(){
			if(MassPara.buf[0]==0x55){
				if(MassPara.buf[1]==0x53){
				   if(MassPara.buf[2]==0x42){
				    	if(MassPara.buf[3]==0x43){
//							LEN.mDataLength=BIG_ENDIAN(MassPara.cbw.dCBWDatL);			//做BO协议处理
							LEN.mdataLen[3] = *(unsigned char *)(&MassPara.cbw.dCBWDatL);  /* 将PC机的低字节在前的16位字数据转换为C51的高字节在前的数据 */
							LEN.mdataLen[2] = *( (unsigned char *)(&MassPara.cbw.dCBWDatL) + 1 );
							LEN.mdataLen[1] = *( (unsigned char *)(&MassPara.cbw.dCBWDatL) + 2 );
							LEN.mdataLen[0] = *( (unsigned char *)(&MassPara.cbw.dCBWDatL) + 3 );
							mdCBWTag[0]=MassPara.buf[4];
                             mdCBWTag[1]=MassPara.buf[5];
                              mdCBWTag[2]=MassPara.buf[6];
	                          mdCBWTag[3]=MassPara.buf[7];													  //取出数据长度
							if(LEN.mDataLength){
									CH375BULKDOWN=(MassPara.cbw.bmCBWFlags&0X80)?0:1;	//判断是上传还是下传数据
									CH375BULKUP=(MassPara.cbw.bmCBWFlags&0X80)?1:0;
								}
						  // if(!CBWLUN){      		//只支持一个物理盘
								CH375CSW=1;
								UFI_Hunding();
							//调用UFI协议处理
						//	}
							//	else ;//此处应做错误处理
					   }
						else
						UEP1_CTRL = UEP1_CTRL | MASK_UEP_T_RES ;
				  }
				   else
					UEP1_CTRL = UEP1_CTRL | MASK_UEP_T_RES ;
			     }
				else
				UEP1_CTRL = UEP1_CTRL | MASK_UEP_T_RES ;
			 }
			else
			UEP1_CTRL = UEP1_CTRL | MASK_UEP_T_RES ;
}

void mCH375UpCsw()
{
		unsigned char i;													//如果数据为0
		pBuf=&MassPara.buf[0];
		CH375CSW=0;																	//上传CSW
		CH375BULKUP=0;														//取消数据上传
		MassPara.buf[0]=0x55;												//dCSWSignature
		MassPara.buf[1]=0x53;
		MassPara.buf[2]=0x42;
		MassPara.buf[3]=0x53;
		MassPara.buf[4]=mdCBWTag[0];
		MassPara.buf[5]=mdCBWTag[1];
		MassPara.buf[6]=mdCBWTag[2];
		MassPara.buf[7]=mdCBWTag[3];
		MassPara.buf[8]=0;
		MassPara.buf[9]=0;
		MassPara.buf[10]=LEN.mdataLen[1];
		MassPara.buf[11]=LEN.mdataLen[0];
		MassPara.buf[12]=BcswStatus;
		for(i = 0;i<13;i++)
		{
			Ep1Buffer[MAX_PACKET_SIZE+i] = *pBuf;
			pBuf++;
		}
		UEP1_T_LEN = 13;
		UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;  // 允许上传
}

//**********************************************************************************
void CH375bulkUpData(){											//调用端点2上传数据
		unsigned char len,i;
		if(LEN.mDataLength>0x40){
			len=0x40;
			LEN.mDataLength-=0x40;
		}
		else {
			len= (unsigned char) LEN.mDataLength;
			LEN.mDataLength=0;
			CH375BULKUP=0;
		}
		
		if(pBuf_ReSelect)
		{
			BlukReadExternalFlash_SPI(Locate_Addr,len,&Ep1Buffer[MAX_PACKET_SIZE]);
			Locate_Addr += len;		
			if(LEN.mDataLength==0)
				pBuf_ReSelect = 0;
		}
		else
		{
			for(i = 0;i<len;i++)
			{
				Ep1Buffer[MAX_PACKET_SIZE+i] = *pBuf;
				pBuf++;
			}
		}
		UEP1_T_LEN = len;
		UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;  // 允许上传
}


void mCH375BulkDownData(){
		unsigned char len,i;
	
		len = USB_RX_LEN;																								
		for(i=0;i!=len;i++)						
			buffer[i]=Ep1Buffer[i];	//将数据读入到缓冲区
	
		BlukWriteExternalFlash_SPI(Locate_Addr,len,&buffer[0]);
		Locate_Addr += len;
	
		LEN.mDataLength-=len;				//全局数据长度减掉当前获得的长度

		if(LEN.mDataLength==0){														//如果数据为0,则传送CSW
			CH375BULKDOWN=0;
			mCH375UpCsw();				//上传CSW
	}
}

/*******************************************************************************
* Function Name  : USB_DeviceInterrupt()
* Description    : CH559USB模拟设置中断处理函数
*******************************************************************************/
void    USB_DeviceInterrupt( void ) interrupt INT_NO_USB using 1               /* USB中断服务程序,使用寄存器组1 */
{
    UINT8   len,length;
    static  UINT8   SetupReqCode, SetupLen;
    static  PUINT8  pDescr;
	
    if ( UIF_TRANSFER )                                                        // USB传输完成
    {
        if ( U_IS_NAK )                                                        // not enable for this example
        {
//          switch ( USB_INT_ST & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) )         // 分析操作令牌和端点号
//          {  
//              case UIS_TOKEN_OUT | 2:                                        // endpoint 2# 批量端点下传
//                  break;
//              case UIS_TOKEN_IN | 2:                                         // endpoint 2# 批量端点上传
//                  break;
//              case UIS_TOKEN_IN | 1:                                         // endpoint 1# 中断端点上传
//                  break;
//              default:
//                  break;
//          }
            printf("NAK INT,PrepareData\n");
        }
        else
        {
            switch ( USB_INT_ST & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) )         // 分析操作令牌和端点号
            {
            case UIS_TOKEN_IN | 1:                                             // endpoint 1# 中断端点上传
				if(CH375BULKUP) CH375bulkUpData();								//调用数据上传
				else if(CH375CSW) mCH375UpCsw();									//上传CSW
				else
                UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;      // 暂停上传
                break;
			case UIS_TOKEN_OUT | 1:
                if ( U_TOG_OK )                                                // 不同步的数据包将丢弃
                {
					if(CH375BULKDOWN)  mCH375BulkDownData();									//如果上传数据阶段则调用数据上传
					else{	 										//不是数据下传则判断是否
							length = USB_RX_LEN;
							if(!length)break;								//数据包长度为零则跳出																		
							for(len=0;len!=length;len++)						
								MassPara.buf[len]=Ep1Buffer[len];	//将数据读入到缓冲区

							mCH375BulkOnly();

							if(!CH375BULKDOWN){
								if(CH375BULKUP) CH375bulkUpData();					//调用批量数据上传
								else if(!FSTALL) mCH375UpCsw();								//没有数据上传调用CSW上传
																				//在这里做上传数据调用
							}
					}	
                }				
			
				break;
            case UIS_TOKEN_SETUP | 0:                                          // endpoint 0# SETUP
                len = USB_RX_LEN;
                if ( len == sizeof( USB_SETUP_REQ ) )                          // SETUP包长度
                {
                    SetupLen = UsbSetupBuf->wLengthL;
                    if ( UsbSetupBuf->wLengthH || SetupLen > 0x7F )
                    {
                        SetupLen = 0x7F;                                       // 限制总长度
                    }
                    len = 0;                                                   // 默认为成功并且上传0长度
                    if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )/* 类请求 */
                    {                                                                  
                        SetupReqCode = UsbSetupBuf->bRequest;
						if(SetupReqCode == 0xFE)   //GET MAX LUN
						{
							pDescr = (PUINT8)( &MAX_LUN[0] ); 							
							len = 1;
						    if ( SetupLen > len )
                            {
                                SetupLen = len;                                 // 限制总长度
                            }
                            len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen;  // 本次传输长度
                            memcpy( Ep0Buffer, pDescr, len );                   /* 加载上传数据 */
                            SetupLen -= len;
                            pDescr += len;
							
						}
						
                    }
                    else                                                       // 标准请求
                    {
                        SetupReqCode = UsbSetupBuf->bRequest;
                        switch( SetupReqCode )                                 // 请求码
                        {
                        case USB_GET_DESCRIPTOR:
                            switch( UsbSetupBuf->wValueH )
                            {
                            case 1:                                            // 设备描述符
                                pDescr = (PUINT8)( &MyDevDescr[0] );
                                len = sizeof( MyDevDescr );
                                break;
                            case 2:                                            // 配置描述符
                                pDescr = (PUINT8)( &MyCfgDescr[0] );
                                len = sizeof( MyCfgDescr );
                                break;
                            case 3:                                            // 字符串描述符
                                switch( UsbSetupBuf->wValueL )
                                {
                                case 1:
                                    pDescr = (PUINT8)( &MyManuInfo[0] );       
                                    len = sizeof( MyManuInfo );
                                    break;
                                case 2:
                                    pDescr = (PUINT8)( &MyProdInfo[0] );        
                                    len = sizeof( MyProdInfo );
                                    break;
                                case 0:
                                    pDescr = (PUINT8)( &MyLangDescr[0] );
                                    len = sizeof( MyLangDescr );
                                    break;
                                default:
                                    len = 0xFF;                                 // 不支持的字符串描述符
                                    break;
                                }
                                break;
                            default:
                                len = 0xFF;                                     // 不支持的描述符类型
                                break;
                            }
                            if ( SetupLen > len )
                            {
                                SetupLen = len;                                 // 限制总长度
                            }
                            len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen;  // 本次传输长度
                            memcpy( Ep0Buffer, pDescr, len );                   /* 加载上传数据 */
                            SetupLen -= len;
                            pDescr += len;
                            break;
                        case USB_SET_ADDRESS:
                            SetupLen = UsbSetupBuf->wValueL;                    // 暂存USB设备地址
                            break;
                        case USB_GET_CONFIGURATION:
                            Ep0Buffer[0] = UsbConfig;
                            if ( SetupLen >= 1 )
                            {
                                len = 1;
                            }
                            break;
                        case USB_SET_CONFIGURATION:
                            UsbConfig = UsbSetupBuf->wValueL;
						
							printf("Config\n");
                            if ( UsbConfig )
                            {
//                                LED_CFG = 0;
                            }
                            else
                            {
//                                LED_CFG = 1;
                            }
                            break;
                        case USB_CLEAR_FEATURE:
                            if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )// 端点
                            {
                                switch( UsbSetupBuf->wIndexL )
                                {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
									if(CH375CSW) mCH375UpCsw();
									lastFSTALL=FSTALL;
									FSTALL=0;
                                    break;
                                case 0x01:
                                    UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;						
									if(CH375CSW) mCH375UpCsw();
									lastFSTALL=FSTALL;
									FSTALL=0;
                                    break;
                                default:
                                    len = 0xFF;                                     // 不支持的端点
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF;                                         // 不是端点不支持
                            }
                            break;
                        case USB_GET_INTERFACE:
                            Ep0Buffer[0] = 0x00;
                            if ( SetupLen >= 1 )
                            {
                                len = 1;
                            }
                            break;
                        case USB_GET_STATUS:
                            Ep0Buffer[0] = 0x00;
                            Ep0Buffer[1] = 0x00;
                            if ( SetupLen >= 2 )
                            {
                                len = 2;
                            }
                            else
                            {
                                len = SetupLen;
                            }
                            break;
                        default:
                            len = 0xFF;                                             // 操作失败
                            printf("ErrEp0ReqCode=%02X\n",(UINT16)SetupReqCode);
                            break;
                        }
                    }
                }
                else
                {
                    len = 0xFF;                                                    // SETUP包长度错误
                    printf("ErrEp0ReqSize\n");
                }
                if ( len == 0xFF )                                                 // 操作失败
                {
                    SetupReqCode = 0xFF;
                    UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;// STALL
                }
                else if ( len <= THIS_ENDP0_SIZE )                                 // 上传数据或者状态阶段返回0长度包
                {
                    UEP0_T_LEN = len;
                    UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;// 默认数据包是DATA1
                }
                else                                                               // 下传数据或其它
                {
                    UEP0_T_LEN = 0;                                                // 虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
                    UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;// 默认数据包是DATA1
                }
                break;
            case UIS_TOKEN_IN | 0:                                                 // endpoint 0# IN
                switch( SetupReqCode )
                {
                case USB_GET_DESCRIPTOR:
                    len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; // 本次传输长度
                    memcpy( Ep0Buffer, pDescr, len );                               /* 加载上传数据 */
                    SetupLen -= len;
                    pDescr += len;
                    UEP0_T_LEN = len;
                    UEP0_CTRL ^= bUEP_T_TOG;                                        // 翻转
                    break;
                case USB_SET_ADDRESS:
                    USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                    break;
                default:
                    UEP0_T_LEN = 0;                                                 // 状态阶段完成中断或者是强制上传0长度数据包结束控制传输
                    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                    break;
                }
                break;
            case UIS_TOKEN_OUT | 0:                                                 // endpoint 0# OUT
                switch( SetupReqCode )
                {
//                      case download:
//                          if ( U_TOG_OK ) {                                       // 不同步的数据包将丢弃
//                              UEP0_CTRL ^= bUEP_R_TOG;                            // 翻转
//                              get_data;
//                              //UEP0_CTRL = UEP0_CTRL & bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;// 预置上传0长度数据包DATA1以防主机提前进入状态阶段
//                          }
//                          break;
                case USB_GET_DESCRIPTOR:
                default:
                    if ( U_TOG_OK )                                                 // 不同步的数据包将丢弃
                    {
//                              if ( USB_RX_LEN ) control_status_error;
//                              else control_ok;                                    // 收到0长度包表示控制读操作/上传OK
                    }
//                          else control_status_error;
                    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                      // 准备下一控制传输
                    break;
                }
                break;
            default:
                printf("ErrEndp INT\n");
                break;
            }
        }
        UIF_TRANSFER = 0;                                                           // 清中断标志
    }
    else if ( UIF_BUS_RST )                                                         // USB总线复位
    {
		printf("Reset\n");
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;                                                            // 清中断标志
    }
    else if ( UIF_SUSPEND )                                                         // USB总线挂起/唤醒完成
    {
        UIF_SUSPEND = 0;
        if ( USB_MIS_ST & bUMS_SUSPEND )                                            // 挂起
        {
            printf( "Suspend\n" );                                                        // 睡眠状态
            while ( XBUS_AUX & bUART0_TX );                                         // 等待发送完成
            SAFE_MOD = 0x55;
            SAFE_MOD = 0xAA;
            WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO;                                 // USB或者RXD0有信号时可被唤醒
            PCON |= PD;                                                             // 睡眠
            SAFE_MOD = 0x55;
            SAFE_MOD = 0xAA;
            WAKE_CTRL = 0x00;
        }
        else                                                                        // 唤醒
        {
            printf( "Awake\n" ); 
        }
    }
    else 
    {                                                                               // 意外的中断,不可能发生的情况
        printf("Unknown INT\n");
        USB_INT_FG = 0xFF;                                                          // 清中断标志
    }
	
}
