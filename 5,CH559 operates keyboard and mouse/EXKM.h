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


// ���ʼ������
void KM_LibInit( void );
// �������ú�������Ҫ��ѯ�豸���ӣ��豸�Ͽ���
u8	 KM_ProcessSystem( );
// ���ڼ��̵�Ʋ�����
void KM_HostSetReport( u8 report );
// ���ڻ�ȡ��ֵ����
// index - �˿������ţ�0/1��
// GetDatBuf - ���ڴ�ż�ֵ�Ļ�����
// *len - ��ȡ��ֵ�����ݳ��ȣ�����Ϊ8�����Ϊ4
u8   KM_HostGetData( u8 index,u8 *GetDatBuf, u8 *len ,u8 *SourceDatBuf,u8 *SourceLen);
// ��ѯ�˿��豸����
// index - �˿������ţ�0/1��
// GetDatBuf - ���ڴ�����͵Ļ����� �ӿ�(B)+����(B)
// *len - ��ȡ�����ݳ���
void KM_TypeQuery( u8 RootHubIndex,u8 *GetDatBuf,u8 *len );