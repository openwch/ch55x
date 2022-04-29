/********************************** (C) COPYRIGHT *******************************
* File Name          : DEBUG.C
* Author             : WCH
* Version            : V1.3
* Date               : 2016/06/24
* Description        : CH559 DEBUG Interface
					   Support serial port printf function, baud can be changed             				   
*******************************************************************************/
#define		NO_XSFR_DEFINE
#include 	<stdio.h>
#include 	<string.h>
#include	"DEBUG.H"

#define	 	FREQ_SYS	12000000	                                                   // System clock 12MHz
#ifndef  	BUAD
#define  	BUAD    57600
#endif

/*******************************************************************************
* Function Name  : CfgFsys( )
* Description    : Internal craystal 12MHz was set default, if define FREQ_SYS,Fsys can be configed as below: 
                   Fsys = (Fosc * ( PLL_CFG & MASK_PLL_MULT ))/(CLOCK_CFG & MASK_SYS_CK_DIV);
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/ 
void	CfgFsys( )  
{
    SAFE_MOD = 0x55;                                                         		 	// Safe mode
    SAFE_MOD = 0xAA;                                                 			
//  CLOCK_CFG |= bOSC_EN_XT;                                                 		 	// Enable External craystal
//  CLOCK_CFG &= ~bOSC_EN_INT;                                              			
// 	CLOCK_CFG &= ~MASK_SYS_CK_DIV;			
//  CLOCK_CFG |= 6;                                                          		 	// Fsys = 48MHz
//  CLOCK_CFG |= 8;                                                          		 	// Fsys = 36MHz
//  CLOCK_CFG |= 10;                                                         		 	// Fsys = 28.8MHz
//  CLOCK_CFG |= 12;                                                         		 	// Fsys = 24MHz
//  CLOCK_CFG |= 16;                                                         		 	// Fsys = 18MHz  	
    SAFE_MOD = 0xFF;                                                         		 	// Disable safe mode
//  Modify the FREQ_SYS, if system clock has been changed
}

/*******************************************************************************
* Function Name  : mDelayus(UNIT16 n)
* Description    : Delay n us
* Input          : UNIT16 n
* Output         : None
* Return         : None
*******************************************************************************/ 
void	mDelayuS( UINT16 n )  
{
	while ( n ) {  																		// total = 12~13 Fsys cycles, 1uS @Fsys=12MHz
		++ SAFE_MOD;  																	// 2 Fsys cycles, for higher Fsys, add operation here
#ifdef	FREQ_SYS
#if		FREQ_SYS >= 14000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 16000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 18000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 20000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 22000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 24000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 26000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 28000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 30000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 32000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 34000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 36000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 38000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 40000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 42000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 44000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 46000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 48000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 50000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 52000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 54000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 56000000
		++ SAFE_MOD;
#endif
#endif
		-- n;
	}
}

/*******************************************************************************
* Function Name  : mDelayms(UNIT16 n)
* Description    : Delay n ms
* Input          : UNIT16 n
* Output         : None
* Return         : None
*******************************************************************************/
void	mDelaymS( UINT16 n )                                                 	 		
{
	while ( n ) 
	{
		mDelayuS( 1000 );
		-- n;
	}
}                                         
#if 0
/*******************************************************************************
* Function Name  : CH559UART0Alter()
* Description    : CH559 remap the serial port, P0.2 & P0.3
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void	CH559UART0Alter()
{
    PORT_CFG |= bP0_OC;
    P0_DIR |= bTXD_;
    P0_PU |= bTXD_ | bRXD_;
    PIN_FUNC |= bUART0_PIN_X;                                                  			//´®¿ÚÓ³Éäµ½P0.2ºÍP0.3
}
#endif
/*******************************************************************************
* Function Name  : mInitSTDIO()
* Description    : Init serial port, T1 set as baud generator
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void	mInitSTDIO( )
{
    UINT32 x;
    UINT8 x2; 

    SM0 = 0;
    SM1 = 1;
    SM2 = 0;                                                                   			// Mode 1
																						// Timer 1 generate baud
    RCLK = 0;                                                                  			// RX clk
    TCLK = 0;                                                                  			// TX clk
    PCON |= SMOD;			
    x = 10 * FREQ_SYS / BUAD / 16;                                             			// If system clock changed, please check whether x overflow
    x2 = x % 10;			
    x /= 10;			
    if ( x2 >= 5 ) x ++;                                                       			// Data round
			
    TMOD = TMOD & ~ bT1_GATE & ~ bT1_CT & ~ MASK_T1_MOD | bT1_M1;              			// 0X20£¬Timer1 auto reload
    T2MOD = T2MOD | bTMR_CLK | bT1_CLK;                                        			// Select Timer1 clock
    TH1 = 0-x;                                                                 			// Set baud count
    TR1 = 1;                                                                   			// Start Timer1
    TI = 1;			
    REN = 1;                                                                   			// Enable COM0
}

/*******************************************************************************
* Function Name  : CH559UART0RcvByte()
* Description    : CH559UART0 receice a byte
* Input          : None
* Output         : None
* Return         : SBUF
*******************************************************************************/
UINT8	CH559UART0RcvByte( )
{
    while(RI == 0);                                                            			// Inquiry receice mode
    RI = 0;
    return SBUF;
}
#if 0
/*******************************************************************************
* Function Name  : CH559UART0SendByte(UINT8 SendDat)
* Description    : CH559UART0 send a byte
* Input          : UINT8 SendDat
* Output         : None
* Return         : None
*******************************************************************************/
void	CH559UART0SendByte(UINT8 SendDat)
{
	SBUF = SendDat;                                                              		// Inquiry send mode
	while(TI ==0);
	TI = 1;
}
#endif
/**************************** END *************************************/
