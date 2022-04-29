/********************************** (C) COPYRIGHT ******************************
* File Name          :Compound_Dev.C											
* Author             : WCH                                                      
* Version            : V1.2                                                     
* Date               : 2017/02/24                                               
* Description        : A demo for USB compound device created by CH559, support 
					   keyboard and mouse, and HID compatible device.           
********************************************************************************/

#include	"Compound.h"
#include 	"debug.h"
#include 	"stdio.h"

extern UINT8 	FLAG;												// Trans complete flag
extern UINT8 	EnumOK;												// Enum ok flag

void main( void )
{
	mDelaymS(5);                                                    //
    mInitSTDIO( );                                                  // Init Stdio, support UART print out

#if	DEBUG
    printf( "Start config.\r\n" );
    printf( "Init USB device.\r\n" );
#endif
	USBDeviceCfg();                                                  // Configure the USB module
    USBDeviceEndPointCfg();                                          // Configure the Endpoint
    USBDeviceIntCfg();                                               // configure the USB Interrupt
	
	UEP1_T_LEN = 0;                                                  // Reset the trans length register
    UEP2_T_LEN = 0;                                                  //
    FLAG = 0;
	EnumOK = 0;
    while(1)
	{
        if( EnumOK == 1 )
        {
            HIDValueHandle();
        }

        mDelaymS( 100 );                                             
    }
}

/**************************** END *************************************/
