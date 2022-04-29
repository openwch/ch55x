/* Debug function */
/* Support printf function and delay function */

#ifndef	__DEBUG_H__
#define __DEBUG_H__

#define 	NO_XSFR_DEFINE
#include 	"CH559.H"

// Define function return
#ifndef  	SUCCESS
#define  	SUCCESS  	1
#endif	
#ifndef  	FAIL
#define  	FAIL    	0
#endif

// Define Timer start bit
#ifndef  	START
#define  	START  		1
#endif	
#ifndef  	STOP
#define  	STOP    	0
#endif

#define		DEBUG		1

#define		readKey()		CH559UART0RcvByte()

extern	void	CfgFsys();                        	// Config system clock
extern	void	mDelayuS( UINT16 n );              	// Delay n us
extern	void	mDelaymS( UINT16 n );              	// Delay n ms
extern	void  	CH559UART0Alter();                 	// Serial port remap P0.2/P0.3
extern	void	mInitSTDIO( );                      // Baud generate T1
extern	UINT8 	CH559UART0RcvByte();              	// CH559 UART0 receive a byte
extern	void  	CH559UART0SendByte(UINT8 SendDat); 	// CH559 UART0 send a byte

#endif
/**************************** END *************************************/
