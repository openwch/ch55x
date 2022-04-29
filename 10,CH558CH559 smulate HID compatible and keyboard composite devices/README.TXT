/********************************** (C) COPYRIGHT ******************************
* File Name          : README.TXT													
* Author             : WCH                                                      
* Version            : V1.4                                                     
* Date               : 2017/02/24                                               
* Description        : 1. A demo for USB compound device created by CH559, support 
					   keyboard and mouse, and HID compatible device.   
					   2. The device will auto run the HID_RECV.exe, after plug on
						The path of the app locate at "E:\HID_RECV.EXE"
					   3. The device support interrupt transmit, interval set as 10ms
					   for keyboard and 50ms for comppsite device.
********************************************************************************/
µ÷ÊÔÈÕÖ¾
2017/02/22
1.	ÊµÏÖ°´¼üÓëÊó±ê¹¦ÄÜ
2.	Àí½âÔ¶³Ì»½ĞÑ£¿
3.	½â¾öÊó±êµ¥»÷Ê§Ğ§BUG
4.	´®¿Úµ÷ÊÔ£¬Ôö¼ÓWIN+RÏµÍ³ÈÈ¼ü

2017/02/23
1.	ÆóÍ¼Í¨¹ıµ¥Æ¬»ú²Ù×÷¼üÅÌÊı¾İ´ò¿ªÓ¦ÓÃ³ÌĞò
2.	ÊµÏÖWIN+RÈÈ¼ü´ò¿ªÄ³Â·¾¶ÎÄ¼şÖ®¹¦ÄÜ
3. 	½â¾ö´«Êı¾İ¶ª°üBUG(Ã¶¾ÙÍê³É±êÖ¾ SET REPORT...)

2017/02/24
1.	HID¼æÈİÉè???
2.	¼ÓÉîÀí½âUSBĞ­Òé£¬HIDÉè±¸ÃèÊö·û½â¾öÏà¹ØBUG

2017/02/28
1. 	½â¾öHID¼æÈİÉè±¸ÎÊÌâ

2017/03/05
1. Áªµ÷Í¨¹ı

2017/03/07
1. ½â¾ö¶Ëµã2ÂÒÂëÎÊÌâ(¿ª±Ù¶ÔÓ¦»º³åÇø[OUT]+[IN])
2. ½â¾öSET_IDLEÊ±µÈ´ıÎÊÌâ(¸øSTALL)

2017/03/09
1. ÏµÍ³²âÊÔ£ºÖ§³ÖWINXP/WIN7/WIN8/WIN10£¬²»Ö§³ÖLinuxÏµÍ³£¬²»Ö§³ÖĞéÄâ»ú

2017/03/10
1. ĞŞÕıÖĞ¶Ï´¦Àí(CLASS REQ)
2. Áªµ÷²âÊÔÉÛ¡¢³ÂÈí¼ş£
3. ½â¾öĞéÄâ»ú²âÊÔÎŞ·¨Ê¶±ğÎÊÌâ(ÉèÖÃUSBÉè±¸È«²¿¿É¼û£¬Èç¹û»¹ÊÇ²»ĞĞ£¬¼ÓÈë¡°usb.generic.autoconnect = "TRUE"¡±Óï¾ä½ø*.vmxÎÄ¼ş)
