#ifndef		_MAIN_H
#define		_MAIN_H

// Windows Header Files:
#include <windows.h>

// C RunTime Header Files
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>

#include <windows.h>
#include <stdio.h>
#include <initguid.h>
#include <regstr.h>
#include <setupapi.h>
#pragma comment(lib,"setupapi")
#include "CH375DLL.H"
//#include <afxdlgs.h>
#pragma comment (lib,"CH375DLL")
#include "DbgFunc.h"


//�����豸��Ϣ�ṹ
typedef struct _DnDeviceInfor
{
	UCHAR iIndex;   //�豸���
	CHAR  DevName[128];  //�豸����,����ͨѶʹ��
	CHAR  DevDispName[128]; //���豸����������ʾ������
}DnDevInforS,*PDnDevInforS;

#endif		// MAIN_H