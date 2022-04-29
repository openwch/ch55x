/*
Module Name:Main.cpp
*****************************************
**  Copyright  (C)  WCH  2001-2015     **
**  Web:  http://wch.cn                **
*****************************************
Environment:
    user mode only,VC6.0
Revision History:
    9/10/2015: TECH
Descriptor:
    WCH MCU IAP Windows����ʾ���������
	֧��CH56X��CH55X��Ƭ��
*/

#include "resource.h"
#include <stdio.h>
#include "Main.h"
#include "IAP.H"
#include "CH375DLL.H"
#pragma comment (lib,"CH375DLL")

//ȫ�ֱ���
HWND AfxMainHwnd; //��������
HINSTANCE AfxMainIns; //����ʵ��

DEFINE_GUID(GUID_DEVINTERFACE_COMPORT,              0x86e0d1e0L, 0x8089, 0x11d0, 0x9c, 0xe4, 0x08, 0x00, 0x3e, 0x30, 0x1f, 0x73);

#define mDnDev_MAX_NUMBER 16
DnDevInforS AfxDnDev[mDnDev_MAX_NUMBER] = {0}; //�豸�б�
UCHAR AfxDnDevCnt;
ULONG AfxDnInterface;
BOOL IsDownloading,IsDeviceChanged;

//��������
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);

//Ӧ�ó������
int APIENTRY WinMain(HINSTANCE hInstance,
                     HINSTANCE hPrevInstance,
                     LPSTR     lpCmdLine,
                     int       nCmdShow)
{
	AfxMainIns = hInstance;
	return 	DialogBox(hInstance, (LPCTSTR)IDD_IAPDEMO_DIALOG, 0, (DLGPROC)WndProc);
}


//���������豸�ӿڣ�����ʾ
ULONG ScanDnDeviceAndShow()
{
	ULONG i,DevID;
	CHAR FmtStr[128] = "";			
	HKEY	hKey;
	TCHAR	Name[32];
	UCHAR	szPortName[32];
	LONG	Status;
	DWORD	dwIndex = 0;	
	DWORD	dwName;
	DWORD	dwSizeofPortName;
	DWORD	Type;
	int		SerNumber;
	long	ret0;

	AfxDnDevCnt = 0;	
	if( AfxDnInterface == 0 ) //USB�ӿ�
	{
		for (i = 0;i < mDnDev_MAX_NUMBER;++i)
		{
			if( CH375OpenDevice(i) != INVALID_HANDLE_VALUE )//�ҵ�һ���豸
			{
				DevID = CH375GetUsbID(i);
				if( ( LOWORD(DevID) == 0x4348 ) || //���ص�USB�豸VID�ţ���MCU��Ӧ
					( HIWORD(DevID) == 0x55E0 ) )  //���ص�USB�豸PID�ţ���MCU��Ӧ
				{
					strcpy(&AfxDnDev[AfxDnDevCnt].DevName[0],(PCHAR)CH375GetDeviceName(i));
					if( strlen(AfxDnDev[AfxDnDevCnt].DevName) == 0 ) //��Ч�豸����
						continue;
					AfxDnDev[AfxDnDevCnt].iIndex = AfxDnDevCnt;
					AfxDnDevCnt++;
				}
				CH375CloseDevice(i);
			}
		}
		if( AfxDnDevCnt )
			DbgPrint("�Ѳ��ҵ�%d��USB�豸",AfxDnDevCnt);
		else
			DbgPrint("δ�ҵ�USB�豸������MCU�Ƿ�������IAP���룬�Լ�MCU�Ƿ��ѽ���IAPģʽ",AfxDnDevCnt);
	}
	else //ö�ٴ���
	{	
		
		LPCTSTR data_Set = _T("HARDWARE\\DEVICEMAP\\SERIALCOMM\\");
		
		ret0 = (::RegOpenKeyEx(HKEY_LOCAL_MACHINE,data_Set,0,KEY_READ,&hKey));		//��ע���
		if (ret0 != ERROR_SUCCESS)
		{
			return AfxDnDevCnt;
		}

		do
		{
			dwName = sizeof(Name)/sizeof(TCHAR);
			dwSizeofPortName = sizeof(szPortName);
			Status = RegEnumValue(hKey, dwIndex++, Name, &dwName, NULL, &Type,szPortName, &dwSizeofPortName);	//��ȡע���ֵ
			if (ERROR_NO_MORE_ITEMS == Status)
			{
				break;
			}
			//���ɹ���ã����ʾ�д��ڣ�����������������Ӧ�Ĵ��ں�
			if((atoi((LPCTSTR)(&szPortName[8])))==0)
				SerNumber = atoi((LPCTSTR)(&szPortName[6]));
			else
				SerNumber = atoi((LPCTSTR)(&szPortName[6]))*10 + atoi((LPCTSTR)(&szPortName[8]));
		
			strcpy(&AfxDnDev[AfxDnDevCnt].DevName[0],(char*)szPortName);
			AfxDnDev[AfxDnDevCnt].iIndex = AfxDnDevCnt;
			AfxDnDevCnt++;

		}while(1);
		RegCloseKey(hKey);	//�ر�ע���	

		DbgPrint("�Ѳ��ҵ�%d������",AfxDnDevCnt);
	}

	//����豸�б�����
	SendDlgItemMessage(AfxMainHwnd,IDC_DeviceList,CB_RESETCONTENT,0,0);
	for(i=0;i<AfxDnDevCnt;i++)
	{
		if( AfxDnInterface == 0 )
			sprintf(FmtStr,"%d���豸",i); //��ʾ���ƿ����Ѷ���
		else
			sprintf(FmtStr,"%s",AfxDnDev[i].DevName); //��ʾ���ƿ����Ѷ���
		SendDlgItemMessage(AfxMainHwnd,IDC_DeviceList,CB_ADDSTRING,0,(LPARAM)(LPCTSTR)FmtStr);
	}
	SendDlgItemMessage(AfxMainHwnd,IDC_DeviceList,CB_SETCURSEL,0,0);

	return AfxDnDevCnt;
}

// ����USB�豸�Ĳ��
VOID CALLBACK NotifyUsbDnDeviceRoutine(ULONG	iEventStatus)  // �豸�¼��͵�ǰ״̬(�����ж���): 0=�豸�γ��¼�, 3=�豸�����¼�
{
	if( AfxDnInterface != 0 ) //��USB��ʽ���أ����ø����豸�б�
		return;
	if( ( iEventStatus == CH375_DEVICE_ARRIVAL ) || ( iEventStatus == CH375_DEVICE_REMOVE) )//�豸����
	{
		if( IsDownloading ) //�������س�����ͣˢ���豸�б�
			IsDeviceChanged = TRUE; //������ɺ�ˢ���豸�б�
		else
		{
			IsDeviceChanged = FALSE;
			PostMessage(GetDlgItem(AfxMainHwnd,IDC_ScanDev),BM_CLICK,0,0); //ˢ���豸�б�
		}
	}
}

//���������
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent;
	AfxMainHwnd = hWnd;

	switch (message)
	{
	case WM_INITDIALOG:
		//��ʼ�����ط�ʽ
		SendDlgItemMessage(hWnd,IDC_DnInterface,CB_ADDSTRING,0,(LPARAM)(LPCTSTR)"USB");
		SendDlgItemMessage(hWnd,IDC_DnInterface,CB_ADDSTRING,0,(LPARAM)(LPCTSTR)"����");
		SendDlgItemMessage(hWnd,IDC_DnInterface,CB_SETCURSEL,0,0);
		AfxDnInterface = SendDlgItemMessage(hWnd,IDC_DnInterface,CB_GETCURSEL,0,0);

		CH375SetDeviceNotify(0,NULL,NotifyUsbDnDeviceRoutine);	//�����豸��μ���
		IsDownloading = IsDeviceChanged = FALSE;

		PostMessage(GetDlgItem(AfxMainHwnd,IDC_ScanDev),BM_CLICK,0,0); //ˢ���豸�б�
		break;
	case WM_COMMAND:
		wmId    = LOWORD(wParam); 
		wmEvent = HIWORD(wParam); 
		// Parse the menu selections:
		switch (wmId)
		{
		case IDC_DnInterface: 
			if( (wmEvent == CBN_SELCHANGE) ) //���½ӿ��豸�б�
			{
				//ˢ���豸�б�
				AfxDnInterface = SendDlgItemMessage(hWnd,IDC_DnInterface,CB_GETCURSEL,0,0);
				ScanDnDeviceAndShow(); //ˢ���豸�б�
			}
			break;
		case IDC_SelectFile: //ѡ�������ļ�
			
			{// ��ȡ��Ҫ���͵��ļ���		
				CHAR FmtStr[256] = "",FileName[512] = "";
				OPENFILENAME mOpenFile={0};					

				sprintf(FmtStr,"ѡ�������ļ�");			
				// Fill in the OPENFILENAME structure to support a template and hook.
				mOpenFile.lStructSize = sizeof(OPENFILENAME);
				mOpenFile.hwndOwner         = AfxMainHwnd;
				mOpenFile.hInstance         = AfxMainIns;		
				mOpenFile.lpstrFilter       = "*.HEX\0*.HEX\0*.BIN\0*.BIN\0";		
				mOpenFile.lpstrCustomFilter = NULL;
				mOpenFile.nMaxCustFilter    = 0;
				mOpenFile.nFilterIndex      = 0;
				mOpenFile.lpstrFile         = FileName;
				mOpenFile.nMaxFile          = sizeof(FileName);
				mOpenFile.lpstrFileTitle    = NULL;
				mOpenFile.nMaxFileTitle     = 0;
				mOpenFile.lpstrInitialDir   = NULL;
				mOpenFile.lpstrTitle        = FmtStr;
				
				mOpenFile.nFileOffset       = 0;
				mOpenFile.nFileExtension    = 0;
				mOpenFile.lpstrDefExt       = NULL;
				mOpenFile.lCustData         = 0;
				mOpenFile.lpfnHook 		   = NULL;
				mOpenFile.lpTemplateName    = NULL;
				mOpenFile.Flags             = OFN_SHOWHELP | OFN_EXPLORER | OFN_READONLY | OFN_FILEMUSTEXIST;
				if (!GetOpenFileName(&mOpenFile))
				{ 			
					DbgPrint("ѡ�����ļ�����");				
				}
				else
				{
					SetDlgItemText(AfxMainHwnd,IDC_DownloadFile,FileName); //���������ļ�
					SendDlgItemMessage(AfxMainHwnd,IDC_DownloadFile,EM_SETSEL,0xFFFFFFFE,0xFFFFFFFE);//��������Ƶ��ı����ĩβ
				}
			}
			break;
		case IDC_ScanDev: //ɨ���豸
			ScanDnDeviceAndShow();
			break;
		case IDC_ClearResult: //��ս����Ϣ
			SetDlgItemText(AfxMainHwnd,IDC_ResultShow,"");
			break;
		case IDC_Download: //�����ļ�
			DWORD ThreadID;

			if( SendDlgItemMessage(hWnd,IDC_DeviceList,CB_GETCURSEL,0,0) == CB_ERR )
			{
				MessageBox(AfxMainHwnd,"����ѡ�������豸���ߵ�ǰû���ҵ������豸","IAP DEMO",MB_ICONERROR);
				DbgPrint("����ѡ�������豸���ߵ�ǰû���ҵ������豸.");
				break;
			}
			{//�ж������ļ��Ƿ����
				CHAR FileName[MAX_PATH] = "";
				OFSTRUCT lpReOpenBuff = {0};

				if( GetDlgItemText(AfxMainHwnd,IDC_DownloadFile,FileName,sizeof(FileName)) < 1 )
				{
					MessageBox(AfxMainHwnd,"����ѡ�������ļ�","IAP DEMO",MB_ICONERROR);
					DbgPrint("����ʧ��!����ѡ�������ļ�");
					break;
				}				
				if ( OpenFile(FileName,&lpReOpenBuff,OF_EXIST) ==  HFILE_ERROR )
				{
					MessageBox(AfxMainHwnd,"�����ļ������ڻ�����ʹ��","IAP DEMO",MB_ICONERROR);
					DbgPrint("����ʧ��!�����ļ������ڻ�����ʹ��");
					break;
				}
			}
			CloseHandle(CreateThread(NULL,0,IAPFlashDownloadThread,NULL,0,&ThreadID)); //��ʼUSB����

			break;	
		case WM_DESTROY:
			CH375SetDeviceNotify(0,NULL,NULL); //ȡ���豸��μ��
			DestroyWindow(hWnd);
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		break;		
		case WM_DESTROY:
			PostQuitMessage(0);
			break;		
	}
	return 0;
}