//Copyright (c) 1999-2012  WCH Corporation
//Module Name:DbgFunc.h
/*
Abstract:
    Function for outputing debug information 
Environment:
    user mode only
Notes:
  Copyright (c) 1999-2012 WCH Corporation.  All Rights Reserved.
Revision History:
  6/5/2012: L.s create
--*/

#ifndef		_DEBUGFUNC_H
#define		_DEBUGFUNC_H

VOID  DbgPrint (LPCTSTR lpFormat,...); //��ʾ��ʽ���ַ���
void ShowLastError(PCHAR FuncName); //��ʾ�ϴ����д���
ULONG mStrToBcd(PCHAR str); //��ʮ�������ַ�ת����ʮ������,����ת�����ַ���ltoa()����
double GetCurrentTimerVal(); //��ȡӲ��������������ʱ��,msΪ��λ,��GetTickCount��׼ȷ
VOID DelayTime1(double TimerVal); //��ʱֵΪms,�䶨ʱ���һ�㲻����0.5΢�룬������CPU�Ȼ��������й�
PVOID trim(PVOID str);  //ȥ���ַ����е�ǰ��ո�
VOID  DbgPrint (LPCTSTR lpFormat,...); //�����ʽ���ַ���,��dbgview�������
void ShowLastError(PCHAR FuncName); //��ʾ�ϴ����д���
double GetCurrentTimerVal(); //��ȡӲ��������������ʱ��,msΪ��λ,��GetTickCount��׼ȷ
//��ʱ����,��msΪ��λ
VOID DelayTime1(
				double TimerVal //��ʱֵΪms,�䶨ʱ���һ�㲻����0.5΢�룬������CPU�Ȼ��������й�
				);
//���������Ϣ���ļ���
BOOL LogToFile(PUCHAR Buffer,  //��ӡ��Ϣ
				 ULONG OLen,   //��ӡ��Ϣ����
				 BOOL IsChar);  //��ӡ��Ϣ���ַ���ʽ��ʾ��������ֵ��ʽ��ʾ
ULONG mStrToHEX(PCHAR str); //��ʮ�������ַ�ת����ʮ������,����ת�����ַ���ltoa()����
ULONG mStrToBcd(PCHAR str); //��ʮ�����ַ�ת����ʮ������,����ת�����ַ���ltoa()����
PVOID trim(PVOID str); //ȥ���ַ����е�ǰ��ո�
VOID  AddStrToEdit (HWND hDlg,ULONG EditID,const char * Format,...); //����ʽ���ַ���Ϣ������ı���ĩβ

#endif