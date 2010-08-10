// BMUsend.h : main header file for the BMUsend application
//
#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"       // main symbols
#include "EnumSerial.h"
#include <afxadv.h>



// CBMUsendApp:
// See BMUsend.cpp for the implementation of this class
//

class CBMUsendApp : public CWinApp
{
public:
	CBMUsendApp();
	CRecentFileList* GetRecentFileList() {return m_pRecentFileList;}


// Overrides
public:
	virtual BOOL InitInstance();
			void UpdateTitle();

// Implementation
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnSend();
public:
	afx_msg void OnFileOpen();
public:
	afx_msg void OnUpdateSend(CCmdUI *pCmdUI);

	CArray<SSerInfo,SSerInfo&>
					m_asiPorts;				// CArray of available serial ports
	int				m_nPortIndex;			// Zero based index of the selected port

	TCHAR	m_szFileName[200];
	TCHAR	m_szShortName[100];
	BOOL	m_bFileValid;
	int		m_nProgress;


};

extern CBMUsendApp theApp;