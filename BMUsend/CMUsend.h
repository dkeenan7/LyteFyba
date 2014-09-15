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
private:
	virtual BOOL InitInstance();

// Implementation
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
	afx_msg void OnSend();
	afx_msg void OnImageAll1();
	afx_msg void OnImageBSL2();
	afx_msg void OnImageProgram();
	afx_msg void OnImageBadSum();
	afx_msg void OnFileOpen();

public:
			void UpdateTitle();
			void UpdateMenu();
			void Adjust_start_and_len();	// Adjust the start offset and length to send

	CArray<SSerInfo,SSerInfo&>
					m_asiPorts;				// CArray of available serial ports
	int				m_nPortIndex;			// Zero based index of the selected port

	TCHAR	m_szFileName[200];
	TCHAR	m_szShortName[100];
	BOOL	m_bFileValid;
	int		m_nProgress;
	// 1 for BSL1, 2 for BSL2
	int m_password_sel;
	// 1 for main program (all but last 512 bytes), 2 for BSL2 (last 512 bytes), 3 for all
	int m_image_sel;
	bool m_bBadSum;

	unsigned int	m_total_len;			// Number of bytes read into the image
	unsigned int	m_len_to_send;			// Number of bytes to send (including checksum)
		// The checksum replaces the last byte of the image that would otherwise have been sent
	int m_start_off;						// Offset from the start of the image to start sending from


};

extern CBMUsendApp theApp;