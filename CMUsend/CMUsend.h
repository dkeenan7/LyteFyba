// CMUSend.h : main header file for the CMUSend application
//
#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"       // main symbols
#include "EnumSerial.h"
#include <afxadv.h>



// CCMUSendApp:
// See CMUSend.cpp for the implementation of this class
//

class CCMUSendApp : public CWinApp
{
public:
	CCMUSendApp();
	CRecentFileList* GetRecentFileList() {return m_pRecentFileList;}


// Overrides
private:
	virtual BOOL InitInstance();

// Implementation
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
	afx_msg void OnSend();
	afx_msg void OnFileOpen();

public:
			void	UpdateTitle();
			void	UpdateMenu();
			void	Adjust_start_and_len();	// Adjust the start offset and length to send
			CString GetRecentFile(int index) const;

	CArray<SSerInfo,SSerInfo&>
					m_asiPorts;				// CArray of available serial ports
	int				m_nPortIndex;			// Zero based index of the selected port

	TCHAR	m_szFileName[200];
	TCHAR	m_szShortName[100];
	BOOL	m_bFileValid;
	int		m_nProgress;
	bool m_bBadSum;

	unsigned int	m_total_len;			// Number of bytes read into the image
	unsigned int	m_len_to_send;			// Number of bytes to send (including checksum)
		// The checksum replaces the last byte of the image that would otherwise have been sent
	int m_start_off;						// Offset from the start of the image to start sending from
	UINT16			m_uResetVec;			// Last two bytes of the image, which should be the reset vector


};

// Menu indexes
#define MENU_IDX_PROGRAM_4K 1
#define MENU_IDX_PROGRAM_8K 2
#define MENU_IDX_ALL 3
#define MENU_IDX_BAD 4

// Passwords
#define PASSWORD_BSL2 2
#define PASSWORD_PROG_4K 4
#define PASSWORD_PROG_8K 8


extern CCMUSendApp theApp;