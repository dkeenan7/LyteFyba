// LyteFlashDoc.cpp : implementation of the CLyteFlashDoc class
// Modified 18?Apr/2017 for longer delay after password bytes
//

#include "stdafx.h"
#include "LyteFlash.h"
#include <MMSystem.h>				// For timeBeginPeriod / timeEndPeriod
#pragma  comment(lib, "winmm.lib")	// Needed for VS2005; doesn't seem to hurt others

#include "LyteFlashDoc.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CPortDlg dialog used for choosing a serial port

class CPortDlg : public CDialog
{
public:
	CPortDlg();

// Dialog Data
	enum { IDD = IDD_DLG_SELPORT };


protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
public:
	CListBox m_listPorts;
protected:
	virtual void OnOK();

	virtual BOOL OnInitDialog();

public:
	afx_msg void OnBnClickedRefresh();
};

CPortDlg::CPortDlg() : CDialog(CPortDlg::IDD)
{
}

BOOL CPortDlg::OnInitDialog() 
{
	CDialog::OnInitDialog();

	OnBnClickedRefresh();
	return TRUE;
}

void CPortDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_LIST_PORTS, m_listPorts);
}

void CPortDlg::OnOK()
{
	UpdateData();
	theApp.m_nPortIndex = m_listPorts.GetCurSel();
	theApp.UpdateTitle();
	CDialog::OnOK();
}

void CPortDlg::OnBnClickedRefresh()
{
	EnumSerialPorts(theApp.m_asiPorts, true);
	int n = (int)(theApp.m_asiPorts.GetCount());
	m_listPorts.ResetContent();
	for (int i=0; i < n; ++i)
		m_listPorts.AddString(theApp.m_asiPorts[i].strFriendlyName);
	m_listPorts.SetCurSel(theApp.m_nPortIndex);
}


BEGIN_MESSAGE_MAP(CPortDlg, CDialog)
	ON_BN_CLICKED(ID_REFRESH, &CPortDlg::OnBnClickedRefresh)
END_MESSAGE_MAP()

//	//	//	//	//	//	//	//	//	//	//
// CProgDlg dialog used Send Progress	//
//	//	//	//	//	//	//	//	//	//	//

class CProgDlg : public CDialog
{
public:
	CProgDlg();

// Dialog Data
	enum { IDD = IDD_PROGRESS };


protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
public:
	CProgressCtrl m_Bar;
protected:

	virtual BOOL OnInitDialog();

public:
	afx_msg void PostNcDestroy();
public:
	afx_msg void OnSetFocus(CWnd* pOldWnd);
};

CProgDlg::CProgDlg() : CDialog(CProgDlg::IDD)
{
}

BOOL CProgDlg::OnInitDialog() 
{
	CDialog::OnInitDialog();

	//m_Bar.SetRange(0, 2048);
	return TRUE;
}

void CProgDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_PROGRESS, m_Bar);
}

BEGIN_MESSAGE_MAP(CProgDlg, CDialog)
	ON_WM_SETFOCUS()
END_MESSAGE_MAP()




// CLyteFlashDoc

IMPLEMENT_DYNCREATE(CLyteFlashDoc, CDocument)

BEGIN_MESSAGE_MAP(CLyteFlashDoc, CDocument)
	ON_COMMAND(ID_FILE_OPEN, &CLyteFlashDoc::OnFileOpen)
	ON_UPDATE_COMMAND_UI(ID_SEND, &CLyteFlashDoc::OnUpdateSend)
	ON_COMMAND_RANGE(ID_FILE_MRU_FILE1, ID_FILE_MRU_FILE16, &CLyteFlashDoc::OnFileMruFile)
	ON_COMMAND(ID_SEND, &CLyteFlashDoc::OnSend)
	ON_COMMAND(ID_SETSERIAL, &CLyteFlashDoc::OnSetserial)
	ON_COMMAND(ID_FILE_SAVEAS, &CLyteFlashDoc::OnFileSaveas)
	ON_COMMAND(ID_PASSWORD_TRUNK, &CLyteFlashDoc::OnPasswordTrunk)
	ON_COMMAND(ID_PASSWORD_REV61, &CLyteFlashDoc::OnPasswordRev61)

END_MESSAGE_MAP()


// CLyteFlashDoc construction/destruction

CLyteFlashDoc::CLyteFlashDoc()
{
}

CLyteFlashDoc::~CLyteFlashDoc()
{
}

BOOL CLyteFlashDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: add reinitialization code here
	// (SDI documents will reuse this document)

	return TRUE;
}




// CLyteFlashDoc serialization

void CLyteFlashDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: add storing code here
	}
	else
	{
		// TODO: add loading code here
	}
}


// CLyteFlashDoc diagnostics

#ifdef _DEBUG
void CLyteFlashDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CLyteFlashDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG


// CLyteFlashDoc commands

void CLyteFlashDoc::OnFileOpen()
{
	OPENFILENAME ofn;
	ZeroMemory( &ofn , sizeof( ofn));
	ofn.lStructSize = sizeof ( ofn );
	ofn.lpstrFilter = _T("Binary\0*.bin\0Hex\0*.hex\0All\0*.*");
	ofn.lpstrFile = theApp.m_szFileName;
	ofn.nMaxFile = sizeof( theApp.m_szFileName );
	theApp.m_szFileName[0] = '\0';
	ofn.lpstrFileTitle = theApp.m_szShortName;
	ofn.nMaxFileTitle = sizeof( theApp.m_szShortName );
	ofn.lpstrInitialDir = NULL ;
	ofn.hwndOwner = NULL ;
	ofn.nFilterIndex =1;		// First filter, i.e. hex
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST ;
 
	if (GetOpenFileName(&ofn))
	{
		theApp.AddToRecentFileList( theApp.m_szFileName );
		ReadFile();
	}
}


void CLyteFlashDoc::OnUpdateSend(CCmdUI *pCmdUI)
{
	// TODO: Add your command update UI handler code here
	pCmdUI->Enable(theApp.m_bFileValid);
}

void CLyteFlashDoc::OnFileMruFile(UINT uiMsgId)
{
	// TODO: Add your command handler code here
	LPCTSTR pPathlessName = _tcsrchr((*theApp.GetRecentFileList())[uiMsgId - ID_FILE_MRU_FILE1], '\\');
	pPathlessName++;			// Skip the \\ (possibly wide) character
	_tcscpy_s(theApp.m_szShortName, pPathlessName);
	UpdateAllViews(NULL);
	_tcscpy_s(theApp.m_szFileName, (*theApp.GetRecentFileList())[uiMsgId - ID_FILE_MRU_FILE1]);
	ReadFile();
}

// Disgusting globals
static unsigned char sum;
static unsigned int add;
HANDLE hComm;


static bool readColon(CFile& f) {
	char ch;
	do {
		if (f.Read(&ch, 1) == 0)
			return false;
	} while (ch != ':');
	return true;
}

static unsigned int readHexNibble(CFile& f) {
    char c;
	f.Read(&c, 1);
    int r;
    if (c >= '0') {
        r = c - '0';
        if (r > 9)
            r -= 'A'-1-'9';
        if (r <= 0xF)
            return r;
    }

    TCHAR msg[32];
	_stprintf_s(msg, sizeof(msg), _T("Unexpected char '%c' when reading hex, last address = %X"), c, add);
	MessageBox(theApp.m_pMainWnd->m_hWnd, msg, _T("Error"), MB_ICONSTOP);
    exit(1);
    return -1;
}

static unsigned int readHexByte(CFile& f) {
    unsigned int r = (readHexNibble(f) << 4) + readHexNibble(f);
    sum += r;
    return r;
}

static unsigned int readHexWord(CFile& f) {
    return (readHexByte(f) << 8) + readHexByte(f);
}

void CLyteFlashDoc::ReadFile()
{
	theApp.UpdateTitle();
	CFile f(theApp.m_szFileName, CFile::modeRead | CFile::typeBinary);
	// Now only support loading a binary file
	theApp.m_total_len = (unsigned int) f.GetLength();
	m_pFileBuf = new unsigned char[theApp.m_total_len];
	if (m_pFileBuf == NULL)
	{
		theApp.m_pMainWnd->MessageBox(_T("Error allocating file buffer"), _T("File buffer error"),
			MB_ICONSTOP);
		return;
	}
	f.Read(m_pFileBuf, theApp.m_total_len);
	f.Close();
	if (theApp.m_total_len)
		theApp.m_bFileValid = true;		// Enables the ID_SEND green arrow

	// Read the reset vector; needed to determine correct length
	theApp.m_uResetVec = *(UINT16*)(m_pFileBuf + theApp.m_total_len - 2);
	theApp.Adjust_start_and_len();
}


static void writeByte(unsigned char* p) {
	OVERLAPPED osWrite = {0};

	// Create this write operation's OVERLAPPED structure's hEvent.
	osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (osWrite.hEvent == NULL) {
		theApp.m_pMainWnd->MessageBox(_T("Error creating overlapped event handle"), _T("Internal comms error"),
			MB_ICONSTOP);
		return;
	}

	// Issue write.
	DWORD dwWritten;
	if (!WriteFile(hComm, p, 1, /*NULL*/&dwWritten, &osWrite)) {
		if (GetLastError() != ERROR_IO_PENDING) {
		theApp.m_pMainWnd->MessageBox(_T("WriteFile failed, but isn't delayed."), _T("Internal comms error"),
			MB_ICONSTOP);
			return;
		}
	}
	CloseHandle(osWrite.hEvent);
}

void CLyteFlashDoc::OnSend()
{
	ReadFile();					// In case sending the same file with changes
	// Open comm port
	if (theApp.m_asiPorts.GetCount() == 0)
	{
		theApp.m_pMainWnd->MessageBox(_T("There were no communications ports found"), _T("Comms error"), MB_ICONSTOP);
		return;
	}

	CString& csFriendlyName = theApp.m_asiPorts[theApp.m_nPortIndex].strFriendlyName;
	int idxLparen = csFriendlyName.ReverseFind(_T('('));
	if ((idxLparen != -1) &&
		csFriendlyName[idxLparen+1] == 'C' && 
		csFriendlyName[idxLparen+2] == 'O' && 
		csFriendlyName[idxLparen+3] == 'M')
	{
		int idxRparen = csFriendlyName.ReverseFind(_T(')'));
		CString csNumber(csFriendlyName.Mid(idxLparen+4, idxRparen - (idxLparen+4)));
		int comPortNumber = _tstoi(csNumber.GetBuffer());
		TCHAR sName[32];
		COMMCONFIG  lpCC;
		_stprintf_s(sName, sizeof(sName)/sizeof(TCHAR), _T("\\\\.\\COM%d"), comPortNumber);
		hComm = CreateFile(sName,
			GENERIC_WRITE,
			0,
			0,
			OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL,
			0);
		if (hComm == INVALID_HANDLE_VALUE) {
			TCHAR szMsg[64];
			_stprintf_s(szMsg, sizeof(szMsg)/sizeof(TCHAR), _T("Error opening port COM%d\nMay be in use by another application"),
				comPortNumber);
			theApp.m_pMainWnd->MessageBox(szMsg, _T("Comms error"), MB_ICONSTOP);
			return;
		}
		GetCommState( hComm, &lpCC.dcb);

		 /* Initialisation of parameters */
		lpCC.dcb.BaudRate = CBR_9600;
		//lpCC.dcb.BaudRate = CBR_19200;
		lpCC.dcb.ByteSize = 8;
		lpCC.dcb.StopBits = ONESTOPBIT;
		lpCC.dcb.Parity = NOPARITY;
		lpCC.dcb.fDtrControl = DTR_CONTROL_ENABLE;		// Raise DTR when connected
		lpCC.dcb.fRtsControl = RTS_CONTROL_DISABLE;
		SetCommState(hComm, &lpCC.dcb );
	}
	else {
		theApp.m_pMainWnd->MessageBox(CString(_T("Could not parse comm port name from \"")) + csFriendlyName, _T("Parse error"),
			MB_ICONSTOP);
		return;
	}

	// We want calls to Sleep() to be able to be as fast as 3 ms. NOTE: this changes all of Windows, causing all tasks to potentially
	// swap every 1 ms (it's faster for LyteFlash, don't ask why), so make sure you call timeEndPeriod(2) before exiting this function!
	timeBeginPeriod(1);

	CProgDlg* pProg = new CProgDlg();
	pProg->Create(CProgDlg::IDD);
	pProg->ShowWindow(SW_SHOW);
	pProg->m_Bar.SetRange(0, theApp.m_len_to_send);


    /* Now send this image to the BMUs */
    unsigned int i, u;

    /* Write the prefix (escape and the 4-character password) */
	unsigned char pfx[5];
	memcpy(pfx, "\x1B\x05\x04\x03\x02", 5);
	CMenu* pMenu = theApp.m_pMainWnd->GetMenu();
	if (pMenu->GetMenuState(ID_PASSWORD_TRUNK, MF_BYCOMMAND)) {
		pfx[5 - 1] = 0x01;				// Use trunk password sequence 05 04 03 01
	}
    for (i=0; i < 5; ++i) {
        writeByte(pfx+i);
//		Sleep(1+1+1);					// Delay for send, echo, and "rounding"/safety
		Sleep(270);						// Delay for send, BMU echo, and up to 255 CMUs
    }

	// Extra 2 second delay in case it's monolith, and it is busy sending data to the PIP inverter
	Sleep(2000);

    /* Allow time for segment erases (approximately 15 ms per segment) */
	/* Note that m_len_to_send is sometimes 2 short of the real length, because of the reset vector */
	/* Be conservative and use 21 ms per segment erase */
	Sleep((((theApp.m_len_to_send + 2) / 512) * 21) +1);
    // Send the appropriate number of bytes of the binary image (excluding last byte, to be replaced with the checksum)
	sum = 0;							// Checksum
    for (i=0, u = theApp.m_start_off; i < theApp.m_len_to_send-1; ++i, ++u) {
        writeByte(m_pFileBuf+u);        // Write one byte
		sum ^= m_pFileBuf[u];			// Update checksum
        Sleep(2+1);						// Time to transmit, echo, and flash write (~ 0.2 ms); the 0.2 can be
										//	part of the +1, which is because each bit is more like 1.04 ms, and
										//	in case the clock is slow
		if ((u & 0x3F) == 0x3F) {
			theApp.m_nProgress = u;
			pProg->SetFocus();			// Use the focus message to update the progress bar
		}
    }
	// Finally send the checksum byte in place of the very last byte, just before the reset vector
	// Sometimes we want to deliberately send a bad checksum
	if (theApp.m_bBadSum)
	{
		sum ^= 0xFF;					// Invert checksum
		writeByte(&sum);
		sum ^= 0xFF;					// Correct checksum by re-inverting
	}
	else
		writeByte(&sum);

	theApp.m_nProgress = theApp.m_len_to_send;
	pProg->SetFocus();

	timeEndPeriod(1);					// See comments before TimeBeginPeriod()

	CloseHandle(hComm);

}



void CLyteFlashDoc::OnSetserial()
{
	CPortDlg portDlg;
	portDlg.DoModal();
}


void CProgDlg::PostNcDestroy()
{
	CDialog::PostNcDestroy();
	delete this;

}

void CProgDlg::OnSetFocus(CWnd* pOldWnd)
{
	m_Bar.SetPos(theApp.m_nProgress);
	CDialog::OnSetFocus(pOldWnd);
	ShowWindow(SW_SHOW);
}

void CLyteFlashDoc::OnFileSaveas()
{
	OPENFILENAME ofn;
	ZeroMemory( &ofn , sizeof( ofn));
	ofn.lStructSize = sizeof ( ofn );
	ofn.lpstrFilter = _T("Bin\0*.bin\0All\0*.*");
	ofn.lpstrFile = theApp.m_szFileName;
	ofn.nMaxFile = sizeof(theApp.m_szFileName) / sizeof(TCHAR);
	theApp.m_szFileName[0] = '\0';
	ofn.lpstrFileTitle = theApp.m_szShortName;
	ofn.nMaxFileTitle = sizeof(theApp.m_szShortName) / sizeof(TCHAR);
	ofn.lpstrInitialDir = NULL ;
	ofn.hwndOwner = NULL ;
	ofn.nFilterIndex =1;		// First filter, i.e. hex
	ofn.Flags = OFN_OVERWRITEPROMPT | OFN_SHOWHELP; //OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST ;
 
	if (GetSaveFileName(&ofn))
	{
		CFile f(ofn.lpstrFile, CFile::modeCreate | CFile::modeWrite | CFile::typeBinary);
		f.Write(m_pFileBuf, theApp.m_total_len);
		f.Close();
	}
	
}

void CLyteFlashDoc::OnPasswordTrunk()
{
	CMenu* pMenu = theApp.m_pMainWnd->GetMenu();
	pMenu->CheckMenuItem(ID_PASSWORD_TRUNK, MF_CHECKED | MF_BYCOMMAND);
	pMenu->CheckMenuItem(ID_PASSWORD_REV61, MF_UNCHECKED | MF_BYCOMMAND);

}

void CLyteFlashDoc::OnPasswordRev61()
{
	CMenu* pMenu = theApp.m_pMainWnd->GetMenu();
	pMenu->CheckMenuItem(ID_PASSWORD_REV61, MF_CHECKED | MF_BYCOMMAND);
	pMenu->CheckMenuItem(ID_PASSWORD_TRUNK, MF_UNCHECKED | MF_BYCOMMAND);
}
