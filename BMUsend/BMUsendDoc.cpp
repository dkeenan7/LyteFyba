// BMUsendDoc.cpp : implementation of the CBMUsendDoc class
//

#include "stdafx.h"
#include "BMUsend.h"

#include "BMUsendDoc.h"

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

	m_Bar.SetRange(0, 2048);
m_Bar.SetPos(75);
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




// CBMUsendDoc

IMPLEMENT_DYNCREATE(CBMUsendDoc, CDocument)

BEGIN_MESSAGE_MAP(CBMUsendDoc, CDocument)
	ON_COMMAND(ID_FILE_OPEN, &CBMUsendDoc::OnFileOpen)
	ON_UPDATE_COMMAND_UI(ID_SEND, &CBMUsendDoc::OnUpdateSend)
	ON_COMMAND_RANGE(ID_FILE_MRU_FILE1, ID_FILE_MRU_FILE16, &CBMUsendDoc::OnFileMruFile)
	ON_COMMAND(ID_SEND, &CBMUsendDoc::OnSend)
	ON_COMMAND(ID_SETSERIAL, &CBMUsendDoc::OnSetserial)
	ON_COMMAND(ID_FILE_SAVEAS, &CBMUsendDoc::OnFileSaveas)
END_MESSAGE_MAP()


// CBMUsendDoc construction/destruction

CBMUsendDoc::CBMUsendDoc()
{
	// TODO: add one-time construction code here
	m_total_len = 0;
}

CBMUsendDoc::~CBMUsendDoc()
{
}

BOOL CBMUsendDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: add reinitialization code here
	// (SDI documents will reuse this document)

	return TRUE;
}




// CBMUsendDoc serialization

void CBMUsendDoc::Serialize(CArchive& ar)
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


// CBMUsendDoc diagnostics

#ifdef _DEBUG
void CBMUsendDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CBMUsendDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG


// CBMUsendDoc commands

void CBMUsendDoc::OnFileOpen()
{
	OPENFILENAME ofn;
	ZeroMemory( &ofn , sizeof( ofn));
	ofn.lStructSize = sizeof ( ofn );
	ofn.lpstrFilter = _T("Hex\0*.hex\0All\0*.*");
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


void CBMUsendDoc::OnUpdateSend(CCmdUI *pCmdUI)
{
	// TODO: Add your command update UI handler code here
	pCmdUI->Enable(theApp.m_bFileValid);
}

void CBMUsendDoc::OnFileMruFile(UINT uiMsgId)
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
static unsigned int sum, add;
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

void CBMUsendDoc::ReadFile()
{
	theApp.UpdateTitle();
	CFile f(theApp.m_szFileName, CFile::modeRead);
	unsigned int len, typ, checksum;
	theApp.m_bFileValid = false;
	m_total_len = 0;
	unsigned int u;

	memset(m_HexBuf, '\xFF', 2048);

    do {
        if (!readColon(f))
			break;
        sum = 0;
        len = readHexByte(f);
		m_total_len += len;
        add = readHexWord(f);
        typ = readHexByte(f);
        if (typ == 1)
            break;
		if (typ > 0) {
			TCHAR msg[40];
			_stprintf_s(msg, _T("Unexpected record type %X at address %X"), typ, add);
			MessageBox(theApp.m_pMainWnd->m_hWnd, msg, _T("Error"), MB_ICONSTOP);
		    f.Close();
			return;
        }
        if (add < 0xF800)
            continue;           /* Repeat the loop, looking for the colon on the next line */
        unsigned char* p = m_HexBuf + add - 0xF800;
        for (u=0; u < len; ++u) {
            *p++ = readHexByte(f);
        }
        checksum = readHexByte(f);
        if (sum & 0xFF) {
			TCHAR msg[40];
			_stprintf_s(msg, _T("Bad checksum %X expected %X"), checksum, 0-(sum-checksum) & 0xFF);
			MessageBox(theApp.m_pMainWnd->m_hWnd, msg, _T("Error"), MB_ICONSTOP);
		    f.Close();
            return;
        }
        add += len;
    } while (1);

	// printf("Read %d bytes\n", total_len);
    f.Close();
	if (m_total_len)
		theApp.m_bFileValid = true;

    /* Calculate the checksum, and place at 0xFFFD (first unused interrupt vector, starting at highest address,
        after reset */
    sum = 0;
    for (u=0; u < 2048-2; ++u)          /* -2 because reset vector (last 2 bytes) is not sent */
        sum ^= m_HexBuf[u];
    sum ^= m_HexBuf[0xFFFD-0xF800];      /* Remove the existing checksum */
    m_HexBuf[0xFFFD-0xF800] = sum;       /* Now it will checksum to zero */

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

void CBMUsendDoc::OnSend()
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

	CProgDlg* pProg = new CProgDlg();
	pProg->Create(CProgDlg::IDD);
	pProg->ShowWindow(SW_SHOW);

    /* Now send this image to the BMUs */
    unsigned int i, u;

    /* Write the prefix */
#define PASSWORD4 1							// Non zero for the 4 byte password ^C ^B ^A ^@
#if PASSWORD4
#define PASSLEN (2+4)
	unsigned char* pfx = (unsigned char*) "\x01\x01\x03\x02\x01\x00";	/* ^a^a ^C ^B ^A ^@ */
#else
#define PASSLEN (2+3)
    unsigned char* pfx = (unsigned char*) "\x01\x01\x02\x01\x04";		/* ^a^a ^B ^A ^D */
#endif
    for (i=0; i < PASSLEN; ++i) {
        writeByte(pfx+i);                   /* Write 2nd to 5th byte of prefix */
		Sleep(2+1);
    }

    /* Allow time for bulk erase (approximately 32 ms) as well as a send, echo, and flash write */
	/* NOTE: There are two delays now, since we have two versions of the BSL at present.
		Soon the second delay can go away */
	Sleep(35+1);
    /* Send the 2048-2 bytes of the binary image */
    writeByte(m_HexBuf);                     /* Write first byte */
    /* Allow time for bulk erase (approximately 32 ms) as well as a send, echo, and flash write */
	Sleep(35+1);
    for (u=1; u < 2048-2; ++u) {
        writeByte(m_HexBuf+u);               /* Write byte */
        Sleep(3+1);        /* Time to transmit, echo, and flash write */
		if ((u & 0x3F) == 0x3F) {
			theApp.m_nProgress = u;
			pProg->SetFocus();			// Use the focus message to update the progress bar
		}
    }
	theApp.m_nProgress = 2048;
	pProg->SetFocus();

	CloseHandle(hComm);

}


void CBMUsendDoc::OnSetserial()
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
	CDialog::OnSetFocus(pOldWnd);

	m_Bar.SetPos(theApp.m_nProgress);
}

void CBMUsendDoc::OnFileSaveas()
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
		f.Write("\x01""\x01""\x02""\x01""\x04", 5);		// ^a^a ^B^A^D
		f.Write(m_HexBuf, 2048-2);
		f.Close();
	}
	
}
