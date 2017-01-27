// LyteFlash.cpp : Defines the class behaviors for the application.
//

#include "stdafx.h"
#include "LyteFlash.h"
#include "MainFrm.h"

#include "LyteFlashDoc.h"
#include "LyteFlashView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CLyteFlashApp

BEGIN_MESSAGE_MAP(CLyteFlashApp, CWinApp)
	ON_COMMAND(ID_APP_ABOUT,		&CLyteFlashApp::OnAppAbout)
END_MESSAGE_MAP()


// CLyteFlashApp construction

CLyteFlashApp::CLyteFlashApp()
	: m_start_off(0)
	, m_total_len(0)
	, m_len_to_send(0)
	, m_nPortIndex(0)
{
}


// The one and only CLyteFlashApp object

CLyteFlashApp theApp;

// CLyteFlashApp initialization

BOOL CLyteFlashApp::InitInstance()
{
	// InitCommonControlsEx() is required on Windows XP if an application
	// manifest specifies use of ComCtl32.dll version 6 or later to enable
	// visual styles.  Otherwise, any window creation will fail.
	INITCOMMONCONTROLSEX InitCtrls;
	InitCtrls.dwSize = sizeof(InitCtrls);
	// Set this to include all the common control classes you want to use
	// in your application.
	InitCtrls.dwICC = ICC_WIN95_CLASSES;
	InitCommonControlsEx(&InitCtrls);

	CWinApp::InitInstance();

	// Standard initialization
	// If you are not using these features and wish to reduce the size
	// of your final executable, you should remove from the following
	// the specific initialization routines you do not need
	// Change the registry key under which our settings are stored
	// TODO: You should modify this string to be something appropriate
	// such as the name of your company or organization
	SetRegistryKey(_T("Local AppWizard-Generated Applications"));
	LoadStdProfileSettings(8);  // Load standard INI file options (including MRU)
	// Register the application's document templates.  Document templates
	//  serve as the connection between documents, frame windows and views
	CSingleDocTemplate* pDocTemplate;
	pDocTemplate = new CSingleDocTemplate(
		IDR_MAINFRAME,
		RUNTIME_CLASS(CLyteFlashDoc),
		RUNTIME_CLASS(CMainFrame),       // main SDI frame window
		RUNTIME_CLASS(CLyteFlashView));
	if (!pDocTemplate)
		return FALSE;
	AddDocTemplate(pDocTemplate);

	// Parse command line for standard shell commands, DDE, file open
	CCommandLineInfo cmdInfo;
	ParseCommandLine(cmdInfo);


	// Dispatch commands specified on the command line.  Will return FALSE if
	// app was launched with /RegServer, /Register, /Unregserver or /Unregister.
	if (!ProcessShellCommand(cmdInfo))
		return FALSE;

	// The one and only window has been initialized, so show and update it

	m_pMainWnd->MoveWindow(100, 100, 600, 300);
	m_pMainWnd->ShowWindow(SW_SHOW);
	m_pMainWnd->UpdateWindow();
	// call DragAcceptFiles only if there's a suffix
	//  In an SDI app, this should occur after ProcessShellCommand

	m_bFileValid = false;
	m_nProgress = 0;
	_tcscpy_s(m_szShortName, _T("No file loaded"));
	EnumSerialPorts(m_asiPorts, true);

	UpdateMenu();

	return TRUE;
}



// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()



// App command to run the dialog
void CLyteFlashApp::OnAppAbout()
{
	CAboutDlg aboutDlg;
	aboutDlg.DoModal();
}


// CLyteFlashApp message handlers



void CLyteFlashApp::UpdateTitle()
{
	CString csPort;
	if (m_asiPorts.GetCount() == 0)
		csPort = _T("No port available");
	else
		csPort = m_asiPorts[m_nPortIndex].strFriendlyName;
	m_pMainWnd->SetWindowText(CString(m_szShortName) + _T(" -> ") + csPort);
}

void CLyteFlashApp::Adjust_start_and_len() {
	// Calculate start offset and length to send based on the image selection. In the length to send, don't include
	//	reset vector (never sent) but do include the checksum (not sent from the image)
	unsigned uBSL2_len = 0;
	if (m_uResetVec == 0xFC00)
		uBSL2_len = 1024;				// Now using 1K BSL2
	else if (m_uResetVec == 0xFE00)
		uBSL2_len = 512;
	m_start_off = 0;					// Start at the beginning of the image
	m_len_to_send = m_total_len - uBSL2_len; // Remove space for BSL2 (1 or 2 flash segments)

	if (m_total_len == 8 * 1024) {
		CMenu *pMenu = m_pMainWnd->GetMenu();
			pMenu->CheckMenuItem(ID_PASSWORD_REV61, MF_CHECKED | MF_BYCOMMAND);
			pMenu->CheckMenuItem(ID_PASSWORD_TRUNK, MF_UNCHECKED | MF_BYCOMMAND);
		}
	else {
		CMenu *pMenu = m_pMainWnd->GetMenu();
		pMenu->CheckMenuItem(ID_PASSWORD_TRUNK, MF_CHECKED | MF_BYCOMMAND);
		pMenu->CheckMenuItem(ID_PASSWORD_REV61, MF_UNCHECKED | MF_BYCOMMAND);
	}


	m_pMainWnd->InvalidateRect(NULL);
	m_pMainWnd->UpdateWindow();	// Force a paint of the number of bytes
}

void CLyteFlashApp::UpdateMenu() {
	// Used to do a lot of checking and unchecking in the Image submenu here
	Adjust_start_and_len();
}

CString CLyteFlashApp::GetRecentFile(int index) const
{
	return (*m_pRecentFileList)[index];
}