// BMUsend.cpp : Defines the class behaviors for the application.
//

#include "stdafx.h"
#include "BMUsend.h"
#include "MainFrm.h"

#include "BMUsendDoc.h"
#include "BMUsendView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CBMUsendApp

BEGIN_MESSAGE_MAP(CBMUsendApp, CWinApp)
	ON_COMMAND(ID_APP_ABOUT, &CBMUsendApp::OnAppAbout)
	// Standard file based document commands
	ON_UPDATE_COMMAND_UI(ID_SEND, &CBMUsendApp::OnUpdateSend)
END_MESSAGE_MAP()


// CBMUsendApp construction

CBMUsendApp::CBMUsendApp()
{
	// TODO: add construction code here,
	// Place all significant initialization in InitInstance
	m_nPortIndex = 0;
}


// The one and only CBMUsendApp object

CBMUsendApp theApp;

// CBMUsendApp initialization

BOOL CBMUsendApp::InitInstance()
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
		RUNTIME_CLASS(CBMUsendDoc),
		RUNTIME_CLASS(CMainFrame),       // main SDI frame window
		RUNTIME_CLASS(CBMUsendView));
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
	m_pMainWnd->ShowWindow(SW_SHOW);
	m_pMainWnd->UpdateWindow();
	// call DragAcceptFiles only if there's a suffix
	//  In an SDI app, this should occur after ProcessShellCommand

	m_bFileValid = false;
	m_nProgress = 0;
	_tcscpy_s(m_szShortName, _T("No file loaded"));
	EnumSerialPorts(m_asiPorts, true);

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
void CBMUsendApp::OnAppAbout()
{
	CAboutDlg aboutDlg;
	aboutDlg.DoModal();
}


// CBMUsendApp message handlers



void CBMUsendApp::OnUpdateSend(CCmdUI *pCmdUI)
{
}

void CBMUsendApp::UpdateTitle()
{
	CString csPort;
	if (m_asiPorts.GetCount() == 0)
		csPort = _T("No port available");
	else
		csPort = m_asiPorts[m_nPortIndex].strFriendlyName;
	m_pMainWnd->SetWindowText(CString(m_szShortName) + _T(" -> ") + csPort);
}