// LyteFlashView.cpp : implementation of the CLyteFlashView class
//

#include "stdafx.h"
#include "LyteFlash.h"

#include "LyteFlashDoc.h"
#include "LyteFlashView.h"
#include "MainFrm.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CLyteFlashView

IMPLEMENT_DYNCREATE(CLyteFlashView, CView)

BEGIN_MESSAGE_MAP(CLyteFlashView, CView)
	ON_WM_PAINT()
END_MESSAGE_MAP()

// CLyteFlashView construction/destruction

CLyteFlashView::CLyteFlashView()
{
	// TODO: add construction code here

}

CLyteFlashView::~CLyteFlashView()
{
}

BOOL CLyteFlashView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return CView::PreCreateWindow(cs);
}

// CLyteFlashView drawing

void CLyteFlashView::OnDraw(CDC* /*pDC*/)
{
	CLyteFlashDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO: add draw code for native data here
}


// CLyteFlashView diagnostics

#ifdef _DEBUG
void CLyteFlashView::AssertValid() const
{
	CView::AssertValid();
}

void CLyteFlashView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CLyteFlashDoc* CLyteFlashView::GetDocument() const // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CLyteFlashDoc)));
	return (CLyteFlashDoc*)m_pDocument;
}
#endif //_DEBUG


// CLyteFlashView message handlers

void CLyteFlashView::OnPaint()
{
	CPaintDC dc(this); // device context for painting
	// TODO: Add your message handler code here
	// Do not call CView::OnPaint() for painting messages
	CLyteFlashDoc* pDoc = GetDocument();
	TCHAR buf[100];
	if (theApp.m_total_len == 0)
		wcscpy_s(buf, sizeof(buf)/sizeof(TCHAR), _T("No file loaded"));
	else
		_stprintf_s(buf, sizeof(buf)/sizeof(TCHAR), _T("%d (%Xh) bytes"), theApp.m_len_to_send, theApp.m_len_to_send);
	dc.TextOut(5,5, buf, (int)_tcslen(buf));

	if (theApp.m_total_len) {
		_stprintf_s(buf, sizeof(buf)/sizeof(TCHAR), _T("starting at offset %d (%Xh)"), theApp.m_start_off, theApp.m_start_off);
		dc.TextOut(5,25, buf, (int)_tcslen(buf));
	}
	// Add the full file name (including drive and path) so we know it's the right one:
	dc.TextOutW(5, 65, theApp.m_szFileName, (int)_tcslen(theApp.m_szFileName));
}
