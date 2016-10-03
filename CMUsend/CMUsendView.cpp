// CMUSendView.cpp : implementation of the CCMUSendView class
//

#include "stdafx.h"
#include "CMUSend.h"

#include "CMUSendDoc.h"
#include "CMUSendView.h"
#include "MainFrm.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CCMUSendView

IMPLEMENT_DYNCREATE(CCMUSendView, CView)

BEGIN_MESSAGE_MAP(CCMUSendView, CView)
	ON_WM_PAINT()
END_MESSAGE_MAP()

// CCMUSendView construction/destruction

CCMUSendView::CCMUSendView()
{
	// TODO: add construction code here

}

CCMUSendView::~CCMUSendView()
{
}

BOOL CCMUSendView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return CView::PreCreateWindow(cs);
}

// CCMUSendView drawing

void CCMUSendView::OnDraw(CDC* /*pDC*/)
{
	CCMUSendDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO: add draw code for native data here
}


// CCMUSendView diagnostics

#ifdef _DEBUG
void CCMUSendView::AssertValid() const
{
	CView::AssertValid();
}

void CCMUSendView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CCMUSendDoc* CCMUSendView::GetDocument() const // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CCMUSendDoc)));
	return (CCMUSendDoc*)m_pDocument;
}
#endif //_DEBUG


// CCMUSendView message handlers

void CCMUSendView::OnPaint()
{
	CPaintDC dc(this); // device context for painting
	// TODO: Add your message handler code here
	// Do not call CView::OnPaint() for painting messages
	CCMUSendDoc* pDoc = GetDocument();
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
