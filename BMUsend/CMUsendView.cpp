// BMUsendView.cpp : implementation of the CBMUsendView class
//

#include "stdafx.h"
#include "BMUsend.h"

#include "BMUsendDoc.h"
#include "BMUsendView.h"
#include "MainFrm.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CBMUsendView

IMPLEMENT_DYNCREATE(CBMUsendView, CView)

BEGIN_MESSAGE_MAP(CBMUsendView, CView)
	ON_WM_PAINT()
END_MESSAGE_MAP()

// CBMUsendView construction/destruction

CBMUsendView::CBMUsendView()
{
	// TODO: add construction code here

}

CBMUsendView::~CBMUsendView()
{
}

BOOL CBMUsendView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return CView::PreCreateWindow(cs);
}

// CBMUsendView drawing

void CBMUsendView::OnDraw(CDC* /*pDC*/)
{
	CBMUsendDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO: add draw code for native data here
}


// CBMUsendView diagnostics

#ifdef _DEBUG
void CBMUsendView::AssertValid() const
{
	CView::AssertValid();
}

void CBMUsendView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CBMUsendDoc* CBMUsendView::GetDocument() const // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CBMUsendDoc)));
	return (CBMUsendDoc*)m_pDocument;
}
#endif //_DEBUG


// CBMUsendView message handlers

void CBMUsendView::OnPaint()
{
	CPaintDC dc(this); // device context for painting
	// TODO: Add your message handler code here
	// Do not call CView::OnPaint() for painting messages
	CBMUsendDoc* pDoc = GetDocument();
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
}
