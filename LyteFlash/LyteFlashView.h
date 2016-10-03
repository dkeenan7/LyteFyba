// LyteFlashView.h : interface of the CLyteFlashView class
//


#pragma once


class CLyteFlashView : public CView
{
protected: // create from serialization only
	CLyteFlashView();
	DECLARE_DYNCREATE(CLyteFlashView)

// Attributes
public:
	CLyteFlashDoc* GetDocument() const;

// Operations
public:

// Overrides
public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:

// Implementation
public:
	virtual ~CLyteFlashView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnPaint();
};

#ifndef _DEBUG  // debug version in LyteFlashView.cpp
inline CLyteFlashDoc* CLyteFlashView::GetDocument() const
   { return reinterpret_cast<CLyteFlashDoc*>(m_pDocument); }
#endif

