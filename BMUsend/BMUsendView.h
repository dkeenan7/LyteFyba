// BMUsendView.h : interface of the CBMUsendView class
//


#pragma once


class CBMUsendView : public CView
{
protected: // create from serialization only
	CBMUsendView();
	DECLARE_DYNCREATE(CBMUsendView)

// Attributes
public:
	CBMUsendDoc* GetDocument() const;

// Operations
public:

// Overrides
public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:

// Implementation
public:
	virtual ~CBMUsendView();
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

#ifndef _DEBUG  // debug version in BMUsendView.cpp
inline CBMUsendDoc* CBMUsendView::GetDocument() const
   { return reinterpret_cast<CBMUsendDoc*>(m_pDocument); }
#endif

