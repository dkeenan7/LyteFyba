// CMUSendView.h : interface of the CCMUSendView class
//


#pragma once


class CCMUSendView : public CView
{
protected: // create from serialization only
	CCMUSendView();
	DECLARE_DYNCREATE(CCMUSendView)

// Attributes
public:
	CCMUSendDoc* GetDocument() const;

// Operations
public:

// Overrides
public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:

// Implementation
public:
	virtual ~CCMUSendView();
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

#ifndef _DEBUG  // debug version in CMUSendView.cpp
inline CCMUSendDoc* CCMUSendView::GetDocument() const
   { return reinterpret_cast<CCMUSendDoc*>(m_pDocument); }
#endif

