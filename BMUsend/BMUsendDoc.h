// BMUsendDoc.h : interface of the CBMUsendDoc class
//

#include "EnumSerial.h"

#pragma once


class CBMUsendDoc : public CDocument
{
protected: // create from serialization only
	CBMUsendDoc();
	DECLARE_DYNCREATE(CBMUsendDoc)

// Attributes
public:

// Operations
public:
	void	ReadFile();
	
// Overrides
public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);

// Implementation
public:
	virtual ~CBMUsendDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:
	unsigned char	m_HexBuf[2048];			// The 2K image for F800-FFFF
public:
	unsigned int	m_total_len;			// Number of bytes read into the image

// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnFileOpen();
public:
	afx_msg void OnUpdateSend(CCmdUI *pCmdUI);
	afx_msg void OnFileMruFile(UINT uiMsgId);
public:
	afx_msg void OnSend();
public:
	afx_msg void OnSetserial();
public:
	afx_msg void OnFileSaveas();
};


