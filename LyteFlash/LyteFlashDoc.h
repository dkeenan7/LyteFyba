// LyteFlashDoc.h : interface of the CLyteFlashDoc class
//

#include "EnumSerial.h"

#pragma once


class CLyteFlashDoc : public CDocument
{
protected: // create from serialization only
	CLyteFlashDoc();
	DECLARE_DYNCREATE(CLyteFlashDoc)

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
	virtual ~CLyteFlashDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:
	unsigned char*	m_pFileBuf;				// Pointer to binary file image
public:
	unsigned int	m_first_addr;			// Lowest address read from the file

// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnFileOpen();
private:
	afx_msg void OnUpdateSend(CCmdUI *pCmdUI);
	afx_msg void OnFileMruFile(UINT uiMsgId);
public:
	afx_msg void OnSend();
public:
	afx_msg void OnSetserial();
public:
	afx_msg void OnFileSaveas();
};


