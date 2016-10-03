// CMUSendDoc.h : interface of the CCMUSendDoc class
//

#include "EnumSerial.h"

#pragma once


class CCMUSendDoc : public CDocument
{
protected: // create from serialization only
	CCMUSendDoc();
	DECLARE_DYNCREATE(CCMUSendDoc)

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
	virtual ~CCMUSendDoc();
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


