/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aUI.h	                                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent user 	   */
/*		interface layer.       				   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Copyright 1994-2010. Acroname Inc.                              */
/*                                                                 */
/* This software is the property of Acroname Inc.  Any             */
/* distribution, sale, transmission, or re-use of this code is     */
/* strictly forbidden except with permission from Acroname Inc.    */
/*                                                                 */
/* To the full extent allowed by law, Acroname Inc. also excludes  */
/* for itself and its suppliers any liability, wheither based in   */
/* contract or tort (including negligence), for direct,            */
/* incidental, consequential, indirect, special, or punitive       */
/* damages of any kind, or for loss of revenue or profits, loss of */
/* business, loss of information or data, or other financial loss  */
/* arising out of or in connection with this software, even if     */
/* Acroname Inc. has been advised of the possibility of such       */
/* damages.                                                        */
/*                                                                 */
/* Acroname Inc.                                                   */
/* www.acroname.com                                                */
/* 720-564-0373                                                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef _aUI_H_
#define _aUI_H_

#include "aIO.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * opaque reference to the UI library
 */

typedef aLIBREF aUILib;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * define symbol import mechanism
 */

#ifndef aUI_EXPORT
#define aUI_EXPORT aLIB_IMPORT
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * UI library manipulation routines
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

aUI_EXPORT aLIBRETURN 
aUI_GetLibRef(aUILib* pUIRef, 
	      aErr* pErr);

aUI_EXPORT aLIBRETURN 
aUI_ReleaseLibRef(aUILib UIRef, 
		  aErr* pErr);

aUI_EXPORT aLIBRETURN 
aUI_GetVersion(aUILib UIRef, 
	       unsigned long* pVersion,
	       aErr* pErr);

#ifdef __cplusplus 
}
#endif /* __cplusplus */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Common Dialogs
 */

typedef aErr (*aUIDialogIdleProc)(const void* ref);
typedef aBool (*aUIPickFileFilterProc)(const char* pFilename,
				       const unsigned long nSize);

#ifdef __cplusplus
extern "C" {
#endif

aUI_EXPORT aLIBRETURN 
aDialog_Message(aUILib uiRef,
		const char* pMessage,
		aUIDialogIdleProc idleProc,
		void* idleRef,
		aErr* pErr);

aUI_EXPORT aLIBRETURN 
aDialog_PickFile(aUILib uiRef,
		 const char* pMessage,
		 char* pFileName,
		 const aFileArea eFileArea,
		 aUIPickFileFilterProc filterProc,
		 aUIDialogIdleProc idleProc,
		 void* idleRef,
		 aErr* pErr);

#ifdef __cplusplus 
}
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Graphics Device definitions
 */

typedef void* aGDRef;

typedef short aUIPixelType;
typedef float aUIAngleType;

typedef struct aPT {
  aUIPixelType x;
  aUIPixelType y;
} aPT;

typedef struct aRECT {
  aUIPixelType x;
  aUIPixelType y;
  aUIPixelType width;
  aUIPixelType height;
} aRECT;

#ifdef aUNIX
typedef struct aUnixGDCreateData {
  void* pDisplay;
  int parent;
} aUnixGDCreateData;
#endif

#define aUIDEFAULTFONTSIZE	12
#define aUIALIGNCENTER		0x01
#define aUIALIGNLEFT		0x02
#define aUIALIGNRIGHT		0x04
#define aUIALIGNMASK		0x07
#define aUITEXTINVISIBLE	0x10
#define aUIFIXEDWIDTHFONT       0x20

#define aUI_SCREEN_DPI          72
#define aUI_IMAGE_BUFFER_SIZE   4096

typedef struct aFONTDEF {
  char flags;
} aFONTDEF;

#ifdef __cplusplus
extern "C" {
#endif
  
  aUI_EXPORT aLIBRETURN 
  aGD_Create(aUILib uiRef,
	     const aRECT* pRect, 
	     void* createData,
	     aGDRef* pGDRef,
	     aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_CreateOffscreen(aUILib uiRef,
		      const aUIPixelType width,
		      const aUIPixelType height,
		      aGDRef* pGDRef,
		      aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_GetGraphicsMemory(aUILib uiRef,    // only available for offscreen
			aGDRef gdRef,
			char** ppMemory,
			aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_SetPixel(aUILib uiRef,             // only available for offscreen
	       aGDRef gdRef,
	       const aPT* pPoint,
	       const aColorType color,
	       aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_CreatePDF(aUILib uiRef,
		const char* pFileName,
		const aFileArea eFileArea,
		aGDRef* pGDRef,
		aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_GetSize(aUILib uiRef,
	      aGDRef gdRef,
	      aRECT* pRect, 
	      aErr* pErr);
    
  aUI_EXPORT aLIBRETURN 
  aGD_SetSize(aUILib uiRef,
	      aGDRef gdRef,
	      const aRECT* pRect, 
	      aErr* pErr);

  aUI_EXPORT aLIBRETURN 
  aGD_GetDPI(aUILib uiRef,
	     aGDRef gdRef,
	     aUInt16* pDPI,
	     aErr* pErr);

  aUI_EXPORT aLIBRETURN
  aGD_Scale(aUILib uiRef,                // only available for offscreen
	    aGDRef* pGDRef,
	    const aUIPixelType width,
	    const aUIPixelType height,
	    const aUInt16 dpi,
	    aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_StartDrawing(aUILib uiRef,
		   aGDRef gdRef,
		   aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_Erase(aUILib uiRef,
	    aGDRef gdRef,
	    aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_SetColor(aUILib uiRef,
	       aGDRef gdRef,
	       const aColorType color,
	       aErr* pErr);
  
  aUI_EXPORT aLIBRETURN
  aGD_Line(aUILib uiRef,
	   aGDRef gdRef,
	   const aPT* pPoints,
	   const unsigned int nPoints,
	   aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_Rect(aUILib uiRef,
	   aGDRef gdRef,
	   const aRECT* pRect,
	   const aBool bFilled,
	   aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_Elipse(aUILib uiRef,
	     aGDRef gdRef,
	     const aRECT* pRect,
	     const aBool bFilled,
	     aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_RoundedRect(aUILib uiRef,
		  aGDRef gdRef,
		  const aRECT* pRect,
		  const aUIPixelType radius,
		  aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_Arc(aUILib uiRef,
	  aGDRef gdRef,
	  const aPT* pCenter,
	  const aUIPixelType radius,
	  const aUIAngleType startAngle,
	  const aUIAngleType endAngle,
	  aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_SetFont(aUILib uiRef,
	      aGDRef gdRef,
	      const aFONTDEF* pFontDef,
	      aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_Text(aUILib uiRef,
	   aGDRef gdRef,
	   const aRECT* pRect,
	   const int flags,
	   const unsigned int nSize,
	   const char* pText,
	   aErr* pErr);
  
  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * aGD_Paragraph
   * Computes and can draw paragraph text on a GD.
   * The rectangle passed in has the maximum allowed size of the text.
   * If you pass in a height of -1 initially, the height will
   * be computed without limit until the text is all used.
   *
   * Setting the aUITEXTINVISIBLE flag only computes, but doesn't draw
   * the text.
   *
   * pnLines can be NULL or a pointer to an unsigned int that recieves
   * the number of lines of text used.  This times the line size gives
   * the pixels height of the paragraph.
   *
   * ppText points to the starting point of the text to draw and will 
   * be updated to point to point that either the text or space ran out.
   */
  aUI_EXPORT aLIBRETURN 
  aGD_Paragraph(aUILib uiRef,
		aGDRef gdRef,
		const aRECT* pRect,
		const int flags,
		const unsigned int nSize,
		char** ppText,
		unsigned int* pnLines,
		aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_Copy(aUILib uiRef,
	   aGDRef gdRef,
	   const aRECT* pDstRect,
	   aGDRef srcGDRef,
	   const aRECT* pSrcRect,
	   aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_EndDrawing(aUILib uiRef,
		 aGDRef gdRef,
		 aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_ReadJPG(aUILib uiRef,
	      aStreamRef jpgStream,
	      aGDRef* pGDRef,
	      aErr* pErr);
  
  /* dpi should default to aUI_SCREEN_DPI */
  /* quality is in the range 0-100 */
  aUI_EXPORT aLIBRETURN 
  aGD_WriteJPG(aUILib uiRef,
	       aGDRef gdRef,
	       const aUInt16 dpi,
	       const aUInt16 quality,
	       aStreamRef jpgStream,
	       aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_WritePNG(aUILib uiRef,
	       aGDRef gdRef,
	       aStreamRef pngStream,
	       aErr* pErr);
  
  aUI_EXPORT aLIBRETURN 
  aGD_TextWidth(aUILib uiRef,
		aGDRef gdRef,
		const char* pText,
		const unsigned int nSize,
		unsigned int* pnWidth,
		aErr* pErr);
  
  aUI_EXPORT aLIBRETURN
  aGD_Destroy(aUILib uiRef,
	      aGDRef gdRef,
	      aErr* pErr);
  
#if defined(aUNIX) && !defined(aMACX)
  aUI_EXPORT aLIBRETURN 
  aGD_HandleEvent(aUILib uiRef,
		  void* pXEvent,
		  aBool* pbHandled,
		  aErr *pErr);
#endif /* aUNIX && !aMACX */
  
  /* utility functions */
  aUI_EXPORT aColorType 
  aUI_BlendColor(const aColorType c1,
		 const aColorType c2);
  
  aUI_EXPORT aColorType 
  aUI_LightenColor(const aColorType c,
		   const float amount);
  
  aUI_EXPORT void 
  aUI_InsetRect(aRECT* pRect,
		const aUIPixelType amount);
  
  aUI_EXPORT aBool
  aUI_PointInRect(const aRECT* r, 
		  const aPT* p);
  
#ifdef __cplusplus 
}
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * HTTP Definitions
 */

#define aHTTPMAXREQESTLINE	80
#define aHTTPMAXTEMPLATELINE	200
#define aHTTPMAXPARAMLEN	aMAXIDENTIFIERLEN
#define aHTTPSETTINGFILE        "http.config"
#define aHTTPSETTINGMAX         32
#define aHTTPPORTKEY            "http-port"
#define aHTTPPORTDEFAULT        8000
#define aHTTPADDRKEY            "http-address"
#define aHTTPADDRDEFAULT        0x7F000001

typedef void* aHTTPRef;

typedef aErr (*aHTTPRequestProc)(const char* pURLStr,
				 aSymbolTableRef params,
				 aStreamRef reply,
				 void* vpRef);
typedef aErr (*aHTTPTemplateProc)(const unsigned int nParamIndex,
				  const unsigned int nBlockIndex,
				  aStreamRef reply,
				  void* vpRef);

#ifdef __cplusplus
extern "C" {
#endif

aUI_EXPORT aLIBRETURN 
aHTTP_Create(aUILib uiRef,
	     aSettingFileRef settings,
	     aHTTPRequestProc requestProc,
	     void* vpRef,
	     aHTTPRef* pHTTPRef,
	     aErr* pErr);

aUI_EXPORT aLIBRETURN 
aHTTP_TimeSlice(aUILib uiRef,
		aHTTPRef http,
		aBool* bChanged,
		aErr* pErr);

aUI_EXPORT aLIBRETURN 
aHTTP_Template(aUILib uiRef,
	       aHTTPRef http,
	       const char* pNameStr,
	       aHTTPTemplateProc templateProc,
	       void* vpRef,
	       aStreamRef reply,
	       aErr* pErr);

aUI_EXPORT aLIBRETURN 
aHTTP_Destroy(aUILib uiRef,
	      aHTTPRef http,
	      aErr* pErr);

#ifdef __cplusplus 
}
#endif



#ifdef __cplusplus
extern "C" {
#endif

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Browser Definitions
 */

aUI_EXPORT aLIBRETURN 
aBrowser_LaunchURL(aUILib uiRef,
		   const char* pURLStr,
		   aErr* pErr);

#ifdef __cplusplus 
}
#endif



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Graphics Widget definitions
 */


#ifdef __cplusplus
extern "C" {
#endif

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Browser Definitions
 */

aUI_EXPORT aLIBRETURN 
aWidget_CreateLogView(aUILib uiRef,
		      void* createData,
		      const aRECT* pRect, 
		      const unsigned int nFontSize,
		      const unsigned int nMaxLines,
		      aStreamRef* pLogStream,
		      aErr* pErr);

#ifdef __cplusplus 
}
#endif


#ifdef __cplusplus
////////////////////////////////////////////////////////////////////////////////

#define aVIEW_ATTR_ANCHOR_LEFT    0x0001
#define aVIEW_ATTR_ANCHOR_RIGHT   0x0002
#define aVIEW_ATTR_ANCHOR_TOP	  0x0004
#define aVIEW_ATTR_ANCHOR_BOTTOM  0x0008

class aUI_EXPORT acpView
{
public:
  acpView(acpView* pParent = NULL);
  virtual ~acpView();
  
  virtual void setBounds(const aRECT& r);
  virtual void getBounds(aRECT& r) const;

  virtual void show(void) = 0;
  virtual void hide(void) = 0;
  
  virtual void recalculateSize(aPT& size);
  virtual void shrinkWrap(void);
  
  virtual void boundsChanged(void);

  virtual bool handleKeyDown(const aKeyType key);
  virtual bool handleKeyUp(const aKeyType key);
  virtual bool filterKey(const aKeyType key);
  // clicks for buttons
  virtual bool handleClick(acpView* pView);
  // selection for tabs, popups, list
  virtual bool handleSelection(acpView* pContainer);
  // value for checkbox, value, slider, edit
  virtual bool handleValue(acpView* pControl);

  void setMargin(const aUIPixelType margin);
  void setAttributes(const unsigned short attributes);
  void setAspectRatio(const float fRatio);

  void getMargins(aUIPixelType& left,
		  aUIPixelType& top,
		  aUIPixelType& right,
		  aUIPixelType& bottom);

  acpView* getParent(void) const { return m_pParent; }

  void* m_viewPrivate;

protected:
  void deleteChildren(void);

  acpView* m_pParent;
  acpView* m_pSiblings;
  acpView* m_pChildren;
  aRECT m_bounds;

  // private?
  unsigned int m_attributes;
  aUIPixelType m_marginLeft;
  aUIPixelType m_marginRight;
  aUIPixelType m_marginTop;
  aUIPixelType m_marginBottom;
  float m_fAspectRatio;

  friend class acpRowView;
  friend class acpColumnView;
  friend class acpDialogWindow;
};


class aUI_EXPORT acpRowView : public acpView
{
public:
  acpRowView(acpView* pParent = NULL);
  virtual ~acpRowView();
  
  virtual void setBounds(const aRECT& r);
  
  virtual void show(void);
  virtual void hide(void);
  
  virtual void recalculateSize(aPT& size);
};


class aUI_EXPORT acpColumnView  : public acpView
{
public:
  acpColumnView(acpView* pParent = NULL);
  virtual ~acpColumnView();  
  
  virtual void setBounds(const aRECT& r);
  
  virtual void show(void);
  virtual void hide(void);
  
  virtual void recalculateSize(aPT& size);
};


////////////////////////////////////////////////////////////////////////////////
// we size to 800 x 600 and center on a 1024 x 768 window
#define aMAXSETTINGLEN		64
#define aSETTING_BACKGROUND	"background"
#define aDEFAULT_BACKGROUND	0x99CC99
#define aSETTING_WIDTH		"width"
#define aDEFAULT_WIDTH		800
#define aSETTING_HEIGHT		"height"
#define aDEFAULT_HEIGHT		450
#define aSETTING_XPOS		"x"
#define aDEFAULT_XPOS		112
#define aSETTING_YPOS		"y"
#define aDEFAULT_YPOS		84

#define aCMD_OPEN		100
#define aCMD_CLOSE		101
#define aCMD_EXIT		102
#define aCMD_CLOSEWINDOW	103
#define aCMD_LAST_CORE		aCMD_CLOSEWINDOW

#define aAPP_ATTR_RESIZABLE	0x00000001

/////////////////////////////////////////////////////////////////////

class aUI_EXPORT acpControl : public acpView {
public:
  acpControl(acpView* pParent);
  virtual ~acpControl(void);
  
  virtual bool isEnabled(void);
  virtual bool setEnable(const bool bEnabled);

  virtual void redraw(void) {}
  virtual void setText(const acpString& text) {}

private:
  bool m_bEnabled;
};


/////////////////////////////////////////////////////////////////////

class aUI_EXPORT acpControlButton : public acpControl 
{
public:
  acpControlButton(acpView* pParentView,
		   const acpString& text);
  virtual ~acpControlButton(void);
  
  virtual void setText(const acpString& text) = 0;
  virtual void getText(acpString& text) const = 0;
};


/////////////////////////////////////////////////////////////////////

class aUI_EXPORT acpControlCheckbox : public acpControl 
{
public:
  acpControlCheckbox(acpView* pParentView,
		     const acpString& text,
		     const bool bChecked = false);
  virtual ~acpControlCheckbox(void);


  virtual operator bool(void) const = 0;
  virtual bool operator=(const bool value) = 0;
  virtual void setText(const acpString& text) = 0;
  virtual void getText(acpString& text) const = 0;
};


/////////////////////////////////////////////////////////////////////

class aUI_EXPORT acpControlDrawable : public acpControl 
{
public:
  acpControlDrawable(acpView* pParentView,
		     const aUIPixelType width,
		     const aUIPixelType height);
  virtual ~acpControlDrawable(void);

  virtual void drawToGD(aGDRef gd) = 0;
  virtual void erase(void) = 0;
  virtual void setColor(const aColorType color) = 0;
  virtual void line(const aPT* pPoints, const unsigned int nPoints) = 0;
  virtual void rect(const aRECT& r, const bool bFilled = false) = 0;
  virtual void elipse(const aRECT& r, const bool bFilled = false) = 0;
  virtual void roundedRect(const aRECT& r, const aUIPixelType radius) = 0;
  virtual void arc(const aPT& center, const aUIPixelType radius,
		   const aUIAngleType startAngle, 
		   const aUIAngleType endAngle) = 0;
  virtual void setFont(const aFONTDEF& font) = 0;
  virtual void text(const aRECT& r, const int flags, const unsigned int nSize,
		    const acpString& string) = 0;
  virtual void copy(const aRECT& dstRect,
		    aGDRef srcGD,
		    const aRECT& srcRect) = 0;
  
  // bracket drawing with these calls
  virtual void start(void) = 0;
  virtual void finish(void) = 0;
};


/////////////////////////////////////////////////////////////////////

class aUI_EXPORT acpControlEdit : public acpControl 
{
public:
  acpControlEdit(acpView* pParentView,
		 const acpString& text);
  virtual ~acpControlEdit(void);

  virtual void setWidth(const int width) = 0;
  virtual void setText(const acpString& text) = 0;
  virtual void getText(acpString& text) const = 0;
  virtual void getSelection(unsigned int& start, 
			    unsigned int& end) = 0;
};


/////////////////////////////////////////////////////////////////////

class aUI_EXPORT acpControlGL : public acpControl 
{
public:
  acpControlGL(acpView* pParentView);
  virtual ~acpControlGL(void);

  // acpControlGL
  virtual void init(void) = 0;
  virtual void setBounds(const aRECT& r);
  virtual void prepare(void) = 0;
  virtual void finalize(void) = 0;
};


/////////////////////////////////////////////////////////////////////

class aUI_EXPORT acpControlLabel : public acpControl 
{
public:
  typedef enum {
    kHeading,
    kBody
  } style;
  acpControlLabel(acpView* pParentView,
		 const style s,
		 const acpString& text);
  virtual ~acpControlLabel(void);
  virtual void setStyle(const style s) { m_style = s; }
  virtual style getStyle(void) { return m_style; }
  virtual void setText(const acpString& text) = 0;
  virtual void getText(acpString& text) const = 0;
  virtual void setWidth(const unsigned int width) = 0;

protected:
  style m_style;
};


/////////////////////////////////////////////////////////////////////

class aUI_EXPORT acpControlLED : public acpControl 
{
public:
  acpControlLED(acpView* pParentView,
		const aUIPixelType nRadius,
		const aColorType color);
  virtual ~acpControlLED(void);
  
  virtual void recalculateSize(aPT& size);
  
  virtual void setColor(const aColorType color);
  virtual bool getState() const;
  virtual void setState(const bool bOn);
  
protected:
  bool m_bOn;
  aUIPixelType m_nRadius;
  aColorType m_color;
  aColorType m_offColor;    
};


/////////////////////////////////////////////////////////////////////

class aUI_EXPORT acpControlList : public acpControl 
{
public:

  class item {
  public:
    item(void) {}
    virtual ~item(void) {}
    virtual const char* display(const unsigned int column) { return ""; }
    virtual int sort(const item& other) { return 0; }
  };

  acpControlList(acpView* pParentView);
  virtual ~acpControlList(void);
  virtual void addColumn(const acpString& name,
			 const aUIPixelType pixelWidth) = 0;
  virtual void add(item* pItem) = 0;
  virtual item* getSelection(void) const { return m_pSelection; }
  virtual void select(item* pItem) = 0;
  virtual void clear(void) = 0;

protected:
  void setSelection(item* pItem) { m_pSelection = pItem; }

private:
  item* m_pSelection;
};


/////////////////////////////////////////////////////////////////////

class aUI_EXPORT acpControlLog : public acpControl 
{
public:
  acpControlLog(acpView* pParentView,
		unsigned int nLines = 1000);
  virtual ~acpControlLog(void);
  // dims is width and height
  virtual void setSize(const aPT& dims) = 0;
  // lines is number of log lines to retain
  virtual void setLines(const unsigned int nLines) { m_nLines = nLines; }
  virtual unsigned int getLines(void) const { return m_nLines; }
  // clears entire log
  virtual void erase(void);
  // resets log to a single line
  virtual const char* operator=(const char* pLine) = 0;
  // appends a line
  virtual const char* operator+=(const char* pLine) = 0;
protected:
  aPT m_dims;
  unsigned int m_nLines;
};


/////////////////////////////////////////////////////////////////////

class aUI_EXPORT acpControlPopup : public acpControl 
{
public:
  acpControlPopup(acpView* pParentView);
  virtual ~acpControlPopup(void);
  virtual void addItem(const acpString& name) = 0;
  // selection is zero for none, 1-based for entries
  virtual operator unsigned int (void) const = 0;
  virtual unsigned int operator=(const unsigned int item) = 0;
  virtual void getItemText(const unsigned int item,
			   acpString& text) const = 0;
};


/////////////////////////////////////////////////////////////////////

class aUI_EXPORT acpControlProgress : public acpControl
{
public:
  acpControlProgress(acpView* pParentView);
  virtual ~acpControlProgress(void);

  // setting normalized from 0 to 1
  virtual operator float (void) const = 0;
  virtual float operator=(const float amount) = 0;
};


/////////////////////////////////////////////////////////////////////

class aUI_EXPORT acpControlSlider : public acpControl 
{
public:
  acpControlSlider(acpView* pParentView,
		   const int min,
		   const int max,
		   const int val);
  virtual ~acpControlSlider(void);
  virtual void setMin(const int min) = 0;
  virtual int getMin(void) const = 0;
  virtual void setMax(const int max) = 0;
  virtual int getMax(void) const = 0;
  virtual int operator=(const int val) = 0;
  virtual operator int(void) const = 0;
};


/////////////////////////////////////////////////////////////////////

#define aBUTTONCONTROLHEIGHT  14
class aUI_EXPORT acpControlTab : public acpControl 
{
public:
  acpControlTab(acpView* pParentView);
  virtual ~acpControlTab(void);
  virtual void recalculateSize(aPT& size) = 0;  
  virtual acpControl* addTab(void) = 0;
  virtual void setTab(const unsigned int nTab);  
  virtual unsigned int currentTab(void) { return m_nCurrent; }

protected:
  acpList<acpControl> m_tabs;
  
private:
  unsigned int m_nCurrent;
};


/////////////////////////////////////////////////////////////////////

class aUI_EXPORT acpControlValue : public acpControl 
{
public:
  typedef enum {
    kFloat,       // editable values
    kInt,
    kFloatView,   // read-only values
    kIntView
  } type;
  acpControlValue(acpView* pParentView,
		  const acpString& title,
		  const acpControlValue::type t);
  virtual ~acpControlValue(void);
  
  virtual void setSize(const unsigned int title, 
		       const unsigned int value) = 0;
  virtual void setMin(const float min) = 0;
  virtual void setMax(const float max) = 0;
  virtual int operator=(const int v) = 0;
  virtual float operator=(const float v) = 0;
  virtual operator float(void) const = 0;
  virtual operator int(void) const = 0;
  virtual void setText(const acpString& text) = 0;
  virtual void getText(acpString& text) const = 0;
  virtual bool handleValue(acpView* pControl) = 0;
protected:
  type m_type;
};


/////////////////////////////////////////////////////////////////////

class acpOSApplication;
class acpDialogPickFile;

class aUI_EXPORT acpApplication : public acpView {
public:
  acpApplication(const char* name,
		 unsigned long appAttributes = 0);
  virtual ~acpApplication(void);
  
  virtual void show(void);
  virtual void hide(void);
  
  // init should only do work that is preparation for running
  // based on the argc/argv inputs.
  virtual bool init(const int argc,
		    const char* argv[]);
  
  virtual void dispatchCommand(int nCmd);
  
  virtual void message(const acpString& msg);
  
  virtual void cmdOK(void);
  virtual void cmdExit(void);
  virtual void cmdFileOpen(const aFileArea eArea,
		           const acpString& filename);

  virtual int runApp(void);

  // set the bounds of the client area
  virtual void setBounds(const aRECT& r);
  // notification of bounds changes
  virtual void boundsChanged(void);

  virtual bool idle(void);

  // createMainWindow builds the actual application window.  Subclasses
  // will want to first call this superclass method to establish the 
  // main window, then fill in any specifics required for that application.
  virtual void createMainWindow(void);
  
  void setWindowTitle(const acpString& title);

  // This is the loop that runs the app.  You can subclass and do final
  // initialization after the UI framework is all built and then chain
  // to the base class.
  virtual void runLoop(void);
  
  // These creation methods must be called after creating the main window.
  virtual aGDRef createGD(const aRECT& bounds,
			  acpView* pParent = NULL);
  virtual acpControlButton* createButtonControl(acpView* pParent,
						const char* text);
  virtual acpControlCheckbox* createCheckboxControl(acpView* pParent,
						const char* text,
						const bool bChecked = false);
  virtual acpControlDrawable* createDrawableControl(acpView* pParent,
						    const aUIPixelType width,
						    const aUIPixelType height);
  virtual acpControlEdit* createEditControl(acpView* pParent,
					    const acpString& text);
  virtual acpControlGL* createGLControl(acpView* pParent);
  virtual acpControlLabel* createLabelControl(acpView* pParent,
					      const acpControlLabel::style s,
					      const acpString& text);
  virtual acpControlLED* createLEDControl(acpView* pParent,
					  const aUIPixelType nRadius,
					  const aColorType color);
  virtual acpControlList* createListControl(acpView* pParent);
  virtual acpControlLog* createLogControl(acpView* pParent,
					  const unsigned int nLines = 100);
  virtual acpControlPopup* createPopupControl(acpView* pParent);
  virtual acpControlProgress* createProgressControl(acpView* pParent);
  virtual acpControlSlider* createSliderControl(acpView* pParent,
						const int min,
						const int max,
						const int val);
  virtual acpControlTab* createTabControl(acpView* pParent);
  virtual acpControlValue* createValueControl(acpView* pParent,
					      const char* title,
					      const acpControlValue::type t);

  void sendMessage(acpMessage* pMessage);
  
  acpMutex* m_pMessageListMutex;
  acpList<acpMessage> m_messages;

  aIOLib ioRef(void) const;
  aUILib uiRef(void) const;
  aSettingFileRef settings(void) const; 
  const char* appName(void) const;
  aColorType backgroundColor(void) const;
  bool done(void) const;
  virtual unsigned long attributes(void) const;

protected:
  aIOLib m_ioRef;
  aUILib m_uiRef;
  aSettingFileRef m_settings;

  unsigned long m_nMSSleepTicks;
  acpString m_appName;

  aColorType m_backgroundColor;
  unsigned long m_attributes;

private:
  bool m_bMainWindowCreated;
  bool m_bDone;
  acpOSApplication* m_pOSApp;
  friend class acpDialog;
  friend class acpDialogWindow;
};

/////////////////////////////////////////////////////////////////////
// inline functions (must be in header file)
inline void acpApplication::cmdOK(void) 
{ }
inline void acpApplication::cmdExit(void) 
{ m_bDone = aTrue; }
inline aIOLib acpApplication::ioRef(void) const 
{ return m_ioRef; }
inline aUILib acpApplication::uiRef(void) const 
{ return m_uiRef; }
inline aSettingFileRef acpApplication::settings(void) const
{ return m_settings; }
inline const char* acpApplication::appName(void) const
{ return m_appName; }
inline aColorType acpApplication::backgroundColor(void) const
{ return m_backgroundColor; }
inline bool acpApplication::done(void) const 
{ return m_bDone; }
inline unsigned long acpApplication::attributes(void) const
{ return m_attributes; }


/////////////////////////////////////////////////////////////////////

class acpDialogWindow;

class aUI_EXPORT acpDialog : public acpView {
public:
  acpDialog(acpApplication& application);
  virtual ~acpDialog(void);

  // acpView
  virtual void setBounds(const aRECT& r);
  virtual void show(void);
  virtual void hide(void);
  virtual void shrinkWrap(void);
protected:
  acpApplication& m_application;
private:
  acpDialogWindow* m_pDialogWindow;
  friend class unix_acpDialogWindow;
};


/////////////////////////////////////////////////////////////////////

class aUI_EXPORT acpDialogMessage : public acpDialog {
public:
  acpDialogMessage(acpApplication& application,
		   const acpString& title,
		   const acpString& message);
  virtual ~acpDialogMessage(void);

  virtual void setBounds(const aRECT& r) { acpDialog::setBounds(r); }
  virtual void show(void);
  virtual bool handleClick(acpView* pView);

private:
  acpString m_title;
  acpString m_message;
  acpControlLabel* m_pTitle;
  acpControlLabel* m_pMessage;
  acpControlButton* m_pOK;
};


/////////////////////////////////////////////////////////////////////

class aUI_EXPORT acpDialogPickFile : public acpDialog {
public:
  acpDialogPickFile(acpApplication& application,
		    const acpString& prompt,
		    const aFileArea eArea,
		    const char* extension);
  virtual ~acpDialogPickFile(void);

  //  virtual void setBounds(const aRECT& r) { acpDialog::setBounds(r); }
  virtual void show(void);
  virtual bool handleClick(acpView* pView);
  virtual bool handleSelection(acpView* pContainer);
  
private:
  acpString m_prompt;
  aFileArea m_eArea;
  acpControlLabel* m_pPrompt;
  acpControlList* m_pFileList;
  acpControlButton* m_pCancelButton;
  acpControlButton* m_pOKButton;
  static aErr sDirList(const char* pFilename,
		       const unsigned long nSize,
		       void* ref);
  int m_nFiles;
};

#endif // __cplusplus 

#endif /* _aUI_H_ */
