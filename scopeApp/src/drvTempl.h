// drvTempl.h
// 
// Asyn driver that inherits from the asynPortDriver class.  This is an
// adaptation of testAsynPortDriver.cpp written by Mark Rivers.
// Started on 4/23/2010, zms
//-----------------------------------------------------------------------------
#include <epicsMessageQueue.h>
#include "asynPortDriver.h"

#ifndef SIZE
#define SIZE(x)         (sizeof(x)/sizeof(x[0]))
#endif
#ifndef MIN
#define MIN(a,b)        (((a)<(b))?(a):(b))
#endif
#ifndef MAX
#define MAX(a,b)        (((a)>(b))?(a):(b))
#endif
typedef unsigned char   byte;
typedef unsigned short  word;
typedef unsigned int    uint;

#define NELM		256
#define NCHAN		1
#define NMSGQ		100

typedef struct{
  int	type,
	pix,		//parameter library index
	ival;		// possible int value
  float	fval;		// possible floating point value
} msgq_t;

#define siNameStr	"SI_NAME"
#define wfMessageStr    "WF_MESSAGE"
#define liPollTmoStr	"LI_POLLTMO"
#define loPollTmoStr	"LO_POLLTMO"

class drvTempl : public asynPortDriver{
public:
  drvTempl(const char* port,const char* udp,int addr);

  virtual asynStatus readInt32( asynUser* pau,epicsInt32* v);
  virtual asynStatus readInt16Array( asynUser* pau,epicsInt16* v,
                        size_t nelm,size_t* nin);
  virtual asynStatus readFloat64( asynUser* pau,epicsFloat64* v);
  virtual asynStatus writeInt8Array( asynUser* pau,epicsInt8* v,size_t n);
  virtual asynStatus writeInt16Array( asynUser* pau,epicsInt16* v,size_t n);
  virtual asynStatus writeInt32( asynUser* pau,epicsInt32 v);
  virtual asynStatus writeFloat64( asynUser* pau,epicsFloat64 v);
  void exitHndl();
  void afterInit();
  void taskThread();

protected:
  char*		_getTime();
  asynStatus    _write( const char* pw,size_t nw);
  asynStatus    _writeRead( const char* pw,size_t nw,char* pr,size_t nr,
                                size_t* nbo,size_t* nbi);
  asynStatus    _wtrd( const char* cmnd);
  void		_drvInit();
  void		_message( char*);

  int	_siName,   _wfMessage,
	_liPollTmo,_loPollTmo;

#define FIRST_ITEM _siName
#define LAST_ITEM  _loPollTmo
#define N_PARAMS (&LAST_ITEM - &FIRST_ITEM + 1)

enum{	ixSiName,   ixWfMessage,
	ixLiPollTmo,ixLoPollTmo};

private:
  epicsMessageQueue* _pmq;
  asynUser*     _pvt;
  const char*	_port;
  const char*   _udp;
  ELLLIST	_alist;
  int		_isinit;
  double        _timeout;	// sleep period in sec for taskThread
  char		_buf[NELM];
  int		_firstix;
  int           _adrs;
};
