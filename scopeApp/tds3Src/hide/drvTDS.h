/* drvTDS.h
 * 
 * Asyn driver that inherits from the asynPortDriver class.  This is an
 * adaptation of testAsynPortDriver.cpp written by Mark Rivers.
 * Started on 11/11/2010, zms
 *---------------------------------------------------------------------------*/
#ifndef _drvTDS_h
#define _drvTDS_h
#include <epicsMessageQueue.h>
#include "utils.h"

#ifndef SIZE
#define SIZE(x)         (sizeof(x)/sizeof(x[0]))
#endif
#ifndef MIN
#define MIN(a,b)        (((a)<(b))?(a):(b))
#endif
#ifndef MAX
#define MAX(a,b)        (((a)>(b))?(a):(b))
#endif

#define MAX_ADDR	4
#define NCHAN		(MAX_ADDR)
#define MSGNB		100
#define NMSGQ           400
#define MSGQNB          20
#define NAME_LEN	23
#define CMND_LEN	32
#define N_CMNDS		80
#define DBUF_LEN	10240
#define WF_LEN		800

#define POLLWF		0x01
#define POLLCTRLS	0x02

typedef enum{ enTMNone,enTMASync,enTMSync} tracem_e;
typedef enum{ enPutInt,enPutFlt,enQuery} ctype_e;
typedef struct{
  int   type,           // one of ctype_e (type of command processing)
        ix,             // parameter library index
        addr,           // channel or address
        ival;           // possible integer set value
  float fval;           // possible floating point value
} msgq_t;

#define siNameStr	"SI_NAME"
#define biEnablStr	"BI_ENABLE"	// channel enable
#define boEnablStr	"BO_ENABLE"
#define aiPositStr	"AI_POSITION"	// 
#define aoPositStr	"AO_POSITION"

#define aiOffstStr	"AI_OFFSET"	// 
#define aoOffstStr	"AO_OFFSET"
#define biImpedStr	"BI_IMPED"	// 
#define boImpedStr	"BO_IMPED"	// 
#define mbbiCouplStr	"MBBI_COUPL"

#define mbboCouplStr	"MBBO_COUPL"
#define aiVDivStr	"AI_VDIV"	// 
#define mbboSclStr	"MBBO_SCL"	// 
#define mbbiSclStr	"MBBI_SCL"
#define wfTraceStr	"WF_TRACE"

#define biCtGetsStr	"BI_CTGETS"	// 
#define boGetWfStr	"BO_GETWF"	// 
#define boGetWfAStr	"BO_GETWFALL"	// 
#define boUpdtStr	"BO_UPDT"
#define aiTDlyStr	"AI_TDELAY"	// horizontal time delay

#define aoTDlyStr	"AO_TDELAY"
#define siTDlyStr	"SI_TDELAY"
#define biTDlyStStr	"BI_TDLYST"	// horizontal time delay state
#define boTDlyStStr	"BO_TDLYST"
#define aiTDivStr	"AI_TDIV"	// 

#define siTDivStr	"SI_TDIV"
#define aoCTDivStr	"AO_CTDIV"	// 
#define mbbiTDivVStr	"MBBI_TDIVV"	// 
#define mbboTDivVStr	"MBBO_TDIVV"
#define mbbiTDivUStr	"MBBI_TDIVU"	// 

#define mbboTDivUStr	"MBBO_TDIVU"
#define aiTRefStr	"AI_TREF"	// 
#define aoTRefStr	"AO_TREF"
#define aiTrigLevStr	"AI_TRIGLEV"	// 
#define aoTrigLevStr	"AO_TRIGLEV"

#define aiHldOffStr	"AI_HOLDOFF"	// 
#define aoHldOffStr	"AO_HOLDOFF"
#define biTModeStr	"BI_TRIGMODE"	// 
#define boTModeStr	"BO_TRIGMODE"
#define mbbiTrigSoStr	"MBBI_TRIGSO"	// 

#define mbboTrigSoStr	"MBBO_TRIGSO"
#define biTrigSloStr	"BI_TRIGSLO"	// 
#define boTrigSloStr	"BO_TRIGSLO"
#define biAcqStatStr	"BI_ACQSTAT"	// 
#define mbbiTrigStaStr	"MBBI_TRIGSTATE"	// 

#define boRunStr	"BO_RUN"	// 
#define boStopStr	"BO_STOP"	// 
#define biFLockStr	"BI_FPLOCK"	// 
#define boFLockStr	"BO_FPLOCK"
#define biBusyStr	"BI_Busy"	// 

#define boResetStr	"BO_RESET"	// 
#define wfSetupStr	"WF_SETUP"	// 
#define wfWSetupStr	"WF_WSETUP"	// 
#define loRecallStr	"LO_RECALL"	// 
#define loStoreStr	"LO_STORE"	// 

#define siDSourceStr	"SI_SOURCE"	// 
#define biDAutoCStr	"BI_DAUTOC"	// 
#define boDAutoCStr	"BO_DAUTOC"
#define liDBrightStr	"LI_DBRIGHT"	// 
#define loDBrightStr	"LO_DBRIGHT"

#define biDPictStStr	"BI_DPICTST"	// 
#define boDPictStStr	"BO_DPICTST"
#define mbbiDIntensStr	"MBBI_DINTENS"	// 
#define mbboDIntensStr	"MBBO_DINTENS"
#define liIntensWfStr	"LI_INTENSWF"	// 

#define loIntensWfStr	"LO_INTENSWF"
#define boInitStr	"BO_INIT"	// 
#define mbbiDEncodStr	"MBBI_DENCODE"	// 
#define mbboDEncodStr	"MBBO_DENCODE"
#define siDEncodStr	"SI_DENCODE"

#define liRecLenStr	"LI_RECLEN"	// 
#define loRecLenStr	"LO_RECLEN"
#define siHeaderStr	"SI_HEADER"	// 
#define liDWidthStr	"LI_DWIDTH"	// 
#define loDWidthStr	"LO_DWIDTH"

#define liDStartStr	"LI_DSTART"	// 
#define loDStartStr	"LO_DSTART"
#define liDStopStr	"LI_DSTOP"	// 
#define loDStopStr	"LO_DSTOP"
#define wfIdnStr	"WF_IDN"	// 

#define siIpNameStr	"SI_IPNAME"	// 
#define soCmndStr	"SO_CMND"	// 
#define wfReplyStr	"WF_REPLY"	// 
#define aoPTMOStr	"AO_PTMO"	// 
#define loPCtrlStr	"LO_PCTRL"	// 

#define boAnalStr	"BO_ANAL"
#define biAnalStr	"BI_ANAL"
#define boAPedStr	"BO_PED"
#define aiAreaStr	"AI_AREA"
#define aiPedStr	"AI_PED"

#define mbboMChanStr	"MBBO_MCHAN"
#define loMark1Str	"LO_MARK1"
#define loMark2Str	"LO_MARK2"
#define liMark1Str	"LI_MARK1"
#define liMark2Str	"LI_MARK2"

#define liDeseStr	"LI_DESE"
#define loDeseStr	"LO_DESE"
#define liEsrStr	"LI_ESR"
#define liEseStr	"LI_ESE"
#define loEseStr	"LO_ESE"

#define liEvQStr	"LI_EVQ"
#define boClsStr	"BO_CLS"	// 
#define siOPCStr	"SI_OPC"	// 
#define liStbStr	"LI_STB"	// 
#define wfEventStr	"WF_EVENT"	// 

#define wfMessgStr	"WF_MESSG"	// 
#define biChSelStr	"BI_SELECT"
#define boChSelStr	"BO_SELECT"
#define liChPosStr	"LI_CHPOS"
#define loChPosStr	"LO_CHPOS"

#define liTLevlStr	"LI_TLEVL"
#define loTLevlStr	"LO_TLEVL"
#define liMQSuccsStr	"LI_MSGQS"
#define liMQFailStr	"LI_MSGQF"
#define mbbiTracModStr	"MBBI_TRCMOD"

#define mbboTracModStr	"MBBO_TRCMOD"
#define liXNptsStr	"LI_XNPTS"
#define biStateStr	"BI_STATE"
#define boRdTracesStr	"BO_RDTRACE"	// when true, read traces from scope
#define liEvQtyStr	"LI_EVQTY"	// 

typedef unsigned char	byte;
typedef unsigned short	word;
typedef unsigned int	uint;

typedef struct{ int pix; const char* pcmd;} cmnds_t;

class drvTDS : public asynPortDriver {
public:
  friend class Utils;
  drvTDS(const char* port,const char* udp);

  virtual asynStatus writeOctet( asynUser* paU,const char* val,size_t nc,
		size_t* nActual);
  virtual asynStatus writeInt32( asynUser* pau,epicsInt32 v);
  virtual asynStatus writeFloat64( asynUser* pau,epicsFloat64 v);
  void	pollerThread();
  void	afterInit();

protected:

  int	_siName,     _biEnabl,    _boEnabl,    _aiPosit,    _aoPosit,
	_aiOffst,    _aoOffst,    _biImped,    _boImped,    _mbbiCoupl,
	_mbboCoupl,  _aiVDiv,     _mbboScl,    _mbbiScl,    _wfTrace,
	_biCtGets,   _boGetWf,    _boGetWfA,   _boUpdt,     _aiTDly,
	_aoTDly,     _siTDly,     _biTDlySt,   _boTDlySt,   _aiTDiv,
	_siTDiv,     _aoCTDiv,    _mbbiTDivV,  _mbboTDivV,  _mbbiTDivU,
	_mbboTDivU,  _aiTRef,     _aoTRef,     _aiTrigLev,  _aoTrigLev,
	_aiHldOff,   _aoHldOff,   _biTMode,    _boTMode,    _mbbiTrigSo,
	_mbboTrigSo, _biTrigSlo,  _boTrigSlo,  _biAcqStat,  _mbbiTrigSta,
	_boRun,      _boStop,     _biFLock,    _boFLock,    _biBusy,
	_boReset,    _wfSetup,    _wfWSetup,   _loRecall,   _loStore,
	_siDSource,  _biDAutoC,   _boDAutoC,   _liDBright,  _loDBright,
	_biDPictSt,  _boDPictSt,  _mbbiDIntens,_mbboDIntens,_liIntensWf,
	_loIntensWf, _boInit,     _mbbiDEncod, _mbboDEncod, _siDEncod,
	_liRecLen,   _loRecLen,   _siHeader,   _liDWidth,   _loDWidth,
	_liDStart,   _loDStart,   _liDStop,    _loDStop,    _wfIdn,
	_siIpName,   _soCmnd,     _wfReply,    _aoPTMO,     _loPCtrl,
	_boAnal,     _biAnal,     _boAPed,     _aiArea,     _aiPed,
	_mbboMChan,  _loMark1,    _loMark2,    _liMark1,    _liMark2,
	_liDese,     _loDese,     _liEsr,      _liEse,      _loEse,
	_liEvQ,      _boCls,      _siOPC,      _liStb,      _wfEvent,
	_wfMessg,    _biChSel,    _boChSel,    _liChPos,    _loChPos,
	_liTLevl,    _loTLevl,    _liMsgQS,    _liMsgQF,    _mbbiTracMod,
	_mbboTracMod, _liXNpts,    _biState,   _boRdTraces, _liEvQty;

  enum{	ixSiName,     ixBiEnabl,    ixBoEnabl,    ixAiPosit,    ixAoPosit,
	ixAiOffst,    ixAoOffst,    ixBiImped,    ixBoImped,    ixMbbiCoupl,
	ixMbboCoupl,  ixAiVDiv,     ixMbboScl,    ixMbbiScl,    ixWfTrace,
	ixBiCtGets,   ixBoGetWf,    ixBoGetWfA,   ixBoUpdt,     ixAiTDly,
	ixAoTDly,     ixSiTDly,     ixBiTDlySt,   ixBoTDlySt,   ixAiTDiv,
	ixSiTDiv,     ixAoCTDiv,    ixMbbiTDivV,  ixMbboTDivV,  ixMbbiTDivU,
	ixMbboTDivU,  ixAiTRef,     ixAoTRef,     ixAiTrigLev,  ixAoTrigLev,
	ixAiHldOff,   ixAoHldOff,   ixBiTMode,    ixBoTMode,    ixMbbiTrigSo,
	ixMbboTrigSo, ixBiTrigSlo,  ixBoTrigSlo,  ixBiAcqStat,  ixMbbiTrigSta,
	ixBoRun,      ixBoStop,     ixBiFLock,    ixBoFLock,    ixBiBusy,
	ixBoReset,    ixWfSetup,    ixWfWSetup,   ixLoRecall,   ixLoStore,
	ixSiDSource,  ixBiDAutoC,   ixBoDAutoC,   ixLiDBright,  ixLoDBright,
	ixBiDPictSt,  ixBoDPictSt,  ixMbbiDIntens,ixMbboDIntens,ixLiIntensWf,
	ixLoIntensWf, ixBoInit,     ixMbbiDEncod, ixMbboDEncod, ixSiDEncod,
	ixLiRecLen,   ixLoRecLen,   ixSiHeader,   ixLiDWidth,   ixLoDWidth,
	ixLiDStart,   ixLoDStart,   ixLiDStop,    ixLoDStop,    ixWfIdn,
	ixSiIpName,   ixSoCmnd,     ixWfReply,    ixAoPTMO,     ixLoPCtrl,
	ixBoAnal,     ixBiAnal,     ixBoAPed,     ixAiArea,     ixAiPed,
	ixMbboMChan,  ixLoMark1,    ixLoMark2,    ixLiMark1,    ixLiMark2,
	ixLiDese,     ixLoDese,     ixLiEsr,      ixLiEse,      ixLoEse,
	ixLiEvQ,      ixBoCls,      ixSiOPC,      ixLiStb,      ixWfEvent,
	ixWfMessg,    ixBiChSel,    ixBoChSel,    ixLiChPos,    ixLoChPos,
	ixLiTLevl,    ixLoTLevl,    ixLiMsgQS,    ixLiMsgQF,    ixMbbiTracMod,
	ixMbboTracMod,ixLiXNpts,    ixBiState,    ixBoRdTraces, ixLiEvQty};

  #define FRST_COMMAND _siName
  #define LAST_COMMAND _liEvQty
  #define N_PARAMS (&LAST_COMMAND - &FRST_COMMAND + 1)

private:
  void		_message( const char* m);
  asynStatus	_write( const char* pw,size_t nw);
  asynStatus	_wtrd( const char* pw,size_t nw,char* pr,size_t nr);
  int           _opc();
  char*		_makeQuery( const char* cmnd);
  const char*	_getCmnd( int pix);
  void		_getIdn();
  void		_getIpAddr();
  void		_setTimePerDiv( epicsFloat64 v);
  void		_setTimePerDiv( uint vix,uint uix);
  void		_timeDelayStr( double td);
  void		_setTimeDelayStr();
  int		_wfPreamble( char* p,int*,int*,double*,double*,double*);
  void		_getWaveform( int ch);
  void		_getTraces();
  int		_find( const char* item,const char** list,int n);
  asynStatus	_getInt( const char* cmnd,int pix);
  asynStatus	_getFloat( const char* cmnd,int pix);
  asynStatus	_getBinary( const char* cmnd,int pix,const char** list,int n);
  asynStatus	_getString( const char* cmnd,int pix);
  asynStatus	_command( const char* cmnd);
  asynStatus	_getIntCh( const char* cmnd,int i,int pix);
  asynStatus	_getFloatCh( const char* cmnd,int i,int pix);
  asynStatus	_getBinaryCh( const char* cmnd,int i,
				int pix,const char** list,int n);
  void		_setBinaryCh( int ix,int ch,const char* cmnd,
				const char** list,int n);
  void		_setBinary( int ix,const char* cmnd,const char** list,int n);
  void		_setInt( const char* cmnd,int v,int pix=0);
  void		_getUpdate();
  void		_doUpdate();
  void		_update();
  void		_setPosSlider( double v);
  void		_selectChannel();
  void		_selectChan( int ch);
  void		_getTrigLevel();
  void		_setTrigLevel( int v);
  asynStatus    _putFltCmnds( int ix,int addr,float v);
  asynStatus    _putIntCmnds( int ix,int addr,int v);
  asynStatus    _getCmnds( int ix,int addr);
  void          _putInMessgQ( int tp,int ix,int addr,int iv,float fv=0.0);
  asynUser*	_aPvt;
  const char*	_port;
  const char*	_udpp;
  Utils*	_putil;
  epicsMessageQueue* _pmq;
  int		_firstix;
  int		_ncmnds;
  double	_pollT;
  int		_pollCtrl;
  char		_name[NAME_LEN];
  char		_mbuf[MSGNB];
  char		_cmnd[CMND_LEN];
  char		_rbuf[DBUF_LEN];
  char		_wbuf[DBUF_LEN];
  float		_wfbuf[WF_LEN];
  float		_wfraw[WF_LEN];
  int		_markchan;		// selected marker channel
  int		_analize[NCHAN];	// analysis on/off flags
  int		_doPeds[NCHAN];		// flag to do pedestals
  double	_area[NCHAN];		// integrated value
  double	_pedestal[NCHAN];	// pedestal value subtracted
  int		_mix1[NCHAN];		// marker 1 for integration
  int		_mix2[NCHAN];		// marker 2 for integration
  int		_chSel;			// selected channel
  int           _mqSent;
  int           _mqFailed;
  int		_tracemode;		// 0 async, 1 sync
  int		_rdtraces;		// read traces flag
};

#endif	// _drvTDS_h
