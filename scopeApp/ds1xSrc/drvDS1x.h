#ifndef DRVDS1X_H
#define DRVDS1X_H

/* drvDS1x.h
 * This is a specific class for the Rigol DS4000 and DS6000 series oscilloscopes
 * which is subclassed from the drvScope base class.
 * Started on 05/27/2015, zms
 *---------------------------------------------------------------------------*/

#include "drvScope.h"

#define WF_LEN	1200
#define WFPRE	128

#define mbboTrModeStr	"MBBO_TRMODE"	// trigger mode
#define mbboTrSouStr    "MBBO_TRSOU"    // trigger source
#define mbboTrCplStr	"MBBO_TRCPL"	// trigger coupling
#define mbboTrSloStr	"MBBO_TRSLO"	// trigger slope
#define mbboTrSweStr	"MBBO_TRSWE"	// trigger sweep mode

#define siTrStaStr	"SI_TRSTA"	// trigger state
#define loTimDivVStr	"LO_TIMDIVV"	// time base mantissa
#define mbboTimDivUStr  "MBBO_TIMDIVU"	// time base units
#define boTmModeStr	"BO_TMMODE"	// time base mode
#define aoTimDivStr	"AO_TIMDIV"	// time base scale (s/div)

#define aoTmOfstStr	"AO_TMOFST"	// time base offset
#define aoTmDlyOfsStr	"AO_TMDLYOFS"	// time base delay offset (s)
#define aoTmDlySclStr	"AO_TMDLYSCL"	// time base delay scale (s/div)
#define siSRateStr	"SI_SRATE"	// Sampling rate (smpl/sec)
#define boAutoSclStr	"BO_AUTOS"	// Auto scale

#define loAcqAveStr	"LO_ACQAV"	// acquire averages
#define mbboAcqTpStr	"MBBO_ACQTP"	// acquire type
#define boAcqModeStr	"BO_ACQMODE"	// acquire mode
#define mbboWfFmtStr	"MBBO_WFFMT"	// Waveform format
#define boDumpStr	"BO_DUMP"	// for printing data dump


class drvDS1x: public drvScope{
public:
  drvDS1x(const char* port, const char* udp);

  virtual asynStatus writeInt32( asynUser* pau,epicsInt32 v);
  virtual asynStatus writeFloat64( asynUser* pau,epicsFloat64 v);
  virtual void  afterInit();
  void		getWaveform( int ch);
  const char*	getCommand( int cix);
  const char**	getCmndList( int cix,uint* ni);
  void		saveConfig();
  void		restoreConfig();
  void		getChanPos( int addr);
  void		setChanPos( int addr,double v);

protected:

  int	_mbboTrMode, _mbboTrSou,  _mbboTrCpl,   _mbboTrSlo,   _mbboTrSwe,
	_siTrSta,    _loTimDivV,  _mbboTimDivU, _boTmMode,    _aoTimDiv,
	_aoTmOfst,   _aoTmDlyOfs, _aoTmDlyScl,  _siSRate,     _boAutoScl,
	_loAcqAve,   _mbboAcqTp,  _boAcqMode,   _mbboWfFmt,   _boDump;

  enum{	ixMbboTrMode, ixMbboTrSou,  ixMbboTrCpl,  ixMbboTrSlo, ixMbboTrSwe,
	ixSiTrSta,    ixLoTimDivV,  ixMbboTimDivU,ixBoTmMode,  ixAoTimDiv,
	ixAoTmOfst,   ixAoTmDlyOfs, ixAoTmDlyScl, ixSiSRate,   ixBoAutoScl,
	ixloAcqAve,   ixMbboAcqTp,  ixBoAcqMode,  ixMbboWfFmt,  ixBoDump};

  asynStatus putFltCmnds( int ix,int addr,float v);
  asynStatus putIntCmnds( int ix,int addr,int v);
  asynStatus getCmnds( int ix,int addr);
  void setTimePerDiv( double v);
  void timeDelayStr( int m,int uix);
  void getTrigLevl();
  void setTrigLevl(int v);
  void updateUser();

private:
  int		_wfPreamble( char* p,int*,int*,double*,double*,int*);
  void		_setTimePerDiv( uint uix);
  void		_printWF( int nb,int len,int n,char* pbuf);
  void		_printWFData();
  char*		_getChanCmnds( int ch);
  char*		_getAcquCmnds();
  char*		_getTrigCmnds();
  char*		_getTimeCmnds();
  char*		_getOneCmnd( const char* cmnd);

  char		_name[NAME_LEN];
  char		_rbuf[DBUF_LEN];
  char		_wbuf[DBUF_LEN];
  float		_wfbuf[WF_LEN];
  word		_wfraw[WF_LEN];
  char		_wfpre[WFPRE];
  int		_wfprei[6];
  float		_wfpref[4];
  int		_posInProg;		// when true, positioning by slider.
  int		_initDone;		// set true when initialization is done
  int		_firstix;		// index of first item in this class
};

#endif

