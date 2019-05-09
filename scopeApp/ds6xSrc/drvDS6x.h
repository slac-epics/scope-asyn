/* drvDS6x.h
 * This is a specific class for the Rigol DS4000 and DS6000 series oscilloscopes
 * which is subclassed from the drvScope base class.
 * Started on 05/27/2015, zms
 *---------------------------------------------------------------------------*/
#ifndef _drvDS6x_h
#define _drvDS6x_h
#include "drvScope.h"

#define WF_LEN	1500

#define mbboTrModeStr	"MBBO_TRMODE"	// trigger mode
#define mbboTrSouStr    "MBBO_TRSOU"    // trigger source
#define mbboTrCplStr	"MBBO_TRCPL"	// trigger coupling
#define mbboTrSloStr	"MBBO_TRSLO"	// trigger slope
#define loTimDivVStr	"LO_TIMDIVV"	// time base mantissa

#define mbboTimDivUStr  "MBBO_TIMDIVU"	// time base units
#define loSreStr	"LO_SRE"	// SRE register
#define mbboTrSweStr	"MBBO_TRSWE"	// trigger sweep mode
#define mbboTmHModeStr	"MBBO_TMHMODE"	// time horizontal reference mode
#define mbboTmModeStr	"MBBO_TMMODE"	// time base mode

#define aoTimDivStr	"AO_TIMDIV"	// time base scale (s/div)
#define aoTmOfstStr	"AO_TMOFST"	// time base delay scale (s/div)
#define aoTmDlySclStr	"AO_TMDLYSCL"	// time base delay scale (s/div)
#define boTmXY1Str	"BO_TMXY1"	// XY mode ch 1 and 2
#define boTmXY2Str	"BO_TMXY2"	// XY mode ch 3 and 4

#define siTrStaStr	"SI_TRSTA"	// trigger state
#define siSRateStr	"SI_SRATE"	// Sampling rate (smpl/sec)
#define boAutoSclStr	"BO_AUTOS"	// Auto scale
#define boWfFmtStr	"BO_WFFMT"	// Waveform format
#define loAcqAveStr	"LO_ACQAV"	// acquire averages

#define mbboAcqTpStr	"MBBO_ACQTP"	// acquire type
#define NDS6_PARAM	21


class drvDS6x: public drvScope{
public:
  drvDS6x(const char* port,const char* udp,int np);

  virtual asynStatus writeInt32( asynUser* pau,epicsInt32 v);
  virtual asynStatus writeFloat64( asynUser* pau,epicsFloat64 v);
  void		getWaveform( int ch);
  const char*	getCommand( int cix);
  const char**	getCmndList( int cix,uint* ni);
  void		saveConfig();
  void		restoreConfig();
  void		getChanPos( int addr);
  void		setChanPos( int addr,double v);
  void		postInit();

protected:

  int	_mbboTrMode, _mbboTrSou, _mbboTrCpl, _mbboTrSlo,   _loTimDivV,
	_mbboTimDivU,_loSre,     _mbboTrSwe, _mbboTmHMode, _mbboTmMode,
	_aoTimDiv,   _aoTmOfst,  _aoTmDlyScl,_boTmXY1,     _boTmXY2,
	_siTrSta,    _siSRate,   _boAutoScl, _boWfFmt,     _loAcqAve,
	_mbboAcqTp;

  enum{	ixMbboTrMode, ixMbboTrSou, ixMbboTrCpl, ixMbboTrSlo,  ixLoTimDivV,
	ixMbboTimDivU,ixLoSre,     ixMbboTrSwe, ixMbboTmHMode,ixMbboTmMode,
	ixAoTimDiv,   ixAoTmOfst,  ixAoTmDlyScl,ixBoTmXY1,    ixBoTmXY2,
	ixSiTrSta,    ixSiSRate,   ixBoAutoScl, ixBoWfFmt,    ixloAcqAve,
	ixMbboAcqTp};

  asynStatus putFltCmnds( int ix,int addr,float v);
  asynStatus putIntCmnds( int ix,int addr,int v);
  asynStatus getCmnds( int ix,int addr);
  void setTimePerDiv( double v);
  void timeDelayStr( int m,int uix);
  void getTrigLevl();
  void setTrigLevl(int v);
  void updateUser();

private:
  int		_wfPreamble( char* p,int*,int*,double*,double*,double*);
  void		_setTimePerDiv( uint uix);
  void		_printWF( int nb,int len,int n,char* pbuf,byte* p);
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
  int		_posInProg;		// when true, positioning by slider.
  int		_initDone;		// set true when initialization is done
  int		_firstix;		// index of first item in this class
};

#endif	// _drvDS6x_h
