/* drvTDS.h
 * 
 * Asyn driver that inherits from the drvScope base class.  It is a specific
 * class for the Tekronix TDS 3000 series oscilloscopes.
 * Started on 05/25/2015, zms
 *---------------------------------------------------------------------------*/
#ifndef _drvTDS_h
#define _drvTDS_h

#include "drvScope.h"

#define WF_LEN    800

#define loWfWidStr      "LO_WFWIDTH"    // bytes per point
#define boTrModeStr     "BO_TRMODE"     // trigger mode
#define mbboTrSouStr    "MBBO_TRSOU"    // trigger source
#define boTrSlopeStr    "BO_TRSLOPE"    // trigger slope
#define mbbiTrStaStr    "MBBI_TRSTA"    // trigger state

#define mbboChSclStr    "MBBO_CHSCL"    //
#define mbboTimDivVStr  "MBBO_TIMDIVV"  
#define mbboTimDivUStr  "MBBO_TIMDIVU"  
#define biAcqStatStr    "BI_ACQSTAT"    // acquire state
#define liEvQStr        "LI_EVQ"

#define liEvQtyStr      "LI_EVQTY"      //
#define loRecallStr     "LO_RECALL"     // Recall stored configuration
#define loStoreStr      "LO_STORE"      // Save configuration
#define siSourceStr     "SI_SOURCE"     //
#define siHeadStr       "SI_HEAD"       // state of the header flag

#define aiMeas1Str      "AI_MEAS1"       // Measurement value
#define aiMeas2Str      "AI_MEAS2"       // Measurement value
#define aiMeas3Str      "AI_MEAS3"       // Measurement value
#define aiMeas4Str      "AI_MEAS4"       // Measurement value
#define siMeas1UnitsStr "SI_MEAS1_UNITS" // Measurement units

#define siMeas2UnitsStr "SI_MEAS2_UNITS" // Measurement units
#define siMeas3UnitsStr "SI_MEAS3_UNITS" // Measurement units
#define siMeas4UnitsStr "SI_MEAS4_UNITS" // Measurement units


class drvTDS: public drvScope{
public:
    drvTDS(const char* port, const char* udp);
  
    virtual asynStatus writeInt32(asynUser* pau,epicsInt32 v);
    virtual asynStatus writeFloat64(asynUser* pau,epicsFloat64 v);
    void          getWaveform(int ch);
    virtual void  getMeasurements();
    const char*   getCommand(int cix);
    const char**  getCmndList(int cix,uint* ni);
    void          getHSParams(double hs,int* x0,int* np);
    int           isTriggered();
    void          postInit();

protected:
    int _loWfWid,       _boTrMode,      _mbboTrSou,     _boTrSlo,   _mbbiTrSta,
        _mbboChScl,     _mbboTimDivV,   _mbboTimDivU,   _biAcqStat, _liEvQ,
        _liEvQty,       _loRecall,      _loStore,       _siSource,  _siHead,
        _aiMeas1,       _aiMeas2,       _aiMeas3,       _aiMeas4,   _siMeas1Units,
        _siMeas2Units,  _siMeas3Units,  _siMeas4Units;
  
    enum {ixLoWfWid,      ixBoTrMode,     ixMbboTrSou,    ixBoTrSlo,   ixMbbiTrSta,
          ixMbboChScl,    ixMbboTimDivV,  ixMbboTimDivU,  ixBiAcqStat, ixLiEvQ,
          ixLiEvQty,      ixLoRecall,     ixLoStore,      ixSiSource,  ixSiHead,
          ixAiMeas1,      ixAiMeas2,      ixAiMeas3,      ixAiMeas4,   ixSiMeas1Units,
          ixSiMeas2Units, ixSiMeas3Units, ixSiMeas4Units};
  
    asynStatus  putFltCmnds(int ix,int addr,float v);
    asynStatus  putIntCmnds(int ix,int addr,int v);
    asynStatus  getCmnds(int ix,int addr);
    void        setTimePerDiv(double v);
    void        getChanScl(int ch);
    void        timeDelayStr(int,int);
    void        getTrigLevl();
    void        setTrigLevl(int v);
    void        updateUser();
    asynStatus  trigState();

private:
    void      _setTimePerDiv(uint vix,uint uix);
    int       _wfPreamble(char* p,int*,int*,double*,double*,double*);
    char      _name[NAME_LEN];
    float     _wfbuf[WF_LEN];
    float     _wfraw[WF_LEN];
    char      _rbuf[DBUF_LEN];
    int       _firstix;
    void      _getMeasData(int meas_num, int param);
    const int _num_meas;
};

#endif    // _drvTDS_h

