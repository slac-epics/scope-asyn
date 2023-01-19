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

#define mbboWfWidStr    "MBBO_WFWIDTH"  // bytes per point (1 or 2)
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

#define meas1Str        "MEAS1"         // Measurement value
#define meas2Str        "MEAS2"         // Measurement value
#define meas3Str        "MEAS3"         // Measurement value
#define meas4Str        "MEAS4"         // Measurement value
#define meas1UnitsStr   "MEAS1_UNITS"   // Measurement units

#define meas2UnitsStr   "MEAS2_UNITS"   // Measurement units
#define meas3UnitsStr   "MEAS3_UNITS"   // Measurement units
#define meas4UnitsStr   "MEAS4_UNITS"   // Measurement units
#define meas1TypeStr    "MEAS1_TYPE"    // Measurement type
#define meas2TypeStr    "MEAS2_TYPE"    // Measurement type

#define meas3TypeStr    "MEAS3_TYPE"    // Measurement type
#define meas4TypeStr    "MEAS4_TYPE"    // Measurement type
#define meas1StateStr   "MEAS1_STATE"   // Measurement state
#define meas2StateStr   "MEAS2_STATE"   // Measurement state
#define meas3StateStr   "MEAS3_STATE"   // Measurement state

#define meas4StateStr   "MEAS4_STATE"   // Measurement state


class drvTDS: public drvScope{
public:
    drvTDS(const char* port, const char* udp);
  
    virtual asynStatus writeInt32(asynUser* pau,epicsInt32 v);
    virtual asynStatus writeFloat64(asynUser* pau,epicsFloat64 v);
    //virtual asynStatus readEnum(asynUser *pau, char *strings[], int values[], int severities[],
    //        size_t nElements, size_t *nIn);
    void          getWaveform(int ch);
    const char*   getCommand(int cix);
    const char**  getCmndList(int cix,uint* ni);
    void          getHSParams(double hs,int* x0,int* np);
    bool          isTriggered();
    bool          isRunning();
    void          postInit();

protected:
    int _mbboWfWid,     _boTrMode,      _mbboTrSou,     _boTrSlo,     _mbbiTrSta,
        _mbboChScl,     _mbboTimDivV,   _mbboTimDivU,   _biAcqStat,   _liEvQ,
        _liEvQty,       _loRecall,      _loStore,       _siSource,    _siHead,
        _meas1,         _meas2,         _meas3,         _meas4,       _meas1Units,
        _meas2Units,    _meas3Units,    _meas4Units,    _meas1Type,   _meas2Type,
        _meas3Type,     _meas4Type,     _meas1State,    _meas2State,  _meas3State,
        _meas4State;
  
    enum {ixMbboWfWid,    ixBoTrMode,     ixMbboTrSou,    ixBoTrSlo,     ixMbbiTrSta,
          ixMbboChScl,    ixMbboTimDivV,  ixMbboTimDivU,  ixBiAcqStat,   ixLiEvQ,
          ixLiEvQty,      ixLoRecall,     ixLoStore,      ixSiSource,    ixSiHead,
          ixMeas1,        ixMeas2,        ixMeas3,        ixMeas4,       ixMeas1Units,
          ixMeas2Units,   ixMeas3Units,   ixMeas4Units,   ixMeas1Type,   ixMeas2Type,
          ixMeas3Type,    ixMeas4Type,    ixMeas1State,   ixMeas2State,  ixMeas3State,
          ixMeas4State};
  
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
    void        getMeasurements(int pollCount);

private:
    void      _setTimePerDiv(uint vix,uint uix);
    int       _wfPreamble(char* p,int*,int*,double*,double*,double*);
    char      _name[NAME_LEN];
    float     _wfbuf[WF_LEN];
    float     _wfraw[WF_LEN];
    char      _rbuf[DBUF_LEN];
    int       _firstix;
    const int _num_meas;
};

#endif    // _drvTDS_h

