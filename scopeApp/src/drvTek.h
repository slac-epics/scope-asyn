#ifndef DRVTEK_H
#define DRVTEK_H

/* drvTek.h
 * Tektronix scope drivers should inherit from this class.
 * asynPortDriver --> drvScope --> drvTek
 *---------------------------------------------------------------------------*/

#include "drvScope.h"

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


//struct Command {
//    const char* command;
//    std::vector<std::string> keywords;
//};

struct HorScale {
    int nanos;
    int start;
    int num_pts;
};


class drvTek: public drvScope {
public:
    drvTek(const char* port, const char* udp);
    virtual ~drvTek(){}
  
    virtual asynStatus writeInt32(asynUser* pau,epicsInt32 v);
    virtual asynStatus writeFloat64(asynUser* pau,epicsFloat64 v);
    virtual void afterInit();
    virtual void getWaveform(int ch);
    virtual const char* getCommand(int cix);
    virtual const std::vector<std::string> getKeywordList(int cix) const;
    virtual void getHSParams(double hs, int* x0, int* np);
    virtual bool isTriggered();
    virtual bool isRunning();

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
  
    virtual asynStatus putFltCmnds(int ix, int addr, float v);
    virtual asynStatus putIntCmnds(int ix, int addr, int v);
    virtual asynStatus getCmnds(int ix, int addr);
    using drvScope::getEnum;  // So the overloaded functions aren't hidden
    virtual asynStatus getEnum(int cix, int pix, int ch=0);
    using drvScope::setEnum;  // So the overloaded functions aren't hidden
    virtual void setEnum(int cix, int val, int ch=0);
    virtual void setTimePerDiv(double v);
    virtual void getChanScl(int ch);
    virtual void timeDelayStr(int, int);
    virtual void getTrigLevl();
    virtual void setTrigLevl(int v);
    virtual void updateUser();
    virtual void getMeasurements(int pollCount);
    virtual int _parseWfPreamble(const char* buf, int*, int*, double*, double*, double*) = 0;

    // List of commands and corresponding keywords lists that we implement.
    // The order must agree with the order of enumerated names defined in the drvScope.h header file,
    // where the first item is ixBoChOn, the second is ixAoChPos, and so on.
    // This should be initialized in derived classes.
    std::vector<Command> commands;

    // Horizontal scale parameters
    // This should be initialized in derived classes.
    std::vector<HorScale> horScaleParams;

    // Supported commands
    // These should be initialized in derived classes.
    const char* ChOnCmnd;
    const char* ChPosCmnd;
    const char* ChImpCmnd;
    const char* ChCplCmnd;
    const char* ChSclCmnd;
    const char* WfDatCmnd;
    const char* WfNptCmnd;
    const char* WfWidCmnd;
    const char* WfStrtCmnd;
    const char* WfStopCmnd;
    const char* WfFormCmnd;
    const char* TmDlyOfsCmnd;
    const char* TmDlyEnCmnd;
    const char* TmSclCmnd;
    const char* TrigPosCmnd;
    const char* TrigLevCmnd;
    const char* TrigHoldCmnd;
    const char* TrigModeCmnd;
    const char* TrigSouCmnd;
    const char* TrigSloCmnd;
    const char* AcqStateCmnd;
    const char* TrigStaCmnd;
    const char* RunCmnd;
    const char* StopCmnd;
    const char* EseCmnd;
    const char* ClsCmnd;
    const char* EsrCmnd;
    const char* EvqCmnd;
    const char* OpcCmnd;
    const char* StbCmnd;
    const char* ResetCmnd;
    const char* RecallCmnd;
    const char* SaveCmnd;
    const char* IdnCmnd;
    const char* IPAddrCmnd;
    const char* WfSouCmnd;
    const char* InitCmnd;
    const char* HeaderCmnd;
    const char* GetConfCmnd;
    const char* ErrMsgCmnd;
    const char* MeasValCmnd;
    const char* MeasUnitsCmnd;
    const char* MeasTypeCmnd;
    const char* MeasStateCmnd;

    // Keyword lists for commands which return specific strings.
    // These should be initialized in derived classes.
    std::vector<std::string> chanImp;
    std::vector<std::string> chanCpl;
    std::vector<std::string> chanScl;
    std::vector<std::string> timDivV;
    std::vector<std::string> timDivU;
    std::vector<std::string> trgMode;
    std::vector<std::string> trigSou;
    std::vector<std::string> trigSlo;
    std::vector<std::string> trigSta;
    std::vector<std::string> dataFmt;
    std::vector<std::string> measType;

private:
    asynStatus _get_trig_state();
    void      _get_hs_params(double hs, int* x0, int* np);
    void      _setTimePerDiv(uint vix, uint uix);
    int       _firstix;
    const int _max_wf_length;
    const int _num_meas;
    const int _trigd_enum_val;  // Enum val for the TRIGGERed state
};

#endif

