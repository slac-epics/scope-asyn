#ifndef DRVSCOPE_H
#define DRVSCOPE_H

/* drvScope.cpp
 * Base class for oscilloscope drivers.
 * asynPortDriver --> drvScope
 *---------------------------------------------------------------------------*/

#include <epicsMessageQueue.h>
#include <epicsTimer.h>
#include "asynPortDriver.h"

#ifndef SIZE
#define SIZE(x)   (sizeof(x)/sizeof(x[0]))
#endif
#ifndef MIN
#define MIN(a,b)  (((a)<(b))?(a):(b))
#endif
#ifndef MAX
#define MAX(a,b)  (((a)>(b))?(a):(b))
#endif

#define NCHAN       4
#define MSGNB       100
#define NMSGQ       400
#define MSGQNB      20
#define NAME_LEN    23
#define CMND_LEN    32
#define DBUF_LEN    10240
#define FNAME       128

typedef unsigned char  byte;
typedef unsigned short word;
typedef unsigned int   uint;

typedef enum {enTMNone, enTMASync, enTMSync} tracem_e;
typedef enum {enPutInt, enPutFlt, enQuery} ctype_e;

typedef struct {
    int type,           // one of ctype_e (type of command processing)
        ix,             // parameter library index
        addr,           // channel or address
        ival;           // possible integer set value
    float fval;         // possible floating point value
} msgq_t;

typedef struct{
    int pix; 
    const char* pcmd;
} cmnds_t;

struct Command {
    const char* command;
    std::vector<std::string> keywords;
};

#define boChOnStr         "BO_CHON"    // (0) chan on/off
#define aoChPosStr        "AO_CHPOS"    // cha position
#define boChImpStr        "BO_CHIMP"    // cha impedance
#define mbboChCplStr      "MBBO_CHCPL"    // cha coupling
#define aoChSclStr        "AO_CHSCL"    // cha scale

#define wfTraceStr        "WF_TRACE"    // (5) get trace
#define loWfNptsStr       "LO_WFNPTS"    // Number of data points in waveform
#define loWfStartStr      "LO_WFSTART"    // waveform starting data index
#define loWfStopStr       "LO_WFSTOP"    // waveform ending data index
#define siWfFmtStr        "SI_WFFMT"    // waveform data format

#define aoTimDlyStr       "AO_TIMDLY"    // (10) time base delay
#define boTimDlyStStr     "BO_TIMDLYST"    // time delay state flag
#define aiTimDivStr       "AI_TIMDIV"    //
#define aoTrPosStr        "AO_TRPOS"    // trigger position
#define aoTrLevStr        "AO_TRLEV"    // trigger level

#define aoTrHOffStr       "AO_TRHOFF"    // (15) trigger hold off
#define boRunStr          "BO_RUN"    //  acquire run state
#define boStopStr         "BO_STOP"    // acquire stop state
#define loEseStr          "LO_ESE"
#define boClsStr          "BO_CLS"    //

#define liEsrStr          "LI_ESR"    // (20)
#define siOpcStr          "SI_OPC"    //
#define liStbStr          "LI_STB"    // Status byte (30)
#define boResetStr        "BO_RESET"    //
#define wfIdnStr          "WF_IDN"    // Identity

#define siIpAddrStr       "SI_IPADDR"    // (25)
#define boInitStr         "BO_INIT"    // trigger sending an init string
#define boSaveStr         "BO_SAVE"    // save configuration to a disk file
#define boEvMsgStr        "BO_EVMSG"    // gets event code and message
#define siTimDlyStr       "SI_TIMDLY"    //

#define siTimDivStr       "SI_TIMDIV"    // (30)

#define siNameStr         "SI_NAME"    // (31) Internal driver commands (44)
#define boGetWfStr        "BO_GETWF"    // one time get a waveform
#define boGetWfAStr       "BO_GETWFALL"    // one time get all waveforms
#define biCtGetsStr       "BI_CTGETS"    //
#define boUpdtStr         "BO_UPDT"    // update various controls

#define soCmndStr         "SO_CMND"    // (36) interactive command
#define wfReplyStr        "WF_REPLY"    // reply to interactive command
#define aoPTMOStr         "AO_PTMO"    //
#define boAnalStr         "BO_ANAL"    // analysis on/off
#define boAPedStr         "BO_PED"    // take pedestal

#define aiAreaStr         "AI_AREA"    // (41) analysis area
#define aiPedStr          "AI_PED"    // analysis pedestal
#define mbboMChanStr      "MBBO_MCHAN"    // channel number for marker use
#define loMark1Str        "LO_MARK1"    // marker for analysis
#define loMark2Str        "LO_MARK2"    // marker for analysis

#define wfEventStr        "WF_EVENT"    // (46)
#define wfMessgStr        "WF_MESSG"    //
#define boChSelStr        "BO_CHSEL"    // channel select for slider
#define loChPosStr        "LO_CHPOS"    // slider trace position
#define loTrLevStr        "LO_TRLEV"    // slider trigger level

#define liMQSuccsStr      "LI_MSGQS"    // (51) put in MsgQ suceesfull
#define liMQFailStr       "LI_MSGQF"    // put in MsgQ failed
#define mbboTracModStr    "MBBO_TRCMOD"
#define liXNptsStr        "LI_XNPTS"    // X axis number of points (GUI)
#define biStateStr        "BI_STATE"

#define boErUpdtStr       "BO_EUPDT"    // (56)
#define wfFPathStr        "WF_FPATH"    // save/restore file path
#define boRestoreStr      "BO_RESTR"    // restore config from a disk file
#define boRdTracesStr     "BO_RDTRACE"    // when true, read traces from scope
#define aiWfTimeStr       "AI_WFTIME"    // time to get all traces

#define aiWfTMinStr       "AI_WFTMIN"    // minimum time to get traces
#define aiWfTMaxStr       "AI_WFTMAX"    // max time to get traces
#define aiWfPerStr        "AI_WFPER"    // get traces period
#define aiWfRateStr       "AI_WFRATE"    // get traces rate
#define boMeasEnabledStr  "BO_MEAS_EN"  // when true, read measurements from scope


class drvScope: public asynPortDriver,
                private epicsTimerNotify {
public:
    friend class Utils;
    drvScope(const char* port, const char* udp);
    virtual ~drvScope();

    virtual asynStatus writeOctet(asynUser* pau, const char* val, size_t nc, size_t* nActual);
    virtual asynStatus writeInt32(asynUser* pau,epicsInt32 v);
    virtual asynStatus writeFloat64(asynUser* pau,epicsFloat64 v);
    void pollerThread();
    void setChanPosition();
    virtual const char*  getCommand(int ix) {return NULL;}
    virtual const char** getCmndList(int cix, uint* ni) {*ni = 0; return NULL;}
    virtual const std::vector<std::string> getKeywordList(int cix) const {return std::vector<std::string>();}
    virtual void afterInit() = 0;
    virtual void getWaveform(int ch) = 0;
    virtual void getHSParams(double hs, int* x0, int* np) {*x0 = 0; *np = 500;}
    virtual void getChanPos(int addr);
    virtual void setChanPos(int addr, double v);
    virtual void saveConfig();
    virtual void restoreConfig();
    virtual bool isTriggered() {return true;}
    virtual bool isRunning() {return true;}
    epicsTimerNotify::expireStatus expire(const epicsTime&) {setChanPosition(); return noRestart;}

protected:
    int _boChOn,     _aoChPos,    _boChImp,    _mbboChCpl,  _aoChScl,
        _wfTrace,    _loWfNpts,   _loWfStart,  _loWfStop,   _siWfFmt,
        _aoTimDly,   _boTimDlySt, _aiTimDiv,   _aoTrPos,    _aoTrLev,
        _aoTrHOff,   _boRun,      _boStop,     _loEse,      _boCls,
        _liEsr,      _siOpc,      _liStb,      _boReset,    _wfIdn,
        _siIpAddr,   _boInit,     _boSave,     _boEvMsg,    _siTimDly,
        _siTimDiv,
        _siName,     _boGetWf,    _boGetWfA,   _biCtGets,   _boUpdt,
        _soCmnd,     _wfReply,    _aoPTMO,     _boAnal,     _boAPed,
        _aiArea,     _aiPed,      _mbboMChan,  _loMark1,    _loMark2,
        _wfEvent,    _wfMessg,    _boChSel,    _loChPos,    _loTrLev,
        _liMsgQS,    _liMsgQF,    _mbboTracMod,_liXNpts,    _biState,
        _boErUpdt,   _wfFPath,    _boRestore,  _boRdTraces, _aiWfTime,
        _aiWfTMin,   _aiWfTMax,   _aiWfPeriod, _aiWfRate, _boMeasEnabled;

    enum {ixBoChOn,     ixAoChPos,    ixBoChImp,    ixMbboChCpl,  ixAoChScl,
         ixWfTrace,    ixLoWfNpts,   ixLoWfStart,  ixLoWfStop,   ixSiWfFmt,
         ixAoTimDly,   ixBoTimDlySt, ixAiTimDiv,   ixAoTrPos,    ixAoTrLev,
         ixAoTrHOff,   ixBoRun,      ixBoStop,     ixLoEse,      ixBoCls,
         ixLiEsr,      ixSiOPC,      ixLiStb,      ixBoReset,    ixWfIdn,
         ixSiIpAddr,   ixBoInit,     ixBoSave,     ixBoEvMsg,    ixSiTimDly,
         ixSiTimDiv,
         ixSiName,     ixBoGetWf,    ixBoGetWfA,   ixBiCtGets,   ixBoUpdt,
         ixSoCmnd,     ixWfReply,    ixAoPTMO,     ixBoAnal,     ixBoAPed,
         ixAiArea,     ixAiPed,      ixMbboMChan,  ixLoMark1,    ixLoMark2,
         ixWfEvent,    ixWfMessg,    ixBoChSel,    ixLoChPos,    ixLoTrLev,
         ixLiMsgQS,    ixLiMsgQF,    ixMbboTracMod,ixLiXNpts,    ixBiState,
         ixBoErUpdt,   ixWfFPath,    ixBoRestore,  ixBoRdTraces, ixAiWfTime,
         ixAiWfTMin,   ixAiWfTMax,   ixAiWfPeriod, ixAiWfRate,   ixBoMeasEnabled};

    virtual asynStatus putFltCmnds(int ix, int addr, float v);
    virtual asynStatus putIntCmnds(int ix, int addr, int v);
    virtual asynStatus getCmnds(int ix, int addr);
    virtual void setTimePerDiv(double v) = 0;
    virtual void getChanScl(int ch) {};
    virtual void getTrigLevl() = 0;
    virtual void setTrigLevl(int v) = 0;
    virtual void timeDelayStr(int m, int uix) = 0;
    virtual void updateUser() {};
    virtual void getMeasurements(int pollCount) {};

    void          putInMessgQ(int tp, int ix, int addr, int iv, float fv=0.0);
    void          message(const std::string msg);
    asynStatus    writeRd(int cix, int ch, char* buf, int blen);
    asynStatus    writeRd(const char* cmnd, char* buf, int blen);
    asynStatus    command(const char* cmnd);
    asynStatus    command(const char* cmnd, char* prd, int len);
    asynStatus    getInt(int cix, int pix, int ch=0);
    asynStatus    getInt(const char* cmnd, int pix, int ch=0);
    void          setInt(int cix, int v, int pix=0);
    void          setInt(int cix, const char* cmnd, int v, int pix=0);
    asynStatus    getFloat(int cix, int pix, int ch=0);
    asynStatus    getFloat(const char* cmnd, int pix, int ch=0);
    virtual asynStatus getEnum(int cix, int pix, int ch=0);
    virtual asynStatus getEnum(const char* cmnd, int pix, const char** list, int ni, int ch=0);
    virtual asynStatus getEnum(const char* cmnd, int pix, std::vector<std::string> list, int ch=0);
    virtual void setEnum(int cix, int val, int ch=0);
    virtual void setEnum(const char* cmnd, int val, const char** list, int ni, int ch=0);
    virtual void setEnum(const char* cmnd, int val, std::vector<std::string> list, int ch=0);
    asynStatus    getString(int cix, int pix);
    asynStatus    getString(const char* cmnd, int pix);
    void          timeDelayStr(float td);
    void          update();

    asynUser*     pasynUser;
    int           _analize[NCHAN];    // analysis on/off flags
    int           _doPeds[NCHAN];        // flag to do pedestals
    double        _area[NCHAN];        // integrated value
    double        _pedestal[NCHAN];    // pedestal value subtracted
    int           _mix1[NCHAN];        // marker 1 for integration
    int           _mix2[NCHAN];        // marker 2 for integration
    char          _fname[FNAME];        // file path for save/restore

private:
    epicsMessageQueue* _pmq;
    void          _evMessage();
    asynStatus    _write(const char* pw, size_t nw);
    asynStatus    _wtrd(const char* pw, size_t nw, char* pr, size_t nr);
    int           _opc();
    char*         _makeQuery(const char* cmnd);
    const char*   _getCmnd(int pix);
    void          _getIdn();
    void          _getIpAddr();
    void          _setTimeDelayStr(float v);
    void          _getTraces();
    int           _find(const char* item, const char** list, int n);
    int           _find(const char* item, std::vector<std::string> list);
    void          _errUpdate();
    void          _getChanOn(int ch);
    void          _setPosSlider(double v);
    void          _selectChannel();
    void          _selectChan(int ch);
    void          _getTrigLevel();
    void          _setTrigLevel(int v);

    const char*   _port;
    const char*   _udpp;
    int           _ncmnds;
    double        _pollT;
    char          _cmnd[CMND_LEN];
    char          _rbuf[DBUF_LEN];
    char          _wbuf[DBUF_LEN];
    int           _firstix;
    int           _markchan;        // selected marker channel
    int           _chSel;            // selected channel
    double        _chPos;        // trace position from slider
    int           _mqSent;
    int           _mqFailed;
    int           _tracemode;        // 0 async, 1 sync
    int           _rdtraces;        // read traces flag
    int           _posInProg;        // when true position slider moving
    double        _wfTime;
    double        _wfTMin;
    double        _wfTMax;
    double        _wfPeriod;
    double        _wfRate;
    int           _measEnabled;
    int           _pollCount;
    long          _err_count;
    epicsTimerQueueActive* _timerQueue;
    epicsTimer*   _chPosTimer;
};

#endif    // DRVSCOPE_H

