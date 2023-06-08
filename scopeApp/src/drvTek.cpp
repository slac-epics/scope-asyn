/* drvTek.cpp
 * Tektronix scope drivers should inherit from this class.
 * asynPortDriver --> drvScope --> drvTek
 *---------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <algorithm>    // std::find

#include <cantProceed.h>
#include <epicsExport.h>
#include <initHooks.h>
#include <iocsh.h>

#include "drvTek.h"

namespace {
    const std::string driverName = "drvTek";
}


drvTek::drvTek(const char* port, const char* udp):
        drvScope(port, udp),
        _max_wf_length(1000),
        _num_meas(4),
        _trigd_enum_val(4) {
/*------------------------------------------------------------------------------
 * Constructor for the drvTek class. Calls constructor for the drvScope class.
 *  port The name of the asyn port driver to be created.
 *  udp  The I/O port.
 *---------------------------------------------------------------------------*/
    createParam(mbboWfWidStr,     asynParamInt32,         &_mbboWfWid);
    createParam(boTrModeStr,      asynParamInt32,         &_boTrMode);
    createParam(mbboTrSouStr,     asynParamInt32,         &_mbboTrSou);
    createParam(boTrSlopeStr,     asynParamInt32,         &_boTrSlo);
    createParam(mbbiTrStaStr,     asynParamInt32,         &_mbbiTrSta);
  
    createParam(mbboChSclStr,     asynParamInt32,         &_mbboChScl);
    createParam(mbboTimDivVStr,   asynParamInt32,         &_mbboTimDivV);
    createParam(mbboTimDivUStr,   asynParamInt32,         &_mbboTimDivU);
    createParam(biAcqStatStr,     asynParamInt32,         &_biAcqStat);
    createParam(liEvQStr,         asynParamInt32,         &_liEvQ);
  
    createParam(liEvQtyStr,       asynParamInt32,         &_liEvQty);
    createParam(loRecallStr,      asynParamInt32,         &_loRecall);
    createParam(loStoreStr,       asynParamInt32,         &_loStore);
    createParam(siSourceStr,      asynParamOctet,         &_siSource);
    createParam(siHeadStr,        asynParamOctet,         &_siHead);
  
    createParam(meas1Str,         asynParamFloat64,       &_meas1);
    createParam(meas2Str,         asynParamFloat64,       &_meas2);
    createParam(meas3Str,         asynParamFloat64,       &_meas3);
    createParam(meas4Str,         asynParamFloat64,       &_meas4);
    createParam(meas1UnitsStr,    asynParamOctet,         &_meas1Units);

    createParam(meas2UnitsStr,    asynParamOctet,         &_meas2Units);
    createParam(meas3UnitsStr,    asynParamOctet,         &_meas3Units);
    createParam(meas4UnitsStr,    asynParamOctet,         &_meas4Units);
    createParam(meas1TypeStr,     asynParamInt32,         &_meas1Type);
    createParam(meas2TypeStr,     asynParamInt32,         &_meas2Type);

    createParam(meas3TypeStr,     asynParamInt32,         &_meas3Type);
    createParam(meas4TypeStr,     asynParamInt32,         &_meas4Type);
    createParam(meas1StateStr,    asynParamInt32,         &_meas1State);
    createParam(meas2StateStr,    asynParamInt32,         &_meas2State);
    createParam(meas3StateStr,    asynParamInt32,         &_meas3State);

    createParam(meas4StateStr,    asynParamInt32,         &_meas4State);

    _firstix=_mbboWfWid;

    setStringParam(_siName, driverName);
    setIntegerParam(_loStore, 1);
    setIntegerParam(_loRecall, 1);
    for (int i=0; i<_num_meas; i++) {
        setDoubleParam(_meas1+i, 0);
        setStringParam(_meas1Units+i, "");
        setIntegerParam(_meas1Type+i, 0);
        setIntegerParam(_meas1State+i, 0);
    }
    callParamCallbacks(0);
}


void drvTek::afterInit() {
/*-----------------------------------------------------------------------------
 * After IOC init.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "afterInit";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s\n",
            driverName.c_str(), functionName.c_str());

    putInMessgQ(enQuery, _mbboTrSou,0,0);
    putInMessgQ(enQuery, _boTrSlo,0,0);
    putInMessgQ(enQuery, _mbbiTrSta,0,0);
    putInMessgQ(enQuery, _boTrMode,0,0);
    putInMessgQ(enQuery, _siSource,0,0);
    putInMessgQ(enQuery, _siHead,0,0);
    putInMessgQ(enQuery, _mbboWfWid,0,0);
    putInMessgQ(enQuery, _liEvQ,0,0);
    putInMessgQ(enQuery, _biAcqStat,0,0);
    for (int i=0; i<_num_meas; i++) {
        putInMessgQ(enQuery, _meas1State+i,0,0);
    }
    drvScope::afterInit();
}


void drvTek::updateUser(){
/*-----------------------------------------------------------------------------
 * This is a re-implementation of a virtual function in the base class.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "updateUser";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s\n",
            driverName.c_str(), functionName.c_str());

    getEnum(TrigSouCmnd, _mbboTrSou, trigSou);
    getEnum(TrigSloCmnd, _boTrSlo, trigSlo);
    getEnum(TrigStaCmnd, _mbbiTrSta, trigSta);
    getEnum(TrigModeCmnd, _boTrMode, trgMode);
}


void drvTek::_get_hs_params(double hs, int* x0, int* np) {
/*-----------------------------------------------------------------------------
 * Returns starting point in x0 and number of points in np where data should
 * be extracted from the raw waveform data.  hs is the horizontal scale in
 * seconds.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "_get_hs_params";
    int hs_nanosec = ((hs*1e9) + 0.5);
    bool found = false;
    
    for (std::vector<HorScale>::iterator it = horScaleParams.begin(); it != horScaleParams.end(); ++it) {
        if (it->nanos == hs_nanosec) {
            *x0 = it->start;
            *np = it->num_pts;
            found = true;
            break;
        }
    }

    if (!found) {
        *x0 = horScaleParams[horScaleParams.size()-1].start;
        *np = horScaleParams[horScaleParams.size()-1].num_pts;
    }

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: found=%d, hs=%E, hs_nanosec=%d, *x0=%d, *np=%d\n",
            driverName.c_str(), functionName.c_str(), found, hs, hs_nanosec, *x0, *np);
}


void drvTek::getHSParams(double hs, int* x0, int* np) {
/*-----------------------------------------------------------------------------
 *---------------------------------------------------------------------------*/
    _get_hs_params(hs, x0, np);
}


asynStatus drvTek::_get_trig_state(){
/*-----------------------------------------------------------------------------
 * Gets trigger state, which gets posted.
 *---------------------------------------------------------------------------*/
    asynStatus stat;
    stat = getEnum(TrigStaCmnd, _mbbiTrSta, trigSta);
    return stat;
}


bool drvTek::isTriggered() {
/*-----------------------------------------------------------------------------
 * Returns true if scope is in a triggered state, returns false otherwise.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "isTriggered";
    asynStatus stat;
    int trst;

    stat = _get_trig_state();
    if (stat != asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: failed in _get_trig_state\n",
                driverName.c_str(), functionName.c_str());
        return false;
    }

    getIntegerParam(0, _mbbiTrSta, &trst);
    if (trst == _trigd_enum_val) return true;
  
    return false;
}


bool drvTek::isRunning() {
/*-----------------------------------------------------------------------------
 * Returns true if scope is in the run state, returns false otherwise.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "isRunning";
    asynStatus status;
    int runState;

    status = getInt(AcqStateCmnd, _biAcqStat);
    callParamCallbacks();

    if (status != asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: failed to get run state\n",
                driverName.c_str(), functionName.c_str());
        return false;
    }
    
    getIntegerParam(_biAcqStat, &runState);
    if (runState) return true;
  
    return false;
}


const std::vector<std::string> drvTek::getKeywordList(int cix) const {
/*-----------------------------------------------------------------------------
 * Overides the empty virtual function in the base class.  It returns a pointer
 * to a list of command items choices for the cix index.  If list is not null,
 * it also returns in ni, number of items in the list.
 *---------------------------------------------------------------------------*/
    std::string functionName = "getKeywordList";

    if ((cix < 0) || (cix >= int(commands.size()))) {
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: ERROR: cix=%d\n",
                driverName.c_str(), functionName.c_str(), cix);
        return std::vector<std::string>();
    }

    return commands[cix].keywords;
}


const char* drvTek::getCommand(int cix) {
/*-----------------------------------------------------------------------------
 * Overrides the virtual function in the base class.  This function returns
 * a pointer to a command string for the index cix into the list commands.
 * Returns a null pointer if index is out of range.
 *---------------------------------------------------------------------------*/
    std::string functionName = "getCommand";

    if ((cix < 0) || (cix >= int(commands.size()))) {
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: WARNING: cix=%d out of bounds\n",
                driverName.c_str(), functionName.c_str(), cix);
        return NULL;
    }

    if (!strlen(commands[cix].command)) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s: ERROR: empty string, cix=%d\n",
                driverName.c_str(), functionName.c_str(), cix);
        return NULL;
    }

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: cix=%d, cmnd=%s\n",
            driverName.c_str(), functionName.c_str(), cix, commands[cix].command);

    return commands[cix].command;
}


void drvTek::getWaveform(int ch) {
/*-----------------------------------------------------------------------------
 * Requests waveform data for channel ch (0..3).  This gets waveform preamble
 * and waveform data.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "getWaveform";
    asynStatus stat = asynSuccess;
    int chon = 0, preamble_len = 0, wf_len = 0, wf_len_act = 0, nbyte = 0, x0 = 0, np = 0;
    double hs, pos, vdiv, ymult, yzr, yof;
    char _rbuf[DBUF_LEN];
    float _wfraw[_max_wf_length];
    float* pwr = _wfraw;
    float _wfbuf[_max_wf_length];
    float* pwf = _wfbuf;
    char* pb; 
    short* pw; 
    float ftmp; 
  
    getIntegerParam(ch, _boChOn, &chon);
    if (chon) {
        stat = writeRd(ixWfTrace, ch+1, _rbuf, DBUF_LEN);
        if (stat != asynSuccess) {
            asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: ch=%d, stat=%d, rbuf=%s\n",
                    driverName.c_str(), functionName.c_str(), ch, stat, _rbuf);
            return;
        }
        getDoubleParam(ch, _aoChPos, &pos);
        getDoubleParam(ch, _aoChScl, &vdiv);
        getDoubleParam(0, _aiTimDiv, &hs);

        preamble_len = _parseWfPreamble(_rbuf, &wf_len_act, &nbyte, &ymult, &yzr, &yof);
        if (preamble_len <= 0) {
            asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: Invalid preamble length, preamble_len=%d\n",
                    driverName.c_str(), functionName.c_str(), preamble_len);
            return;
        }

        if (vdiv < 0) vdiv = 1.0;
        pb = (&_rbuf[preamble_len]);
        pw = (short*)pb;
        _get_hs_params(hs, &x0, &np);
        wf_len = wf_len_act<_max_wf_length?wf_len_act:_max_wf_length;
        wf_len = wf_len<0?0:wf_len;
        for (int i=0, j=0; i<wf_len; i++,pb++,pw++) {
            if (nbyte == 1) {
                ftmp = (*pb);
            } else {
                ftmp = (*pw);
            }

            if ((i >= x0) && (j <= np)) {
                *pwr = (ftmp - yof)*ymult + yzr;
                *pwf = ((ftmp - yof)*ymult + yzr)/vdiv + pos;
                pwf++; 
                pwr++;
                j++;
            }
        }

        // Waveform integration, units are V*s
        if (_analize[ch]) {
            _area[ch] = 0.0;
            for (int i=_mix1[ch]; i<=_mix2[ch]; i++) {
                _area[ch] += _wfraw[i]*hs*(10./wf_len);
            }
            if (_doPeds[ch]) {
                _doPeds[ch] = 0;
                _pedestal[ch] = _area[ch];
            } else {
                _area[ch] -= _pedestal[ch];
            }
            setDoubleParam(ch, _aiArea, _area[ch]);
            setDoubleParam(ch, _aiPed, _pedestal[ch]);
            callParamCallbacks(ch);
        }
    } else {
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: ch=%d not on\n",
                driverName.c_str(), functionName.c_str(), ch);
        for (int i=0; i<_max_wf_length; i++,pwf++) {
            *pwf = 1000.0;
        }
    }
    
    doCallbacksFloat32Array(_wfbuf, _max_wf_length, _wfTrace, ch);
}


void drvTek::getMeasurements(int pollCount) {
/*-----------------------------------------------------------------------------
 *---------------------------------------------------------------------------*/
    char cmnd[32]; 

    // Get measurement data
    for (int i=0; i<_num_meas; i++) {
        sprintf(cmnd, MeasValCmnd, i+1);
        getFloat(cmnd, _meas1+i);
    }

    // Get these at a slower rate
    if (pollCount % 50 == 0) {
        for (int i=0; i<_num_meas; i++) {
            // Get measurement units
            sprintf(cmnd, MeasUnitsCmnd, i+1);
            getString(cmnd, _meas1Units+i);
            // Get measurement type
            sprintf(cmnd, MeasTypeCmnd, i+1);
            getEnum(cmnd, _meas1Type+i, measType);
            // Get measurement state (on/off)
            sprintf(cmnd, MeasStateCmnd, i+1);
            getInt(cmnd, _meas1State+i);
        }
    }
    
    callParamCallbacks();
}


void drvTek::getChanScl(int ch) {
/*-----------------------------------------------------------------------------
 * Re-implemntation of a virtual function in base class.
 *---------------------------------------------------------------------------*/
    getEnum(ChSclCmnd, _mbboChScl, chanScl, ch);
}


void drvTek::timeDelayStr(int m, int uix) {
/*-----------------------------------------------------------------------------
 * This routine is a re-implementation of a virtual in base class.  Here we
 * construct a string and push it to the db record.
 *---------------------------------------------------------------------------*/
    char str[32];
    if((uix < 0) || (uix >= int(timDivU.size()))) return;
    sprintf(str, "%d %s", m, timDivU[uix].c_str());
    setStringParam(_siTimDly, str);
}


void drvTek::getTrigLevl() {
/*-----------------------------------------------------------------------------
 * Setup slider for the trigger level value.
 *---------------------------------------------------------------------------*/
    int ch, sv, en; 
    double levl, y, scl;

    getIntegerParam(0, _mbboTrSou, &ch);
    if (ch < 0 || ch >= NCHAN) return;

    getIntegerParam(ch, _boChOn, &en);
    if (!en) return;

    getDoubleParam(0, _aoTrLev, &levl);
    getDoubleParam(ch, _aoChPos, &y);
    getDoubleParam(ch, _aoChScl, &scl);
    if ((-0.000001 < scl) && (scl < 0.000001)) scl = 1.0;
    sv = (y+levl/scl)*100;
    setIntegerParam(0, _loTrLev, sv);
}


void drvTek::setTrigLevl(int v) {
/*-----------------------------------------------------------------------------
 * trigger level request from a slider.  v is slider value.
 *---------------------------------------------------------------------------*/
    int ch;
    double levl, y ,scl;
    char cmnd[32]; 
    const char* pcmd;

    getIntegerParam(0, _mbboTrSou, &ch);
    if(ch < 0 || ch >= NCHAN) return;

    pcmd = TrigLevCmnd;
    getDoubleParam(ch, _aoChPos, &y);
    getDoubleParam(ch, _aoChScl, &scl);
    levl = (v/100.0-y)*scl;
    sprintf(cmnd, "%s %f", pcmd, levl);
    command(cmnd);
    setDoubleParam(0, _aoTrLev, levl);
}


void drvTek::setTimePerDiv(double v) {
/*-----------------------------------------------------------------------------
 * v is an analog time/div value.  This is parsed and two integer values are
 * calculated, one is "mantissa" like 1,2,4,10,20,40,100,200,400, while the
 * other is an index to the unit (ns,us,ms,s).  Both are set in the parameter
 * library for the corresponding mbbi/mbbo records.
 * Changed 1.0e-6 to 0.9e-6, etc, because if(v<1.0e-6) was returning true when
 * v=1e-6, therefore calculating wrong mantisa and exponent.
 *---------------------------------------------------------------------------*/
    int x0, mix, uix, m, np = 0;
    char str[16];
    std::vector<std::string> list = timDivU;

    if (v < 0.9e-6) {
        uix = 0; 
        m = (int)((v*1000.0)*1000000.0 + 0.5);
    } else if (v < 0.9e-3) {
        uix = 1;
        m = (int)(v*1000000.0 + 0.5);
    } else if (v < 0.9) {
        uix = 2;
        m = (int)(v*1000.0 + 0.5);
    } else {
        uix = 3;
        m = (int)(v + 0.5);
    }

    if (m <= 1) {
        mix = 0;
    } else if (m <= 2) {
        mix = 1;
    } else if (m <= 4) {
        mix = 2;
    } else if (m <= 10) {
        mix = 3;
    } else if (m <= 20) {
        mix = 4;
    } else if (m <= 40) {
        mix = 5;
    } else if (m <= 100) {
        mix = 6;
    } else if (m <= 200) {
        mix = 7;
    } else {
        mix = 8;
    }

    if (!list.empty()) {
        sprintf(str, "%d %s/div", m, list[uix].c_str());
        setIntegerParam(_mbboTimDivV, mix);
    }
    setIntegerParam(_mbboTimDivU, uix);
    setStringParam(_siTimDiv, str);
    _get_hs_params(v, &x0, &np);        // EDM needs to know how many x point
    setIntegerParam(_liXNpts, np);    // of data to dislay.
}


void drvTek::_setTimePerDiv(uint vix, uint uix) {
/*-----------------------------------------------------------------------------
 * Either of the two time per division mbbo records changed.  Indeces to the
 * value (vix) and unit (uix) are used to calculate the new analog value of
 * horizontal time per division.
 *---------------------------------------------------------------------------*/
    int m = 0;
    uint niV = timDivV.size(), niU = timDivU.size();
    double v = 0;
    char cmnd[32], str[16];
    std::vector<std::string> listV = timDivV; 
    std::vector<std::string> listU = timDivU;

    if (vix < 0 || vix >= niV || uix < 0 || uix >= niU) return;

    m = atoi(listV[vix].c_str());
    switch (uix) {
      case 0:     v = m*1.0e-9; break;
      case 1:     v = m*1.0e-6; break;
      case 2:     v = m*1.0e-3; break;
      case 3:     v = m; break;
    }

    sprintf(str, "%s %s/div", listV[vix].c_str(), listU[uix].c_str());
    setStringParam(_siTimDiv, str);
    setDoubleParam(_aiTimDiv, v);
    sprintf(cmnd, "%s %e", TmSclCmnd, v);
    command(cmnd);
    setTimePerDiv(v);
}


asynStatus drvTek::getEnum(int cix, int pix, int ch) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value and puts the obtained value in
 * parameter library at index pix.  The reply is a string and a numeric
 * value is the index in a list of strings.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "getEnum";

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: cix=%d, pix=%d, ch=%d\n",
            driverName.c_str(), functionName.c_str(), cix, pix, ch);

    const char* cmnd = getCommand(cix);
    if (!cmnd) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s::%s: Command not found\n", driverName.c_str(), functionName.c_str());
        return asynError;
    }

    const std::vector<std::string> list = getKeywordList(cix);
    if (list.empty()) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s::%s: Keyword list is empty\n", driverName.c_str(), functionName.c_str());
        return asynError;
    }

    return drvScope::getEnum(cmnd, pix, list, ch);
}


void drvTek::setEnum(int cix, int val, int ch) {
/*-----------------------------------------------------------------------------
 * ix is a bit position that was selected, this corresponds to an index
 * into list of size ni from which we extract the value needed to construct
 * the command.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "setEnum";

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: cix=%d, val=%d, ch=%d\n",
            driverName.c_str(), functionName.c_str(), cix, val, ch);


    const char* cmnd = getCommand(cix);
    if (!cmnd) {
        asynPrint(pasynUser, ASYN_TRACE_FLOW,
                "%s::%s: Command not found\n", driverName.c_str(), functionName.c_str());
        return;
    }

    const std::vector<std::string> list = getKeywordList(cix);
    if (list.empty()) {
        asynPrint(pasynUser, ASYN_TRACE_FLOW,
                "%s::%s: Keyword list is empty\n", driverName.c_str(), functionName.c_str());
        return;
    }
    
    drvScope::setEnum(cmnd, val, list, ch);
}


asynStatus drvTek::getCmnds(int ix, int addr){
/*-----------------------------------------------------------------------------
 * This virtual function reimplements the one in the base class.
 * This routine is called from the pollerThread in the base class when it
 * receives a message on the message queue.
 * ix is the index into parameter library (or the reason)
 * addr is channel or address
 *---------------------------------------------------------------------------*/
    const std::string functionName = "getCmnds";
    asynStatus stat = asynSuccess;
    char cmnd[32];
    int ch = addr + 1;
    int jx = ix - _firstix;

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: jx=%d, addr=%d\n",
            driverName.c_str(), functionName.c_str(), jx, addr);
  
    switch(jx){
        case ixBoTrMode:    getEnum(TrigModeCmnd, ix, trgMode); break;
        case ixMbboTrSou:   getEnum(TrigSouCmnd, ix, trigSou); break;
        case ixBoTrSlo:     getEnum(TrigSloCmnd, ix, trigSlo); break;
        case ixMbbiTrSta:   getEnum(TrigStaCmnd, ix, trigSta); break;
        case ixMbboChScl:   getEnum(ChSclCmnd, ix, chanScl, ch); break;
        case ixSiSource:    stat = getString(WfSouCmnd, ix); break;
        case ixMbboWfWid:   stat = getInt(WfWidCmnd, ix); break;
        case ixLiEvQ:       stat = getInt(EvqCmnd, ix); break;
        case ixBiAcqStat:   stat = getInt(AcqStateCmnd, ix); break;
        case ixSiHead:      stat = getString(HeaderCmnd, ix); break;
  
        case ixMeas1State:  sprintf(cmnd, MeasStateCmnd, 1);
                            getInt(cmnd, ix); break;
  
        case ixMeas2State:  sprintf(cmnd, MeasStateCmnd, 2);
                            getInt(cmnd, ix); break;
  
        case ixMeas3State:  sprintf(cmnd, MeasStateCmnd, 3);
                            getInt(cmnd, ix); break;
  
        case ixMeas4State:  sprintf(cmnd, MeasStateCmnd, 4);
                            getInt(cmnd, ix); break;
  
        case ixMeas1Type:   sprintf(cmnd, MeasTypeCmnd, 1);
                            getEnum(cmnd, ix, measType); break;
  
        case ixMeas2Type:   sprintf(cmnd, MeasTypeCmnd, 2);
                            getEnum(cmnd, ix, measType); break;
  
        case ixMeas3Type:   sprintf(cmnd, MeasTypeCmnd, 3);
                            getEnum(cmnd, ix, measType); break;
  
        case ixMeas4Type:   sprintf(cmnd, MeasTypeCmnd, 4);
                            getEnum(cmnd, ix, measType); break;
  
        default:            stat = drvScope::getCmnds(ix, addr); break;
    } 
  
    callParamCallbacks(addr);
    return stat;
}


asynStatus drvTek::putIntCmnds(int ix, int addr, int v) {
/*-----------------------------------------------------------------------------
 * This is a reimplementation of a virtual function in the base class.
 * It is called from the pollerThread routine in the base dlass when it
 * receives a message on the message queue.
 * ix is the index into parameter library (or the reason)
 * addr is channel or address
 * v is a possible integer set value.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "putIntCmnds";
    asynStatus stat = asynSuccess; 
    char cmnd[32];
    int ch = addr + 1;
    int jx = ix - _firstix;

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: jx=%d, addr=%d, val=%d\n",
            driverName.c_str(), functionName.c_str(), jx, addr, v);

    switch(jx) {
        case ixMbboChScl:   setIntegerParam(addr, ix, v);
                            setEnum(ChSclCmnd, v, chanScl, ch);
                            getFloat(ChSclCmnd, _aoChScl, ch);
                            getTrigLevl();
                            break;
  
        case ixMbboTimDivV: stat=getIntegerParam(_mbboTimDivU,&jx);
                            _setTimePerDiv(v,jx);
                            break;
  
        case ixMbboTimDivU: getIntegerParam(_mbboTimDivV,&jx);
                            _setTimePerDiv(jx,v);
                            break;
  
        case ixMbboWfWid:   setInt(jx, WfDatCmnd, v, ix);
                            break;

        case ixBoTrMode:    setEnum(TrigModeCmnd, v, trgMode);
                            setIntegerParam(ix,v);
                            break;
  
        case ixMbboTrSou:   setEnum(TrigSouCmnd, v, trigSou);
                            break;
  
        case ixBoTrSlo:     setEnum(TrigSloCmnd, v, trigSlo);
                            break;
  
        case ixLoRecall:    sprintf(cmnd,RecallCmnd,v);
                            command(cmnd);
                            update();
                            break;
  
        case ixLoStore:     sprintf(cmnd,SaveCmnd,v);
                            command(cmnd); 
                            break;
  
        case ixMeas1State:  sprintf(cmnd, MeasStateCmnd, 1);
                            setInt(jx, cmnd, v, ix);
                            setIntegerParam(ix, v);
                            break;
  
        case ixMeas2State:  sprintf(cmnd, MeasStateCmnd, 2);
                            setInt(jx, cmnd, v, ix);
                            setIntegerParam(ix, v);
                            break;
  
        case ixMeas3State:  sprintf(cmnd, MeasStateCmnd, 3);
                            setInt(jx, cmnd, v, ix);
                            setIntegerParam(ix, v);
                            break;
  
        case ixMeas4State:  sprintf(cmnd, MeasStateCmnd, 4);
                            setInt(jx, cmnd, v, ix);
                            setIntegerParam(ix, v);
                            break;
  
        case ixMeas1Type:   sprintf(cmnd, MeasTypeCmnd, 1);
                            setEnum(cmnd, v, measType);
                            setIntegerParam(ix, v);
                            sprintf(cmnd, MeasUnitsCmnd, 1);
                            getString(cmnd, _meas1Units);
                            break;
  
        case ixMeas2Type:   sprintf(cmnd, MeasTypeCmnd, 2);
                            setEnum(cmnd, v, measType);
                            setIntegerParam(ix, v);
                            sprintf(cmnd, MeasUnitsCmnd, 2);
                            getString(cmnd, _meas2Units);
                            break;
  
        case ixMeas3Type:   sprintf(cmnd, MeasTypeCmnd, 3);
                            setEnum(cmnd, v, measType);
                            setIntegerParam(ix, v);
                            sprintf(cmnd, MeasUnitsCmnd, 3);
                            getString(cmnd, _meas3Units);
                            break;
  
        case ixMeas4Type:   sprintf(cmnd, MeasTypeCmnd, 4);
                            setEnum(cmnd, v, measType);
                            setIntegerParam(ix, v);
                            sprintf(cmnd, MeasUnitsCmnd, 4);
                            getString(cmnd, _meas4Units);
                            break;
  
        default:            stat=drvScope::putIntCmnds(ix,addr,v); break;
    }

    callParamCallbacks(addr);
    return stat;
}


asynStatus drvTek::putFltCmnds(int ix, int addr, float v) {
/*-----------------------------------------------------------------------------
 * This routine is a reimplementation of a virtual function in base class.
 * It is called from the pollerThread routine in base class when it receives
 * a message on the message queue.
 * ix is the index into parameter library (or the reason)
 * addr is channel or address
 * v is a possible integer set value.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "putFltCmnds";
    asynStatus stat = asynSuccess;
    int jx = ix - _firstix;

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: jx=%d, addr=%d, val=%E\n",
            driverName.c_str(), functionName.c_str(), jx, addr, v);

    switch (jx) {
        default:
            stat = drvScope::putFltCmnds(ix,addr,v);
            break;
    }

    callParamCallbacks(addr);
    return stat;
}


asynStatus drvTek::writeInt32(asynUser* pau, epicsInt32 v) {
/*-----------------------------------------------------------------------------
 * This method overrides the virtual method in asynPortDriver.  Here we service
 * all write requests comming from EPICS records.
 * Parameters:
 *  pau         (in) structure containing addr and reason.
 *  v           (in) this is the command index, which together with
 *              pau->reason define the command to be sent.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "writeInt32";
    asynStatus stat = asynSuccess;
    int addr;
    int ix = pau->reason;
    int jx = ix - _firstix;

    stat = getAddress(pau, &addr);
    if (stat != asynSuccess) return stat;

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: jx=%d, addr=%d, val=%d\n",
            driverName.c_str(), functionName.c_str(), jx, addr, v);

    switch (jx) {
        case ixMbboWfWid:
            setInt(jx, WfWidCmnd, v, jx);
            stat = setIntegerParam(jx, v);
            break;
        default:
            stat = drvScope::writeInt32(pau,v);
            break;
    }

    callParamCallbacks(addr);
    return stat;
}


asynStatus drvTek::writeFloat64(asynUser* pau, epicsFloat64 v){
/*-----------------------------------------------------------------------------
 * This method overrides the virtual method in asynPortDriver.  Here we service
 * all write requests comming from EPICS records.
 * Parameters:
 *  pau         (in) structure containing addr and reason.
 *  v           (in) this is the command index, which together with
 *              pau->reason define the command to be sent.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "writeFloat64";
    asynStatus stat = asynSuccess;
    int addr;
    int ix = pau->reason;
    int jx = ix - _firstix;

    stat = getAddress(pau, &addr);
    if (stat != asynSuccess) return(stat);

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: jx=%d, addr=%d, val=%E\n",
            driverName.c_str(), functionName.c_str(), jx, addr, v);

    switch (jx) {
      default:
        stat = drvScope::writeFloat64(pau, v);
        break;
    }

    callParamCallbacks(addr);
    return stat;
}

