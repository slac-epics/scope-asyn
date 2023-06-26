/* drvTDS.cpp
 * Asyn driver to control Tektronix TDS 3000 series scopes.
 * asynPortDriver --> drvScope --> drvTek --> drvTDS
 *---------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

#include <cantProceed.h>
#include <epicsExport.h>
#include <initHooks.h>
#include <iocsh.h>

#include "drvTDS.h"

namespace {
    const std::string driverName = "drvTDS";
}


drvTDS::drvTDS(const char* port, const char* udp):
        drvTek(port, udp) {
/*------------------------------------------------------------------------------
 * Constructor for the drvTDS class. Calls constructor for the drvTek class.
 *   port  The name of the asyn port driver to be created.
 *   udp   The name of the octet (communication) port.
 *---------------------------------------------------------------------------*/
    _initializeParams();
    message("Constructor drvTDS success");
}

void drvTDS::_initializeParams() {
/*------------------------------------------------------------------------------
 * Initialize instrument-specific parameters.
 *---------------------------------------------------------------------------*/
    // Supported commands
    ChOnCmnd       = "SEL:CH%d";
    ChPosCmnd      = "CH%d:POS";
    ChImpCmnd      = "CH%d:IMP";
    ChCplCmnd      = "CH%d:COUP";
    ChSclCmnd      = "CH%d:SCA";
    WfDatCmnd      = "DAT:SOU CH%d; :WAVF?";
    WfNptCmnd      = "HOR:RECORDL";
    WfWidCmnd      = "DAT:WID";
    WfStrtCmnd     = "DAT:STAR";
    WfStopCmnd     = "DAT:STOP";
    WfFormCmnd     = "DAT:ENC";
    TmDlyOfsCmnd   = "HOR:DEL:TIM";
    TmDlyEnCmnd    = "HOR:DEL:STATE";
    TmSclCmnd      = "HOR:MAI:SCA";
    TrigPosCmnd    = "HOR:TRIG:POS";
    TrigLevCmnd    = "TRIG:A:LEV";
    TrigHoldCmnd   = "TRIG:A:HOL";
    TrigModeCmnd   = "TRIG:A:MOD";
    TrigSouCmnd    = "TRIG:A:EDGE:SOU";
    TrigSloCmnd    = "TRIG:A:EDGE:SLO";
    AcqStateCmnd   = "ACQ:STATE";
    TrigStaCmnd    = "TRIG:STATE?";
    RunCmnd        = "ACQ:STATE RUN";
    StopCmnd       = "ACQ:STATE STOP";
    EseCmnd        = "*ESE";
    ClsCmnd        = "*CLS";
    EsrCmnd        = "*ESR?";
    EvqCmnd        = "EVQ?";
    OpcCmnd        = "*OPC";
    StbCmnd        = "*STB?";
    ResetCmnd      = "*RST";
    RecallCmnd     = "*RCL %d";
    SaveCmnd       = "*SAV %d";
    IdnCmnd        = "*IDN?";
    IPAddrCmnd     = "ETHER:IPADD?";
    WfSouCmnd      = "DATA:SOU";
    InitCmnd       = "*CLS; :DAT:ENC RIB; :HOR:RECORDL 500; :HEAD OFF; :VERB ON; DAT:WID 1; :DAT:STAR 1; :DAT:STOP 500; HOR:DEL:STATE 1";
    HeaderCmnd     = "HEAD?";
    GetConfCmnd    = "*LRN?";
    ErrMsgCmnd     = "EVM?";
    MeasValCmnd    = "MEASU:MEAS%d:VAL?";
    MeasUnitsCmnd  = "MEASU:MEAS%d:UNI?";
    MeasTypeCmnd   = "MEASU:MEAS%d:TYP";
    MeasStateCmnd  = "MEASU:MEAS%d:STATE";

    // Keyword lists for commands which return specific strings.
    chanImp = {"FIFTY", "MEG"};
    chanCpl = {"DC", "AC", "GND"};
    chanScl = {"1.0E-3","2.0E-3","5.0E-3","1.0E-2","2.0E-2",
            "5.0E-2","1.0E-1","2.0E-1","5.0E-1","1.0E0","2.0E0","5.0E0","1.0E1"};
    timDivV = {"1","2","4","10","20","40","100","200","400"};
    timDivU = {"ns","us","ms","s"};
    trgMode = {"NORMAL","AUTO"};
    trigSou = {"CH1","CH2","CH3","CH4","LINE","VERTICAL", "EXT10","EXT"};
    trigSlo = {"FALL","RISE"};
    trigSta = {"AUTO","ARMED","READY","SAVE","TRIGGER"};
    dataFmt = {"ASCII","RIBINARY", "RPBINARY","SRIBINARY","SRPBINARY"};
    measType = {"AMPLITUDE", "FREQUENCY", "DELAY", "MAXIMUM", "MINIMUM", "MEAN", "PERIOD",
            "PHASE", "PK2PK", "PWIDTH", "RISE", "FALL", "RMS"};

    // List of commands and corresponding keywords lists that we implement.
    // The order must agree with the order of enumerated names defined in the drvScope.h header file,
    // where the first item is ixBoChOn, the second is ixAoChPos, and so on.
    commands = {
        {ChOnCmnd,           {}},
        {ChPosCmnd,          {}},
        {ChImpCmnd,     chanImp},
        {ChCplCmnd,     chanCpl},
        {ChSclCmnd,     chanScl},
    
        {WfDatCmnd,          {}},
        {WfNptCmnd,          {}},
        {WfStrtCmnd,         {}},
        {WfStopCmnd,         {}},
        {WfFormCmnd,    dataFmt},
    
        {TmDlyOfsCmnd,       {}},
        {TmDlyEnCmnd,        {}},
        {TmSclCmnd,          {}},
        {TrigPosCmnd,        {}},
        {TrigLevCmnd,        {}},

        {TrigHoldCmnd,       {}},
        {RunCmnd,            {}},
        {StopCmnd,           {}},
        {EseCmnd,            {}},
        {ClsCmnd,            {}},

        {EsrCmnd,            {}},
        {OpcCmnd,            {}},
        {StbCmnd,            {}},
        {ResetCmnd,          {}},
        {IdnCmnd,            {}},
    
        {IPAddrCmnd,         {}},
        {InitCmnd,           {}},
        {GetConfCmnd,        {}},
        {ErrMsgCmnd,         {}},
        {MeasValCmnd,        {}},

        {MeasUnitsCmnd,      {}},
        {MeasTypeCmnd, measType},
        {MeasStateCmnd,      {}}
    };

    // Horizontal scale parameters
    horScaleParams = {
            {1, 225, 51},
            {2, 200, 101},
            {4, 150, 201},
            {10, 0, 500},
    };
}


int drvTDS::_parseWfPreamble(const char* buf, int* ln, int* nb, double* ym, double* yz, double* yo) {
/*-----------------------------------------------------------------------------
 * Unpacks the waveform preamble string.  Returns the length of the preamble.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "_parseWfPreamble";
    int preamble_len = 0, num_params = 0, wd = 0; 
    int nbyt, nbit, wf_len, ptof; 
    char enc[20], bfmt[20], bord[20], ids[80], ptfm[20], xunt[20], yunt[20];
    double xinc, xzr, ymult, yzr, yof;
    const int NUM_PREAMBLE_PARAMS = 16;

    num_params = sscanf(buf,"%d;%d;%19[^;];%19[^;];%19[^;];%d;%79[^;];%19[^;];"
          "%lg;%d;%lg;%19[^;];%lg;%lg;%lg;%19[^;];%n",
          &nbyt, &nbit, enc, bfmt, bord, &wf_len, ids, ptfm,
          &xinc, &ptof, &xzr, xunt, &ymult, &yzr, &yof, yunt, &preamble_len);

    //printf("parseWfPreamble: yunt=%s, ymult=%g, yof=%g, yzr=%g, i=%d\n", yunt, ymult, yof, yzr, preamble_len);

    if (num_params != NUM_PREAMBLE_PARAMS) {
        if (_err_count == 1) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s: failed to unpack preamble, num_params=%d\n",
                    driverName.c_str(), functionName.c_str(), num_params);
        } else if (_err_count == 2) {
            setConnectedState(false);
        }
        _err_count++;
        return -1;
    } else {
        if (_err_count) {
            setConnectedState(true);
        }
    }

    // wd is the width of chars in the waveform length, e.g. for 500, wd = 3
    if ((num_params = sscanf(&buf[preamble_len], "#%1d", &wd)) != 1) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s: failed to get width, num_params=%d\n",
                driverName.c_str(), functionName.c_str(), num_params);
        return -1;
    }

    preamble_len += (2 + wd);

    //printf("_parseWfPreamble: nbyt=%d,nbit=%d,enc=%s,bfmt=%s\n", nbyt, nbit, enc, bfmt);
    //printf("_parseWfPreamble: bord=%s,wf_len=%d,ids=%s,ptfm=%s\n", bord, wf_len, ids, ptfm);
    //printf("_parseWfPreamble: xinc=%g,ptof=%d,xzr=%g,xunt=%s\n", xinc, ptof, xzr, xunt);
    //printf("_parseWfPreamble: ymult=%g,yzr=%g,yof=%g,yunt=%s\n", ymult, yzr, yof, yunt);
    //printf("_parseWfPreamble: preamble_len=%d, wd=%d\n", preamble_len, wd);

    *ln = wf_len;
    *nb = nbyt;
    *ym = ymult;
    *yz = yzr;
    *yo = yof;

    return preamble_len;
}


// Configuration routines.  Called directly, or from the iocsh function below
extern "C" {

int drvTDSConfigure(const char* port, const char* udp) {
/*-----------------------------------------------------------------------------
 * EPICS iocsh callable function to call constructor for the drvTDS class.
 *  port The name of the asyn port driver to be created.
 *  udp  The I/O port.
 *---------------------------------------------------------------------------*/
    new drvTDS(port, udp);
    return asynSuccess;
}

/* EPICS iocsh shell commands */
static const iocshArg initArg0 = {"port", iocshArgString};
static const iocshArg initArg1 = {"udp", iocshArgString};
static const iocshArg * const initArgs[] = {&initArg0, &initArg1};
static const iocshFuncDef initFuncDef = {"drvTDSConfigure", 2, initArgs};
static void initCallFunc(const iocshArgBuf *args){
    drvTDSConfigure(args[0].sval, args[1].sval);
}

void drvTDSRegister(void) {
    iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(drvTDSRegister);
}

