/* drvScope.cpp
 * Base class for oscilloscope drivers.
 *  * asynPortDriver --> drvScope
 *---------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <algorithm>    // std::find
#include <vector>       // std::vector

#include <dbAccess.h>
#include <cantProceed.h>
#include <epicsTypes.h>
#include <epicsThread.h>
#include <errlog.h>
#include <alarm.h>
#include <asynOctetSyncIO.h>

#include "drvScope.h"

namespace {
const std::string driverName = "drvScope";

static void pollerThreadC(void* pPvt) {
    drvScope* pdrvScope = (drvScope*)pPvt;
    pdrvScope->pollerThread();
}
}


drvScope::drvScope(const char* port, const char* udp):
        asynPortDriver(port, NCHAN,
                asynInt32Mask | asynFloat64Mask | asynFloat32ArrayMask |
                asynOctetMask | asynDrvUserMask,
                asynInt32Mask | asynFloat64Mask | asynFloat32ArrayMask | asynOctetMask,
                ASYN_CANBLOCK | ASYN_MULTIDEVICE,1,0,0),
                _ncmnds(0),
                _pollT(0.1),
                _markchan(0),
                _chSel(0),
                _tracemode(0),
                _rdtraces(1),
                _posInProg(0),
                _measEnabled(0),
                _pollCount(0),
                _err_count(0),
                _timerQueue(&epicsTimerQueueActive::allocate(true)) {
/*------------------------------------------------------------------------------
 * Constructor for the drvScope class. Calls constructor for the asynPortDriver
 * base class.
 *  port The name of the asyn port driver to be created.
 *  udp  The I/O port.
 *---------------------------------------------------------------------------*/
    int status = asynSuccess;
    bool conn = false;

    for (int i=0; i<NCHAN; i++) {
        _analize[i] = _mix1[i] = _mix2[i] = 0;
        _area[i] = _pedestal[i] = 0.0;
    }

    status = pasynOctetSyncIO->connect(udp, 0, &pasynUser, 0);

    if (status != asynSuccess) {
        errlogPrintf("%s::%s:connect: failed to connect to port %s\n",
            driverName.c_str(), driverName.c_str(), udp);
    } else {
        epicsPrintf("%s::%s: connected to port %s\n", driverName.c_str(), driverName.c_str(), udp);
        conn = true;
    }

    createParam(boChOnStr,         asynParamInt32,         &_boChOn);
    createParam(aoChPosStr,        asynParamFloat64,       &_aoChPos);
    createParam(boChImpStr,        asynParamInt32,         &_boChImp);
    createParam(mbboChCplStr,      asynParamInt32,         &_mbboChCpl);
    createParam(aoChSclStr,        asynParamFloat64,       &_aoChScl);

    createParam(wfTraceStr,        asynParamFloat32Array,  &_wfTrace);
    createParam(loWfNptsStr,       asynParamInt32,         &_loWfNpts);
    createParam(loWfStartStr,      asynParamInt32,         &_loWfStart);
    createParam(loWfStopStr,       asynParamInt32,         &_loWfStop);
    createParam(siWfFmtStr,        asynParamOctet,         &_siWfFmt);

    createParam(aoTimDlyStr,       asynParamFloat64,       &_aoTimDly);
    createParam(boTimDlyStStr,     asynParamInt32,         &_boTimDlySt);
    createParam(aiTimDivStr,       asynParamFloat64,       &_aiTimDiv);
    createParam(aoTrPosStr,        asynParamFloat64,       &_aoTrPos);
    createParam(aoTrLevStr,        asynParamFloat64,       &_aoTrLev);

    createParam(aoTrHOffStr,       asynParamFloat64,       &_aoTrHOff);
    createParam(boRunStr,          asynParamInt32,         &_boRun);
    createParam(boStopStr,         asynParamInt32,         &_boStop);
    createParam(loEseStr,          asynParamInt32,         &_loEse);
    createParam(boClsStr,          asynParamInt32,         &_boCls);

    createParam(liEsrStr,          asynParamInt32,         &_liEsr);
    createParam(siOpcStr,          asynParamOctet,         &_siOpc);
    createParam(liStbStr,          asynParamInt32,         &_liStb);
    createParam(boResetStr,        asynParamInt32,         &_boReset);
    createParam(wfIdnStr,          asynParamOctet,         &_wfIdn);

    createParam(siIpAddrStr,       asynParamOctet,         &_siIpAddr);
    createParam(boInitStr,         asynParamInt32,         &_boInit);
    createParam(boSaveStr,         asynParamInt32,         &_boSave);
    createParam(boEvMsgStr,        asynParamInt32,         &_boEvMsg);
    createParam(siTimDlyStr,       asynParamOctet,         &_siTimDly);

    createParam(siTimDivStr,       asynParamOctet,         &_siTimDiv);

    createParam(siNameStr,         asynParamOctet,         &_siName);
    createParam(boGetWfStr,        asynParamInt32,         &_boGetWf);
    createParam(boGetWfAStr,       asynParamInt32,         &_boGetWfA);
    createParam(biCtGetsStr,       asynParamInt32,         &_biCtGets);
    createParam(boUpdtStr,         asynParamInt32,         &_boUpdt);

    createParam(soCmndStr,         asynParamOctet,         &_soCmnd);
    createParam(wfReplyStr,        asynParamOctet,         &_wfReply);
    createParam(aoPTMOStr,         asynParamFloat64,       &_aoPTMO);
    createParam(boAnalStr,         asynParamInt32,         &_boAnal);
    createParam(boAPedStr,         asynParamInt32,         &_boAPed);

    createParam(aiAreaStr,         asynParamFloat64,       &_aiArea);
    createParam(aiPedStr,          asynParamFloat64,       &_aiPed);
    createParam(mbboMChanStr,      asynParamInt32,         &_mbboMChan);
    createParam(loMark1Str,        asynParamInt32,         &_loMark1);
    createParam(loMark2Str,        asynParamInt32,         &_loMark2);

    createParam(wfEventStr,        asynParamOctet,         &_wfEvent);
    createParam(wfMessgStr,        asynParamOctet,         &_wfMessg);
    createParam(boChSelStr,        asynParamInt32,         &_boChSel);
    createParam(loChPosStr,        asynParamInt32,         &_loChPos);
    createParam(loTrLevStr,        asynParamInt32,         &_loTrLev);

    createParam(liMQSuccsStr,      asynParamInt32,         &_liMsgQS);
    createParam(liMQFailStr,       asynParamInt32,         &_liMsgQF);
    createParam(mbboTracModStr,    asynParamInt32,         &_mbboTracMod);
    createParam(liXNptsStr,        asynParamInt32,         &_liXNpts);
    createParam(biStateStr,        asynParamInt32,         &_biState);

    createParam(boErUpdtStr,       asynParamInt32,         &_boErUpdt);
    createParam(wfFPathStr,        asynParamOctet,         &_wfFPath);
    createParam(boRestoreStr,      asynParamInt32,         &_boRestore);
    createParam(boRdTracesStr,     asynParamInt32,         &_boRdTraces);
    createParam(aiWfTimeStr,       asynParamFloat64,       &_aiWfTime);

    createParam(aiWfTMinStr,       asynParamFloat64,       &_aiWfTMin);
    createParam(aiWfTMaxStr,       asynParamFloat64,       &_aiWfTMax);
    createParam(aiWfPerStr,        asynParamFloat64,       &_aiWfPeriod);
    createParam(aiWfRateStr,       asynParamFloat64,       &_aiWfRate);
    createParam(boMeasEnabledStr,  asynParamInt32,         &_boMeasEnabled);

    _firstix = _boChOn;

    setStringParam(_siName, driverName);
    setIntegerParam(_biState, conn);
    setIntegerParam(_boRdTraces, _rdtraces);
    setIntegerParam(_boMeasEnabled, _measEnabled);
    setDoubleParam(_aoPTMO, _pollT);

    callParamCallbacks(0);

    _pmq = new epicsMessageQueue(NMSGQ, MSGQNB);

    epicsThreadCreate(driverName.c_str(), epicsThreadPriorityHigh,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)pollerThreadC, this);

    _chPosTimer = &_timerQueue->createTimer();

}


drvScope::~drvScope() {
/*-----------------------------------------------------------------------------
 * Destructor.
// TODO: Add an exit handler in derived classes so that this actually runs
 *---------------------------------------------------------------------------*/
    const std::string functionName = "~drvScope";    

    _chPosTimer->destroy();
    _timerQueue->release();

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s::%s: Exiting...\n", driverName.c_str(), functionName.c_str());
}


void drvScope::pollerThread() {
/*-----------------------------------------------------------------------------
 * This function runs in a separate thread.  It waits for the poll time.
 * Reads a list of registers and it does callbacks to all
 * clients that have registered with registerDevCallback
 *---------------------------------------------------------------------------*/
    const std::string functionName = "pollerThread"; 
    msgq_t msgq;
    int status;

    // Wait until iocInit is finished
    while (!interruptAccept) {
        epicsThreadSleep(0.2);
    }

    // Run post-init commands
    afterInit();

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s::%s: Starting polling loop...\n", driverName.c_str(), functionName.c_str());

    // Poll forever
    while(1) {
        status = _pmq->tryReceive(&msgq,sizeof(msgq));
        if (status == -1) {
            if (_rdtraces) {
                _getTraces();
            }
            if (_measEnabled) {
                getMeasurements(_pollCount);
            }
            _pollCount = (_pollCount >= 99)?(_pollCount = 0):(_pollCount + 1);
            epicsThreadSleep(_pollT);
        } else {
            asynPrint(pasynUser, ASYN_TRACE_FLOW,
                    "%s::%s: msgq.type=%d, msgq.ix=%d, msgq.addr=%d\n", driverName.c_str(), functionName.c_str(),
                    msgq.type, msgq.ix, msgq.addr);
            switch(msgq.type){
                case enPutInt: putIntCmnds(msgq.ix,msgq.addr,msgq.ival);
                               break;
                case enQuery:  getCmnds(msgq.ix,msgq.addr);
                               break;
                case enPutFlt: putFltCmnds(msgq.ix,msgq.addr,msgq.fval);
                               break;
            }
        }
    }
}


void drvScope::_evMessage() {
/*-----------------------------------------------------------------------------
 * Gets next event message from the instrument and posts it in a db record.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "_evMessage"; 
    const char* pcmd = getCommand(ixBoEvMsg);

    if (!pcmd) return;
    asynStatus status = command(pcmd);

    if (status != asynSuccess){
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: command failed\n", 
                driverName.c_str(), functionName.c_str());
        return;
    }

    _rbuf[MSGNB-1] = 0;

    setStringParam(_wfEvent, _rbuf);
    callParamCallbacks();
}


void drvScope::message(const std::string msg) {
/*-----------------------------------------------------------------------------
 * Constructs and posts a message.
 *---------------------------------------------------------------------------*/
    char timestamp[64];
    epicsTimeStamp etime;

    epicsTimeGetCurrent(&etime);
    epicsTimeToStrftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &etime);
    
    std::string msg_full(timestamp + std::string(": ") + msg);

    setStringParam(_wfMessg, msg_full);
    callParamCallbacks();
}


void drvScope::putInMessgQ(int tp, int ix, int addr, int iv, float fv) {
/*-----------------------------------------------------------------------------
 * Construct a message and put in the message queue.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "putInMessgQ";
    int status; 
    msgq_t messg;

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: addr=%d, ix=%d\n", 
            driverName.c_str(), functionName.c_str(), addr, ix);

    messg.type = tp;
    messg.ix = ix;
    messg.addr = addr;
    messg.ival = iv;
    messg.fval = fv;

    status = _pmq->trySend(&messg, sizeof(messg));

    if (!status) {
        _mqSent++;
    } else {
        _mqFailed++;
    }

    setIntegerParam(_liMsgQS, _mqSent);
    setIntegerParam(_liMsgQF, _mqFailed);
    callParamCallbacks(0);
}


asynStatus drvScope::writeRd(int cix, int ch, char* buf, int blen) {
/*-----------------------------------------------------------------------------
 * A protected write-read function that can be called from specific class.
 * Where, cix is an index to list of commands, ch is {1,2,3,4} and data read
 * are returned in buf or length blen bytes.
 *---------------------------------------------------------------------------*/
    const char* pcmd = getCommand(cix);
    char cmnd[32];

    if (!pcmd) return asynError;

    sprintf(cmnd, pcmd, ch);

    return(_wtrd(cmnd, strlen(cmnd), buf, blen));
}


asynStatus drvScope::writeRd(const char* cmnd, char* buf, int blen) {
/*-----------------------------------------------------------------------------
 * A protected write-read function that can be called from specific class,
 * which sends a  command cmnd to the scope. ch is {1,2,3,4} and data read
 * are returned in buf or length blen bytes.
 *---------------------------------------------------------------------------*/
    return(_wtrd(cmnd, strlen(cmnd), buf, blen));
}


asynStatus drvScope::_write(const char* pw, size_t nw) {
/*-----------------------------------------------------------------------------
 * Here we perform an "atomic" write.  Parameters:
 *  pw  buffer that has data to be written,
 *  nw  number of bytes of data in pwb,
 *---------------------------------------------------------------------------*/
    const std::string functionName = "_write";
    asynStatus status = asynSuccess;
    size_t nbw;

    pasynOctetSyncIO->flush(pasynUser);
    status = pasynOctetSyncIO->write(pasynUser, pw, nw, 1, &nbw);
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: status=%d, pw=%s\n",
            driverName.c_str(), functionName.c_str(), status, pw);
    if (status) {
        // Print an error message if this is the first error        
        if (!_err_count) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s::%s: ERROR: status=%d, pw=%s, nbw=%zu\n",
              driverName.c_str(), functionName.c_str(), status, pw, nbw);
        }
        _err_count++;
    }
    return status;
}


asynStatus drvScope::_wtrd(const char* pw, size_t nw, char* pr, size_t nr) {
/*-----------------------------------------------------------------------------
 * Here we perform an "atomic" write and read operation sequence.  Parameters:
 *  pw  buffer that has data to be written,
 *  nw  number of bytes of data in pwb,
 *  pr  buffer into which data will be read in,
 *  nr  size of read buffer in bytes,
 *---------------------------------------------------------------------------*/
    const std::string functionName = "_wtrd";
    asynStatus status = asynSuccess;
    int eom, nParams;
    size_t nbw, nbr;

    pasynOctetSyncIO->flush(pasynUser);
    status = pasynOctetSyncIO->writeRead(pasynUser, pw, nw, pr, nr, 1, &nbw, &nbr, &eom);

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: status=%d, pw=%s\n",
            driverName.c_str(), functionName.c_str(), status, pw);

    if ((status != asynSuccess) || !nbr || (nbr > nr)) {
        if (_err_count < 5) {
            // Allow 5 errors, then set error mode, print a message, set alarm stat/sevr
            setIntegerParam(_biState, false);
            callParamCallbacks();
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                    "%s::%s: ERROR: status=%d, pw=%s, nbw=%zu, nbr=%zu\n",
                    driverName.c_str(), functionName.c_str(), status, pw, nbw, nbr);
            // Set alarm stat/sevr
            status = getNumParams(&nParams);
            if (status != asynSuccess) {
                asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s: getNumParams failed\n", 
                          driverName.c_str(), functionName.c_str());
            }
            for (int param = 0; param < nParams; param++) {
                for (int addr = 0; addr < NCHAN; addr++) {
                    if (param != _biState) {
                        setParamAlarmStatus(addr, param, COMM_ALARM);
                        setParamAlarmSeverity(addr, param, INVALID_ALARM);
                    }
                    callParamCallbacks(addr);
                }
            }
        }
        _err_count++;
    } else {
        if (_err_count) {
            // Reset error status
            setIntegerParam(_biState, true);
            callParamCallbacks();
            asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s: Status OK after %ld error(s)\n",
                    driverName.c_str(), functionName.c_str(), _err_count);
            _err_count = 0;
            // Clear alarm stat/sevr
            status = getNumParams(&nParams);
            if (status != asynSuccess) {
                asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s: getNumParams failed\n", 
                          driverName.c_str(), functionName.c_str());
            }
            for (int param = 0; param < nParams; param++) {
                for (int addr = 0; addr < NCHAN; addr++) {
                    if (param != _biState) {
                        setParamAlarmStatus(addr, param, NO_ALARM);
                        setParamAlarmSeverity(addr, param, NO_ALARM);
                    }
                    callParamCallbacks(addr);
                }
            }
        }
    }

    return status;
}


void drvScope::getChanPos(int addr) {
/*-----------------------------------------------------------------------------
 * This virtual method is coded to work with the tds3000 series scopes.  It
 * needs to be reimplemented for other scope types, e.g. Rigol.
 * addr is parameter library index or addr+1 channel
 *---------------------------------------------------------------------------*/
    getFloat(ixAoChPos, _aoChPos, addr+1);
}


void drvScope::setChanPos(int addr, double v) {
/*-----------------------------------------------------------------------------
 * This virtual method is coded to work with the tds3000 series scopes.  It
 * needs to be reimplemented for other scope types, e.g. Rigol.
 * addr is parameter library index or addr+1 channel
 *---------------------------------------------------------------------------*/
    const char* pcmd = getCommand(ixAoChPos); 
    char cmnd[32];

    if(!pcmd) return;

    sprintf(cmnd, pcmd, addr+1);
    sprintf(cmnd, "%s %f", cmnd, v);
    command(cmnd);
}


void drvScope::saveConfig() {
/*-----------------------------------------------------------------------------
 * This virtual function is reimplemented in a derived class as needed.  It
 * gets an instrument configuration string and writes it to a disk file.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "saveConfig"; 
    const char* pcmd = getCommand(ixBoSave);
    asynStatus status;

    if (!pcmd) return;

    status = command(pcmd);
    if (status != asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s: command failed\n", 
                driverName.c_str(), functionName.c_str());
        return;
    }

    FILE* fd = fopen(_fname, "w");
    if (!fd) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s: fopen failed to open %s\n",
                driverName.c_str(), functionName.c_str(), _fname);
        return;
    }

    int st = fputs(_rbuf, fd);
    if (st == EOF) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s: fputs failed\n", 
                driverName.c_str(), functionName.c_str());
    }

    fclose(fd);
}


void drvScope::restoreConfig() {
/*-----------------------------------------------------------------------------
 * This virtual function is reimplemented in a derived class as needed.  It
 * gets instrument configuration string from a disk file and sends it to
 * the instrument.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "restoreConfig"; 
    FILE* fd = fopen(_fname,"r");

    if (!fd) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s: fopen failed to open %s\n", 
                driverName.c_str(), functionName.c_str(), _fname);
        return;
    }

    char* p = fgets(_rbuf, DBUF_LEN, fd);
    if (!p) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s: fgets failed\n",
                driverName.c_str(), functionName.c_str());
        return;
    }

    asynStatus status = _write(_rbuf, strlen(_rbuf));
    if (status != asynSuccess){
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s: failed to write instrument settings\n",
                driverName.c_str(), functionName.c_str());
        return;
    }

    const char* pcmd = getCommand(ixBoInit);
    if (pcmd) {
        status = command(pcmd);
        if (status != asynSuccess) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s: failed to initialize\n",
                    driverName.c_str(), functionName.c_str());
        }
        update();
    }
}


asynStatus drvScope::putFltCmnds(int ix, int addr, float v) {
/*-----------------------------------------------------------------------------
 * This routine is called from the pollerThread routine, when it receives
 * a message on the message queue.
 * ix is the index into parameter library (or the reason)
 * addr is channel or address
 * v is a possible integer set value.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "putFltCmnds";
    asynStatus status = asynSuccess;
    char cmnd[32];
    int jx = ix - _firstix;
    const char* pcmd = getCommand(jx);

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: jx=%d, pcmd=%s\n",
            driverName.c_str(), functionName.c_str(), jx, pcmd);

    switch(jx) {
        case ixAoChPos:
            _posInProg = 0;
            setChanPos(addr, v);
            if (addr == _chSel) _setPosSlider(v);
            setDoubleParam(addr, ix, v);
            getTrigLevl();
            break;
        case ixAoChScl:
            if (!pcmd) break;
            sprintf(cmnd, pcmd, addr+1);
            sprintf(cmnd, "%s %f", cmnd, v);
            command(cmnd);
            setDoubleParam(addr, ix, v);
            break;
        case ixAoTimDly:
            _setTimeDelayStr(v);
            break;
        case ixAoTrPos:
            if (!pcmd) break;
            sprintf(cmnd, "%s %d", pcmd, (int)v);
            command(cmnd);
            break;
        case ixAoTrLev:
            if (!pcmd) break;
            sprintf(cmnd, "%s %f", pcmd, v);
            command(cmnd);
            setDoubleParam(addr, ix, v);
            getTrigLevl();
            break;
        case ixAoTrHOff:
            if (!pcmd) break;
            sprintf(cmnd, "%s %f", pcmd, v);
            command(cmnd);
            setDoubleParam(addr, ix, v);
            break;
        default:
            status = asynError;
            break;
    }

    callParamCallbacks(addr);
    return status;
}


asynStatus drvScope::putIntCmnds(int ix, int addr, int v) {
/*-----------------------------------------------------------------------------
 * This routine is called from the pollerThread routine, when it receives
 * a message on the message queue.
 * ix is the index into parameter library (or the reason)
 * addr is channel or address
 * v is a possible integer set value.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "putIntCmnds";
    asynStatus status = asynSuccess;
    char cmnd[32]; 
    int ch = addr + 1;
    int jx = ix - _firstix;
    const char* pcmd = getCommand(jx);
    
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: jx=%d, addr=%d, pcmd=%s, val=%d\n",
            driverName.c_str(), functionName.c_str(), jx, addr, pcmd, v);

    switch(jx) {
        case ixBoUpdt:
            update();
            break;
        case ixBoErUpdt:
            _errUpdate();
            break;
        case ixBoSave:
            if (v) saveConfig();
            break;
        case ixBoRestore:
            if (v) restoreConfig();
            break;
        case ixBoChOn:
            if (!pcmd) break;
            sprintf(cmnd, pcmd, addr+1);
            if (v) {
                strcat(cmnd, " ON");
            } else {
                strcat(cmnd, " OFF");
            }
            command(cmnd);
            setIntegerParam(addr, ix, v);
            _selectChannel();
            break;
        case ixBoChImp:
            setIntegerParam(addr, ix, v);
            setEnum(jx, v, ch);
            break;
        case ixMbboChCpl:
            setIntegerParam(addr, ix, v);
            setEnum(jx, v, ch);
            break;
        case ixBoTimDlySt:
            if (!pcmd) break;
            sprintf(cmnd, "%s %d", pcmd, v);
            command(cmnd);
            setIntegerParam(addr, ix, v);
            break;
        case ixBoGetWf:
            getWaveform(addr);
            break;
        case ixBoStop:
        case ixBoRun:
            if (!pcmd) break;
            command(pcmd);
            isRunning();
            break;
        case ixBoReset:
            if (!pcmd) break;
            command(pcmd);
            break;
        case ixBoInit:
            if (!pcmd) break;
            command(pcmd);
            break;
        case ixLoWfNpts:
            setInt(jx, v, ix);
            break;
        case ixLoWfStart:
            setInt(jx, v, ix);
            break;
        case ixLoWfStop:
            setInt(jx, v, ix);
            break;
        case ixLoEse:
            setInt(jx, v, ix);
            break;
        case ixBoCls:
            if (!pcmd) break;
            command(pcmd);
            break;
        case ixBoAPed:
            _doPeds[addr] = 1;
            break;
        case ixBoChSel:
            _selectChan(addr);
            break;
        case ixLoChPos:
            _posInProg = 1;
            _chPosTimer->start(*this, 0.2);
            //_chPosTimer->show(5);
            _chPos = v/100.0;
            break;
        case ixLoTrLev:
            setTrigLevl(v);
            break;
        case ixBoEvMsg:
            if (v) _evMessage();
            break;
        default:
            status = asynError;
            break;
    }

    callParamCallbacks(addr);
    return status;
}


asynStatus drvScope::getCmnds(int ix, int addr) {
/*-----------------------------------------------------------------------------
 * This routine is called from the pollerThread routine, when it receives
 * a message on the message queue.
 * ix is the index into parameter library (or the reason)
 * addr is channel or address
 *---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    int ch = addr + 1;
    int jx = ix - _firstix;
    double dv;

    _opc();

    switch(jx) {
        case ixWfIdn:
            _getIdn();
            break;
        case ixSiIpAddr:
            _getIpAddr();
            break;
        case ixAoTimDly:
            getFloat(jx, ix);
            getDoubleParam(ix, &dv);
            timeDelayStr(dv);
            break;
        case ixAiTimDiv:
            getFloat(jx, ix);
            getDoubleParam(ix, &dv);
            setTimePerDiv(dv);
            break;
        case ixAoTrPos:
            getFloat(jx, ix);
            break;
        case ixAoTrLev:
            getFloat(jx, ix);
            getTrigLevl();
            break;
        case ixAoTrHOff:
            getFloat(jx, ix);
            break;
        case ixSiWfFmt:
            getString(jx, ix);
            break;
        case ixBoChOn:
            _getChanOn(ch);
            break;
        case ixAoChPos:
            getChanPos(addr);
            getDoubleParam(addr, ix, &dv);
            if(addr == _chSel) {
                _setPosSlider(dv);
            }
            break;
        case ixBoChImp:
            getEnum(jx, ix, ch);
            break;
        case ixMbboChCpl:
            getEnum(jx, ix, ch);
            break;
        case ixAoChScl:
            getFloat(jx, ix, ch);
            break;
        case ixLoWfNpts:
            getInt(jx, ix);
            break;
        case ixLoWfStart:
            getInt(jx, ix);
            break;
        case ixLoWfStop:
            getInt(jx, ix);
            break;
        case ixLiEsr:
            getInt(jx, ix);
            break;
        case ixLoEse:
            getInt(jx,ix);
            break;
        case ixLiStb:
            getInt(jx,ix);
            break;
        default:
            status = asynError;
            break;
    }

    callParamCallbacks(addr);
    return status;
}


int drvScope::_opc() {
/*-----------------------------------------------------------------------------
 * OPC query and returns as a function value result returned.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "_opc"; 
    asynStatus status = asynSuccess;
    int count = 0, tries = 0, value = 0;
    char* p;

    _rbuf[0] = 0;

    while(1) {
        status = _wtrd("*OPC?", 5, _rbuf, DBUF_LEN);
        if (status == asynSuccess) {
            p = strchr(_rbuf, '\n');
            if (p) {
                *p = 0;
            }

            value = atoi(_rbuf);
            if (value == 1) {
                break;
            }
            if ((++count) > 10) {
                value = 0;
                break;
            }
            epicsThreadSleep(0.01);
        }  else if ((++tries) > 1) {
            asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: failed in _wtrd after %d tries\n", 
                    driverName.c_str(), functionName.c_str(), tries);
            break;
        }
    }

    return value;
}


void drvScope::_getIdn() {
/*---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    char* p;
    const char* cmnd = getCommand(ixWfIdn);

    if (cmnd) {
        status = _wtrd(cmnd, strlen(cmnd), _rbuf, DBUF_LEN);
        if (status == asynSuccess) {
            p = strchr(_rbuf, '\n');
            if (p) *p = 0;
            status = setStringParam(_wfIdn, _rbuf);
        }
    }
}


void drvScope::_getIpAddr() {
/*---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    char* p0;
    char* p;
    const char* cmnd = getCommand(ixSiIpAddr);

    if (!cmnd) return;

    status = _wtrd(cmnd, strlen(cmnd), _rbuf, DBUF_LEN);
    if (status == asynSuccess) {
        p0 = _rbuf;
        p0 = strchr(p0, '"');

        if (p0) {
            p = strchr(++p0, '"');
            if (p) *p = 0;
        } else {
            p0 = _rbuf;
            p = strchr(p0, '\n');
            if (p) *p = 0;
        }

        status = setStringParam(_siIpAddr,p0);
    }
}


void drvScope::timeDelayStr(float td) {
/*-----------------------------------------------------------------------------
 *---------------------------------------------------------------------------*/
    int uix, m, sign = 1;

    if (td < 0.0) {
        sign = (-1);
        td *= (-1.0);
    }

    if (td < 1.0e-6) {
        uix = 0;
        m = (int)((td*1000.0)*1000000.0 + 0.5);
    } else if (td < 1.0e-3) {
        uix = 1;
        m = (int)(td*1000000.0 + 0.5);
    } else if (td < 1.0) {
        uix = 2;
        m = (int)(td*1000.0 + 0.5);
    } else {
        uix = 3;
        m = (int)(td + 0.5);
    }

    m *= sign;
    timeDelayStr(m, uix);
}


void drvScope::_setTimeDelayStr(float v) {
/*-----------------------------------------------------------------------------
 *---------------------------------------------------------------------------*/
    char str[32];

    timeDelayStr(v);

    const char* pcmd = getCommand(ixAoTimDly);
    if (!pcmd) return;

    sprintf(str, "%s %g", pcmd, v);
    command(str);
}


char* drvScope::_makeQuery(const char* cmnd) {
/*-----------------------------------------------------------------------------
 * Appends '?' to cmnd as needed and returns pointer to the query string.
 *---------------------------------------------------------------------------*/
    if (!cmnd) return NULL;

    int len = strlen(cmnd);
    if (len > CMND_LEN-2) return 0;

    strcpy(_cmnd, cmnd);
    if (!strchr(cmnd, '?')) {
        strcat(_cmnd, "?");
    }

    return(_cmnd);
}


asynStatus drvScope::command(const char* cmnd) {
/*-----------------------------------------------------------------------------
 * Issues a command and puts the reply string, if any, in parameter library.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "command"; 
    asynStatus status = asynSuccess;
    char* p;
    
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: cmnd=%s\n",
            driverName.c_str(), functionName.c_str(), cmnd);

    if (!cmnd) return(asynError);

    int len = strlen(cmnd);

    if (strchr(cmnd, '?')) {
        status = _wtrd(cmnd, len, _rbuf, DBUF_LEN);
        if (status == asynSuccess) {
            p = strchr(_rbuf, '\n');
            if (p) *p = 0;
            status = setStringParam(_wfReply, _rbuf);
        }
    } else {
        status = _write(cmnd, len);
    }

    return status;
}


asynStatus drvScope::command(const char* cmnd, char* prd, int n) {
/*-----------------------------------------------------------------------------
 * Issues a command cmnd and if it is a query, returns result in prd buffer
 * which is n byte long.
 *---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    char* p;

    if (!cmnd) return(asynError);

    int len = strlen(cmnd);

    if (strchr(cmnd, '?')) {
        status = _wtrd(cmnd, len, prd, n);
        if (status == asynSuccess) {
            p = strchr(prd, '\n');
            if (p) *p = 0;
            status = setStringParam(_wfReply, prd);
        }
    } else {
        status = _write(cmnd, len);
    }

    return status;
}


asynStatus drvScope::getInt(int cix, int pix, int ch) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value and puts the obtained value in
 * parameter library at index pix.
 *---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    const char* cmnd = getCommand(cix);
    if (!cmnd) return asynError;

    status = getInt(cmnd, pix, ch);
    return status;
}


asynStatus drvScope::getInt(const char* cmnd, int pix, int ch) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value and puts the obtained value in
 * parameter library at index pix.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "getInt";
    asynStatus status = asynSuccess;
    int val = 0, addr = 0;
    char str[32];
    int len = strlen(cmnd);

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: cmnd=%s, pix=%d, ch=%d\n",
            driverName.c_str(), functionName.c_str(), cmnd, pix, ch);

    if ((len > 30) || (ch < 0) || (ch > NCHAN)) return asynError;

    if (ch > 0) {
        addr = ch - 1;
        sprintf(str, cmnd, ch);
    } else {
        sprintf(str, cmnd);
    }

    if (!strchr(str, '?')) {
        strcat(str,"?");
    }

    status = _wtrd(str, strlen(str), _rbuf, DBUF_LEN);
    if (status == asynSuccess) {
        val = atoi(_rbuf);
        status = setIntegerParam(addr, pix, val);
    }

    return status;
}


void drvScope::setInt(int cix, int v, int pix) {
/*-----------------------------------------------------------------------------
 * Constructs a string command to set an integer value and sends it.
 *---------------------------------------------------------------------------*/
    const char* cmnd = getCommand(cix);
    if (!cmnd) return;
    setInt(cix, cmnd, v, pix);
}


void drvScope::setInt(int cix, const char* cmnd, int v, int pix) {
/*-----------------------------------------------------------------------------
 * Constructs a string command to set an integer value and sends it.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "setInt";
    char str[32];

    sprintf(str, "%s %d", cmnd, v);

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: str=%s, cix=%d, pix=%d\n", 
            driverName.c_str(), functionName.c_str(), str, cix, pix);

    command(str);

    if (!pix) return;

    getInt(cix, pix);
}


asynStatus drvScope::getFloat(int cix, int pix, int ch) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value and puts the obtained value in
 * parameter library at index pix.
 *---------------------------------------------------------------------------*/
    const char* cmnd = getCommand(cix);
    if(!cmnd) return asynError;
    return getFloat(cmnd, pix, ch);
}


asynStatus drvScope::getFloat(const char* cmnd, int pix, int ch) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value and puts the obtained value in
 * parameter library at index pix.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "getFloat";
    asynStatus status = asynSuccess;
    double val = 0.0;
    int addr = 0;
    char str[32];
    int len = strlen(cmnd);

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: cmnd=%s, pix=%d, ch=%d\n",
            driverName.c_str(), functionName.c_str(), cmnd, pix, ch);

    if ((len > 30) || (ch < 0) || (ch > NCHAN)) { 
        return asynError;
    }

    if (ch > 0) {
        addr = ch - 1;
        sprintf(str, cmnd, ch);
    } else {
        sprintf(str, cmnd);
    }

    if (!strchr(str, '?')) {
        strcat(str, "?");
    }

    status = _wtrd(str, strlen(str), _rbuf, DBUF_LEN);
    if (status == asynSuccess) {
        val = atof(_rbuf);
        status = setDoubleParam(addr, pix, val);
    }

    return status;
}


asynStatus drvScope::getEnum(int cix, int pix, int ch) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value for channel i and puts the obtained
 * value in parameter library at index pix.  The reply is a string and a numeric
 * value is the index in a list of strings.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "getEnum";

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: cix=%d, pix=%d, ch=%d\n",
            driverName.c_str(), functionName.c_str(), cix, pix, ch);

    const char* cmnd = getCommand(cix);
    if (!cmnd) return asynError;

    uint ni;
    const char** list = getCmndList(cix, &ni);
    if (!list) return asynError;

    return getEnum(cmnd, pix, list, ni, ch);
}


asynStatus drvScope::getEnum(const char* cmnd, int pix, const char** list, int ni, int ch) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value for channel i and puts the obtained
 * value in parameter library at index pix.  The reply is a string and a
 * numeric value is the index in a list of strings.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "getEnum";
    asynStatus status = asynSuccess;
    int val = 0, addr = 0;
    char str[32];
    int len = strlen(cmnd);

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: cmnd=%s, pix=%d, ch=%d\n",
            driverName.c_str(), functionName.c_str(), cmnd, pix, ch);

    if ((len > 30) || (ch < 0) || (ch > NCHAN)) { 
        return asynError;
    }

    if (ch > 0) {
        addr = ch - 1;
        sprintf(str, cmnd, ch);
    } else {
        sprintf(str, cmnd);
    }

    if (!strchr(str, '?')) {
        strcat(str, "?");
    }

    status = _wtrd(str, strlen(str), _rbuf, DBUF_LEN);
    if (status != asynSuccess) return asynError;

    val = _find(_rbuf, list, ni);
    if (val >= 0) {
        status = setIntegerParam(addr, pix, val);
    }

    return status;
}


asynStatus drvScope::getEnum(const char* cmnd, int pix, std::vector<std::string> list, int ch) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value for channel i and puts the obtained
 * value in parameter library at index pix.  The reply is a string and a
 * numeric value is the index in a list of strings.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "getEnum";
    asynStatus status = asynSuccess;
    int val = 0, addr = 0;
    char str[32];
    int len = strlen(cmnd);

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: cmnd=%s, pix=%d, ch=%d\n",
            driverName.c_str(), functionName.c_str(), cmnd, pix, ch);

    if ((len > 30) || (ch < 0) || (ch > NCHAN)) { 
        return asynError;
    }

    if (ch > 0) {
        addr = ch - 1;
        sprintf(str, cmnd, ch);
    } else {
        sprintf(str, cmnd);
    }

    if (!strchr(str, '?')) {
        strcat(str, "?");
    }

    status = _wtrd(str, strlen(str), _rbuf, DBUF_LEN);
    if (status != asynSuccess) return asynError;

    val = _find(_rbuf, list);
    if (val == -1) {
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: _find failed for cmnd=%s, pix=%d, ch=%d\n",
                driverName.c_str(), functionName.c_str(), cmnd, pix, ch);
        status = asynError;
    } else if (val >= 0) {
        status = setIntegerParam(addr, pix, val);
    }

    return status;
}


void drvScope::setEnum(int cix, int val, int ch) {
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

    uint ni;
    const char** list = getCmndList(cix, &ni);
    if (list) {
        setEnum(cmnd, val, list, ni, ch);
    }
}

void drvScope::setEnum(const char* cmnd, int val, const char** list, int ni, int ch) {
/*-----------------------------------------------------------------------------
 * ix is a bit position that was selected, this corresponds to an index
 * into list of size ni from which we extract the value needed to construct
 * the command.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "setEnum";
    char str[32];

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: cmnd=%s, val=%d, ch=%d\n",
            driverName.c_str(), functionName.c_str(), cmnd, val, ch);

    if ((val < 0) || (val >= ni)) return;

    if (ch > 0) {
        sprintf(str, cmnd, ch);
    } else {
        sprintf(str, cmnd);
    }
    strcat(str, " ");
    strcat(str, list[val]);
    command(str);
}


void drvScope::setEnum(const char* cmnd, int val, const std::vector<std::string> list, int ch) {
/*-----------------------------------------------------------------------------
 * ix is a bit position that was selected, this corresponds to an index
 * into list of size ni from which we extract the value needed to construct
 * the command.
 *---------------------------------------------------------------------------*/
    std::string functionName = "setEnum";
    char str[32];
        
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: cmnd=%s, val=%d, ch=%d\n",
            driverName.c_str(), functionName.c_str(), cmnd, val, ch);

    if ((val < 0) || (val >= int(list.size()))) return;

    if (ch > 0) {
        sprintf(str, cmnd, ch);
    } else {
        sprintf(str, cmnd);
    }

    strcat(str, " ");
    strcat(str, list[val].c_str());
    command(str);
}


asynStatus drvScope::getString(int cix, int pix) {
/*-----------------------------------------------------------------------------
 * Issues a query for a string value and puts the obtained value in
 * parameter library at index pix.
 *---------------------------------------------------------------------------*/
    const char* cmnd = getCommand(cix);
    if (!cmnd) return asynError;

    char str[32]; 
    int len = strlen(cmnd);

    if (len > 30) return asynError;

    strcpy(str, cmnd);
    return getString(str, pix);
}


asynStatus drvScope::getString(const char* cmnd, int pix) {
/*-----------------------------------------------------------------------------
 * Issues a query for a string value and puts the obtained value in
 * parameter library at index pix.
 *---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    char str[32];

    strcpy(str,cmnd);

    if (!strchr(str,'?')) {
        strcat(str,"?");
    }

    status = _wtrd(str, strlen(str), _rbuf, DBUF_LEN);

    if (status == asynSuccess) {
        char* p = strchr(_rbuf, '\n');
        if (p) *p = 0;
        status = setStringParam(pix, _rbuf);
    }

    return status;
}


int drvScope::_find(const char* item, const char** list, int n) {
/*-----------------------------------------------------------------------------
 * Returns an index in list where item matches an element in the list.
 * If no match is found, returns -1.
 *---------------------------------------------------------------------------*/
    int i = 0, m = strlen(item);
    char* p = (char*)strchr(item, '\n');

    if (p) *p = 0;

    while(strncmp(item, list[i], m)) {
        if ((++i) >= n) return -1;
    }

    return i;
}


int drvScope::_find(const char* item, std::vector<std::string> list) {
/*-----------------------------------------------------------------------------
 * Returns an index in list where item matches an element in the list.
 * If no match is found, returns -1.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "_find";

    std::string tstr(item);
    tstr.erase(std::remove(tstr.begin(), tstr.end(), '\n'), tstr.end());

    auto it = std::find(list.begin(), list.end(), tstr);
    if (it == list.end()) {
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: Response %s not found in list\n",
                driverName.c_str(), functionName.c_str(), tstr.c_str());
        //for (std::vector<std::string>::iterator i = list.begin(); i != list.end(); ++i)
        //    printf("    -> %s\n", i->c_str());
        return -1;
    }

    int index = it - list.begin();
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: item=%s, index=%d\n",
            driverName.c_str(), functionName.c_str(), tstr.c_str(), index);
    return index;

}


void drvScope::_setPosSlider(double v) {
/*-----------------------------------------------------------------------------
 * Set slider position to v division.
 *---------------------------------------------------------------------------*/
    if (_posInProg) return;
    setIntegerParam(0, _loChPos, 0);
    setIntegerParam(0, _loChPos, v*100);
    callParamCallbacks(0);
}


void drvScope::_selectChan(int chan) {
/*-----------------------------------------------------------------------------
 * Select channel chan, used to control channel trace position with a slider.
 *---------------------------------------------------------------------------*/
    int ch_on;
    double ch_pos;

    _posInProg = 0;

    if ((chan < 0) || (chan >= NCHAN)) return;

    getIntegerParam(chan, _boChOn, &ch_on);

    if (!ch_on) {
        setIntegerParam(chan, _boChSel, 1);
        setIntegerParam(chan, _boChSel, 0);
        callParamCallbacks(chan);
        return;
    }

    for (int ch=0; ch<NCHAN; ch++) {
        if (ch == chan) {
            ch_on = 1;
        } else {
            ch_on = 0;
        }
        setIntegerParam(ch, _boChSel, ch_on);
        callParamCallbacks(ch);
    }

    getDoubleParam(chan, _aoChPos, &ch_pos);
    _setPosSlider(ch_pos);
    _chSel = chan;
}


void drvScope::_selectChannel() {
/*-----------------------------------------------------------------------------
 * The first enabled channel is selected to be controlled by the pos slider
 *---------------------------------------------------------------------------*/
    int ch_on; 
    static bool firsttime = true;

    getIntegerParam(_chSel, _boChOn, &ch_on);

    if (ch_on && !firsttime) return;

    firsttime = false;

    for (int ch=0; ch<NCHAN; ch++) {
        _selectChan(ch);
    }
}


void drvScope::setChanPosition() {
/*-----------------------------------------------------------------------------
 * Called by epicsTimerNotify::expire() at end of slider move.
 *---------------------------------------------------------------------------*/
    setChanPos(_chSel, _chPos);
    //getFloat(ixAoChPos, _aoChPos, _chSel+1);
    getChanPos(_chSel);
    getTrigLevl();
}


asynStatus drvScope::writeOctet(asynUser* pasynUser, const char* v, size_t nc, size_t* nActual) {
/*-----------------------------------------------------------------------------
 * This method overrides the virtual method in asynPortDriver.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "writeOctet";
    asynStatus status = asynSuccess;
    int ix, jx, addr;

    status = getAddress(pasynUser, &addr);
    if(status != asynSuccess) return status;

    ix = pasynUser->reason;
    jx = ix - _firstix;

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: jx=%d, addr=%d, val=%s\n",
            driverName.c_str(), functionName.c_str(), jx, addr, v);

    switch(jx) {
        case ixSoCmnd:
            status = command(v);
            *nActual = nc;
            break;
        case ixWfFPath:
            strncpy(_fname, v, FNAME);
            _fname[FNAME-1] = 0;
            break;
        default:
            break;
    }

    status = asynPortDriver::writeOctet(pasynUser, v, nc, nActual);
    callParamCallbacks();
    return status;
}


asynStatus drvScope::writeInt32(asynUser* pasynUser, epicsInt32 v) {
/*-----------------------------------------------------------------------------
 * This method overrides the virtual method in asynPortDriver.  Here we service
 * all write requests comming from EPICS records.
 * Parameters:
 *  pasynUser         (in) structure containing addr and reason.
 *  v           (in) this is the command index, which together with
 *              pasynUser->reason define the command to be sent.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "writeInt32";
    asynStatus status = asynSuccess;
    int on, ix, jx, addr;

    status = getAddress(pasynUser, &addr);
    if (status != asynSuccess) return(status);

    ix = pasynUser->reason;
    jx = ix - _firstix;

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: jx=%d, addr=%d, val=%d\n",
            driverName.c_str(), functionName.c_str(), jx, addr, v);

    switch(jx) {
        case ixBoAnal:
            getIntegerParam(addr, _boChOn, &on);
            if (on) {
                _analize[addr] = v;
            } else {
                _analize[addr] = 0;
            }
            setIntegerParam(addr, _boAnal, 1-_analize[addr]);
            setIntegerParam(addr, _boAnal, _analize[addr]);
            break;
        case ixMbboMChan:
            _markchan = MIN(NCHAN-1,MAX(v,0));
            setIntegerParam(_loMark1, _mix1[_markchan] + 1);
            setIntegerParam(_loMark1, _mix1[_markchan]);
            setIntegerParam(_loMark2, _mix2[_markchan] + 1);
            setIntegerParam(_loMark2, _mix2[_markchan]);
            break;
        case ixLoMark1:
            _mix1[_markchan] = v;
            break;
        case ixLoMark2:
            _mix2[_markchan] = v;
            break;
        case ixBoGetWfA:
            _getTraces();
            break;
        case ixBoRdTraces:
            _rdtraces = v;
            setIntegerParam(_boRdTraces, v);
            break;
        case ixBoMeasEnabled:
            _measEnabled = v;
            setIntegerParam(_boMeasEnabled, v);
            getMeasurements(0);
            break;
        case ixMbboTracMod:
            setIntegerParam(addr, _mbboTracMod, v);
            break;
        default:
            putInMessgQ(enPutInt, ix, addr, v);
            break;
    }

    callParamCallbacks(addr);
    return status;
}

asynStatus drvScope::writeFloat64(asynUser* pasynUser, epicsFloat64 v) {
/*-----------------------------------------------------------------------------
 * This method overrides the virtual method in asynPortDriver.  Here we service
 * all write requests comming from EPICS records.
 * Parameters:
 *  pasynUser         (in) structure containing addr and reason.
 *  v           (in) this is the command index, which together with
 *              pasynUser->reason define the command to be sent.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "writeFloat64";
    asynStatus status = asynSuccess;
    int ix, jx, addr;
    float fv = v;

    status = getAddress(pasynUser, &addr);
    if (status != asynSuccess) return status;

    ix = pasynUser->reason;
    jx = ix - _firstix;
    
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: jx=%d, addr=%d, val=%f\n",
            driverName.c_str(), functionName.c_str(), jx, addr, v);

    switch(jx) {
        case ixAoPTMO:
            _pollT = v;
            break;
        default:
            putInMessgQ(enPutFlt, jx, addr, 0, fv);
            break;
    }

    return status;
}


void drvScope::_getTraces() {
/*-----------------------------------------------------------------------------
 * Initiate getting waveform trace data for all channels.  Traces will be
 * read in asynchronously or synchronously depending on the value of the
 * _tracemode variable.  Synchronous mode is when all four traces are obtained
 * for the same event.
 *---------------------------------------------------------------------------*/
    static epicsTimeStamp t1,t2,t3; 
    static bool first = true;
    int tmode;
    bool istrig = true; 
    const char* pcmd;

    epicsTimeGetCurrent(&t1);

    getIntegerParam(_mbboTracMod, &tmode);
    if (tmode == enTMSync){
        if ((istrig = isTriggered())) {
            if ((pcmd = getCommand(_boStop))) {
                command(pcmd);
            }
        }
    }

    if (istrig) {
        for (int ch=0; ch<NCHAN; ch++) {
            getWaveform(ch);
        }
        if(tmode == enTMSync) {
            if ((pcmd=getCommand(_boRun))) {
                command(pcmd);
            }
        }
    }

    epicsTimeGetCurrent(&t2);
    _wfTime = epicsTimeDiffInSeconds(&t2, &t1);

    if(_wfTime < _wfTMin) _wfTMin = _wfTime;
    if(_wfTime > _wfTMax) _wfTMax = _wfTime;

    if (!first) {
        _wfPeriod = epicsTimeDiffInSeconds(&t1, &t3);
        if (_wfPeriod > 0.0) {
            _wfRate = 1.0/_wfPeriod;
        }
    }

    first = false;
    setDoubleParam(_aiWfTime, _wfTime);
    setDoubleParam(_aiWfTMin, _wfTMin);
    setDoubleParam(_aiWfTMax, _wfTMax);
    setDoubleParam(_aiWfPeriod, _wfPeriod);
    setDoubleParam(_aiWfRate, _wfRate);
    t3 = t1;
    setIntegerParam(_biCtGets, 0);
    setIntegerParam(_biCtGets, 1);
    callParamCallbacks(0);
}


void drvScope::_errUpdate() {
/*-----------------------------------------------------------------------------
 * Requests update from Error and Status registers.
 *---------------------------------------------------------------------------*/
    getInt(ixLoEse, _loEse);
    getInt(ixLiEsr, _liEsr);
    getInt(ixLiStb, _liStb);
}


void drvScope::_getChanOn(int ch) {
/*-----------------------------------------------------------------------------
 * Requests read channel on state and other channel parameters as needed.
 *---------------------------------------------------------------------------*/
    const std::string functionName = "_getChanOn";
    static bool first[] = {true, true, true, true};
    int addr = ch-1, val;
    double dval;

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s: ch=%d\n",
            driverName.c_str(), functionName.c_str(), ch);

    getInt(ixBoChOn, _boChOn, ch);
    _selectChannel();

    getIntegerParam(addr ,_boChOn, &val);
    if ((!val) && (!first[addr])) return;
    first[addr] = false;

    getEnum(ixBoChImp, _boChImp, ch);
    getEnum(ixMbboChCpl, _mbboChCpl, ch);
    getFloat(ixAoChScl, _aoChScl, ch);
    getChanScl(ch);
    getChanPos(addr);
    getDoubleParam(addr, _aoChPos, &dval);
    if (addr == _chSel) {
        _setPosSlider(dval);
    }
}


void drvScope::update() {
/*----------------------------------------------------------------------------
 * This is called at startup and periodically to stay in sync with user
 * changing settings on the instrument.
 *--------------------------------------------------------------------------*/
    const std::string functionName = "update";
    int ch;
    static bool firsttime = true;
    double dval;

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s\n",
            driverName.c_str(), functionName.c_str());

    getFloat(ixAiTimDiv, _aiTimDiv);
    getDoubleParam(_aiTimDiv, &dval);
    setTimePerDiv(dval);
    updateUser();

    for (int i=0; i<NCHAN; i++) {
        ch = i+1;
        _getChanOn(ch);
        callParamCallbacks(i);
    }

    getFloat(ixAoTimDly, _aoTimDly);
    getDoubleParam(_aoTimDly, &dval);
    timeDelayStr(dval);
    getInt(ixBoTimDlySt, _boTimDlySt);
    getFloat(ixAoTrPos, _aoTrPos);
    getFloat(ixAoTrLev, _aoTrLev);
    getTrigLevl();
    getFloat(ixAoTrHOff, _aoTrHOff);
    callParamCallbacks();
    firsttime = false;
}


void drvScope::afterInit() {
/*----------------------------------------------------------------------------
 *--------------------------------------------------------------------------*/
    const std::string functionName = "afterInit";
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s\n",
            driverName.c_str(), functionName.c_str());

    const char* pcmd = getCommand(ixBoInit);

    if (pcmd) command(pcmd);

    putInMessgQ(enQuery, _wfIdn, 0, 0);
    putInMessgQ(enQuery, _siIpAddr, 0, 0);

    for (int i=0; i<NCHAN; i++) {
        putInMessgQ(enQuery, _boChOn, i, 0);
    }

    putInMessgQ(enQuery, _aoTimDly, 0, 0);
    putInMessgQ(enQuery, _aiTimDiv, 0, 0);
    putInMessgQ(enQuery, _aoTrPos, 0, 0);
    putInMessgQ(enQuery, _aoTrLev, 0, 0);
    putInMessgQ(enQuery, _aoTrHOff, 0, 0);
    putInMessgQ(enQuery, _siWfFmt, 0, 0);
    putInMessgQ(enQuery, _siOpc, 0, 0);
    putInMessgQ(enQuery, _loWfNpts, 0, 0);
    putInMessgQ(enQuery, _loWfStart, 0, 0);
    putInMessgQ(enQuery, _loWfStop, 0, 0);
    putInMessgQ(enQuery, _liEsr, 0, 0);
    putInMessgQ(enQuery, _loEse, 0, 0);
    putInMessgQ(enQuery, _liStb, 0, 0);
    putInMessgQ(enQuery, _aoTrLev, 0, 0);
}

