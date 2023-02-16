/* drvScope.cpp
 * Asyn driver to control Tektronix TDS 3000 series scopes.  This is a
 * subclass of asynPortDriver, which was created by Mark Rivers.
 * Started on 10/28/2011, zms.
 *---------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

#include <cantProceed.h>
#include <epicsTypes.h>
#include <epicsThread.h>
#include <errlog.h>
#include <asynOctetSyncIO.h>

#include "drvScope.h"

static drvScope* _this;
static const char *dname = "drvScope";

static const int debug = 0;

extern "C" {
static void pollerThreadC(void* pPvt){
    drvScope* _this = (drvScope*)pPvt;
    _this->pollerThread();
}
}

void myTimer::_expired(const epicsTime&){
/*-----------------------------------------------------------------------------
 * Invoked when timer expires.
 *---------------------------------------------------------------------------*/
    _this->setChanPosition();
}


drvScope::drvScope(const char* port, const char* udp):
        asynPortDriver(port, MAX_ADDR,
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
                _pollCount(0) {
/*------------------------------------------------------------------------------
 * Constructor for the drvScope class. Calls constructor for the asynPortDriver
 * base class. Where
 *   portName The name of the asyn port driver to be created.
 *   udpPort is the actual device port name.
 * Parameters passed to the asynPortDriver constructor:
 *  port name
 *  max address
 *  parameter table size
 *  interface mask
 *  interrupt mask,
 *  asyn flags,
 *  auto connect
 *  priority
 *  stack size
 *---------------------------------------------------------------------------*/
    int status = asynSuccess;
    bool conn = false;

    _this = this;

    for (int i=0; i<NCHAN; i++) {
        _analize[i] = _mix1[i] = _mix2[i] = 0;
        _area[i] = _pedestal[i] = 0.0;
    }

    status = pasynOctetSyncIO->connect(udp, 0, &pasynUser, 0);

    if(status != asynSuccess) {
        printf("%s::%s:connect: failed to connect to port %s\n",
            dname, dname, udp);
    } else {
        printf("%s::%s:connect: connected to port %s\n", dname, dname, udp);
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

    setStringParam(_siName, dname);
    setIntegerParam(_biState, conn);
    setIntegerParam(_boRdTraces, _rdtraces);
    setIntegerParam(_boMeasEnabled, _measEnabled);
    setDoubleParam(_aoPTMO, _pollT);

    callParamCallbacks(0);

    _pmq = new epicsMessageQueue(NMSGQ, MSGQNB);

    epicsThreadCreate(dname,epicsThreadPriorityHigh,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)pollerThreadC,this);

    epicsTimerQueueActive& _tmq = epicsTimerQueueActive::allocate(true);

    _chPosTimer = new myTimer("chPosTimer", _tmq);

}


void drvScope::pollerThread(){
/*-----------------------------------------------------------------------------
 * This function runs in a separate thread.  It waits for the poll time.
 * Reads a list of registers and it does callbacks to all
 * clients that have registered with registerDevCallback
 *---------------------------------------------------------------------------*/
    //static const char* iam = "pollerThread"; 
    msgq_t msgq;
    int status;

    while(1){
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


void drvScope::_evMessage(){
/*-----------------------------------------------------------------------------
 * Gets next event message from the instrument and posts it in a db record.
 *---------------------------------------------------------------------------*/
    const char* pcmd = getCommand(ixBoEvMsg);

    if (!pcmd) return;
    asynStatus status = command(pcmd);

    if(status != asynSuccess){
        errlogPrintf("%s::_efMessage:command: failed\n", dname);
        return;
    }

    _rbuf[MSGNB-1] = 0;

    setStringParam(_wfEvent, _rbuf);
    callParamCallbacks();
}


void drvScope::message(const char* m){
/*-----------------------------------------------------------------------------
 * Constructs and posts a message.
 *---------------------------------------------------------------------------*/
    int len;
    char _time[64];
    epicsTimeStamp etime;

    epicsTimeGetCurrent(&etime);
    epicsTimeToStrftime(_time, sizeof(_time), "%Y-%m-%d %H:%M:%S", &etime);
    strncpy(_mbuf, _time, MSGNB);
    _mbuf[MSGNB-1] = 0;

    if (strlen(_mbuf) < (MSGNB-20)) {
        strcat(_mbuf, " ");
        len = strlen(_mbuf);
        strncat(_mbuf, m, MSGNB-len-1);
        _mbuf[MSGNB-1] = 0;
    }

    setStringParam(_wfMessg, _mbuf);
    callParamCallbacks();
}


void drvScope::putInMessgQ(int tp, int ix, int addr, int iv, float fv) {
/*-----------------------------------------------------------------------------
 * Construct a message and put in the message queue.
 *---------------------------------------------------------------------------*/
    if (debug) printf("%s: putInMessgQ: ix=%d\n", dname, ix);
    int status; 
    msgq_t messg;

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

    if (!pcmd) return(asynError);

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
    asynStatus status = asynSuccess;
    size_t nbw;

    status = pasynOctetSyncIO->flush(pasynUser);
    status = pasynOctetSyncIO->write(pasynUser, pw, nw, 1, &nbw);
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
    asynStatus status = asynSuccess;
    int eom;
    size_t nbw, nbr;

    status = pasynOctetSyncIO->flush(pasynUser);
    status = pasynOctetSyncIO->writeRead(pasynUser, pw, nw, pr, nr, 1, &nbw, &nbr, &eom);
    return status;
}


/*--- virtual methods -------------------------------------------------------*/
void drvScope::getWaveform(int ch) {
/*-----------------------------------------------------------------------------
 * Virtual function to be supplied by device specific class.
 *---------------------------------------------------------------------------*/
}


void drvScope::getChanPos(int addr) {
/*-----------------------------------------------------------------------------
 * This virtual method is coded to work with the tds3000 series scopes.  It
 * needs to be reimplemented for other scope types, e.g. Rigol.
 * addr is parameter library index or addr+1 channel
 *---------------------------------------------------------------------------*/
    getFloatCh(ixAoChPos, addr+1, _aoChPos);
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


void drvScope::timeDelayStr(int m,int uix){}
void drvScope::setTimePerDiv(double v){}
void drvScope::getChanScl(int ch){}
void drvScope::getTrigLevl(){}
void drvScope::setTrigLevl(int v){}
const char* drvScope::getCommand(int ix){return(NULL);}

const char** drvScope::getCmndList(int cix,uint* ni){
    *ni=0;
    return(NULL);
}

void drvScope::getHSParams(double hs,int* x0,int* np){
    *x0=0; *np=500;
}


void drvScope::saveConfig() {
/*-----------------------------------------------------------------------------
 * This virtual function is reimplemented in a derived class as needed.  It
 * gets an instrument configuration string and writes it to a disk file.
 *---------------------------------------------------------------------------*/
    const char* pcmd = getCommand(ixBoSave);
    asynStatus status;

    if (!pcmd) return;

    status = command(pcmd);
    if (status != asynSuccess) {
        errlogPrintf("%s::saveConfig:command: failed\n", dname);
        return;
    }

    FILE* fd = fopen(_fname, "w");
    if (!fd) {
        errlogPrintf("%s::saveConfig:fopen: failed to open %s\n", dname, _fname);
        return;
    }

    int st = fputs(_rbuf, fd);
    if (st == EOF) errlogPrintf("%s::saveConfig:fputs: failed\n",dname);
    fclose(fd);
}


void drvScope::restoreConfig() {
/*-----------------------------------------------------------------------------
 * This virtual function is reimplemented in a derived class as needed.  It
 * gets instrument configuration string from a disk file and sends it to
 * the instrument.
 *---------------------------------------------------------------------------*/
    FILE* fd = fopen(_fname,"r");
    if (!fd) {
        errlogPrintf("%s::restoreConfig:fopen: failed to open %s\n", dname, _fname);
        return;
    }

    char* p = fgets(_rbuf, DBUF_LEN, fd);
    if (!p) {
        errlogPrintf("%s::restoreConfig:fgets: failed\n", dname);
        return;
    }

    asynStatus status = _write(_rbuf, strlen(_rbuf));
    if (status != asynSuccess){
        errlogPrintf("%s::restoreConfig: failed to write instrument\n", dname);
        return;
    }

    const char* pcmd = getCommand(ixBoInit);
    if (pcmd) {
        status = command(pcmd);
        if (status != asynSuccess)
        errlogPrintf("%s::restoreConfig: failed to initialize\n", dname);
        update();
    }
}
/*--- end virtual methods ---------------------------------------------------*/


int drvScope::_opc() {
/*-----------------------------------------------------------------------------
 * OPC query and returns as a function value result returned.
 *---------------------------------------------------------------------------*/
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
            errlogPrintf("%s::_opc: failed in _wtrd after %d tries\n", dname, tries);
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


asynStatus drvScope::getString(int cix, int pix) {
/*-----------------------------------------------------------------------------
 * Issues a query for a string value and puts the obtained value in
 * parameter library at index pix.
 *---------------------------------------------------------------------------*/
    const char* cmnd = getCommand(cix);
    if (!cmnd) return(asynError);

    char str[32]; 
    int len = strlen(cmnd);

    if (len > 30) return(asynError);

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


asynStatus drvScope::command(const char* cmnd) {
/*-----------------------------------------------------------------------------
 * Issues a command and puts the reply string, if any, in parameter library.
 *---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    char* p;

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


asynStatus drvScope::getInt(int cix, int pix) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value and puts the obtained value in
 * parameter library at index pix.
 *---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    const char* cmnd = getCommand(cix);

    status = getInt(cmnd, pix);

    return status;
}


asynStatus drvScope::getInt(const char* cmnd, int pix) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value and puts the obtained value in
 * parameter library at index pix.
 *---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    int val;

    status = command(_makeQuery(cmnd));
    if(status == asynSuccess) {
        val = atoi(_rbuf);
        status = setIntegerParam(pix, val);
    }

    return status;
}


asynStatus drvScope::getFloat(int cix, int pix) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value and puts the obtained value in
 * parameter library at index pix.
 *---------------------------------------------------------------------------*/
    const char* cmnd = getCommand(cix);
    if(!cmnd) return(asynError);
    return getFloat(cmnd, pix);
}


asynStatus drvScope::getFloat(const char* cmnd, int pix) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value and puts the obtained value in
 * parameter library at index pix.
 *---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    double val;
    char str[32];
    int len = strlen(cmnd);

    if (len > 30) return(asynError);

    strcpy(str, cmnd);

    if (!strchr(str, '?')) {
        strcat(str, "?");
    }

    status = _wtrd(str, strlen(str), _rbuf, DBUF_LEN);
    if(status == asynSuccess) {
        val = atof(_rbuf);
        status = setDoubleParam(pix, val);
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


asynStatus drvScope::getBinary(int cix, int pix) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value and puts the obtained value in
 * parameter library at index pix.  The reply is a string and a numeric
 * value is the index in a list of strings.
 *---------------------------------------------------------------------------*/
    const char* cmnd = getCommand(cix);
    if (!cmnd) return asynError;

    uint ni;
    const char** list = getCmndList(cix, &ni);
    if (!list) return asynError;

    if (debug) printf("%s: getBinary: pix=%d, cmnd=%s\n", dname, pix, cmnd);
    
    return getBinary(cmnd, pix, list, ni);
}


asynStatus drvScope::getBinary(const char* cmnd, int pix, const char** list, int ni) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value and puts the obtained value in
 * parameter library at index pix.  The reply is a string and a numeric
 * value is the index in a list of strings.
 *---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    int val;
    char str[32];
    int len = strlen(cmnd);

    if (len > 30) return(asynError);
    strcpy(str, cmnd);

    if (!strchr(str, '?')) {
        strcat(str,"?");
    }

    status = _wtrd(str, strlen(str), _rbuf, DBUF_LEN);
    if (status != asynSuccess) return(status);

    val = _find(_rbuf, list, ni);
    status = setIntegerParam(pix, val);
    
    return status;
}


asynStatus drvScope::getIntCh(int cix, int i, int pix) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value for channel i and puts the obtained
 * value in parameter library at index pix.
 *---------------------------------------------------------------------------*/
    const char* cmnd = getCommand(cix);
    if (!cmnd) return(asynError);
    return getIntCh(cmnd, i, pix);
}


asynStatus drvScope::getIntCh(const char* cmnd, int i, int pix) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value for channel i and puts the obtained
 * value in parameter library at index pix.
 *---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    int val;
    char str[32];
    int len = strlen(cmnd);

    if ((len > 30) || (i < 1) || (i > 4)) return(asynError);

    sprintf(str, cmnd, i);

    if (!strchr(str, '?')) {
        strcat(str,"?");
    }

    status = _wtrd(str, strlen(str), _rbuf, DBUF_LEN);
    if (status == asynSuccess){
        val = atoi(_rbuf);
        status = setIntegerParam(i-1 ,pix, val);
    }

    return status;
}


asynStatus drvScope::getFloatCh(int cix, int i, int pix) {
/*-----------------------------------------------------------------------------
 * Issues a query for an float value for channel i and puts the obtained
 * value in parameter library at index pix.
 *---------------------------------------------------------------------------*/
    const char* cmnd = getCommand(cix);
    if (!cmnd) return asynError;
    return getFloatCh(cmnd, i, pix);
}


asynStatus drvScope::getFloatCh(const char* cmnd, int i, int pix) {
/*-----------------------------------------------------------------------------
 * Issues a query for an float value for channel i and puts the obtained
 * value in parameter library at index pix.
 *---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    double val;
    char str[32];
    int len = strlen(cmnd);

    if ((len > 30) || (i < 1) || (i > 4)) {
        return asynError;
    }

    sprintf(str, cmnd, i);

    if (!strchr(str,'?')) {
        strcat(str,"?");
    }

    status = _wtrd(str, strlen(str), _rbuf, DBUF_LEN);
    if (status == asynSuccess) {
        val = atof(_rbuf);
        status = setDoubleParam(i-1, pix, val);
        callParamCallbacks(i-1);
    }

    return status;
}


asynStatus drvScope::getBinaryCh(int cix, int i, int pix) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value for channel i and puts the obtained
 * value in parameter library at index pix.  The reply is a string and a numeric
 * value is the index in a list of strings.
 *---------------------------------------------------------------------------*/
    const char* cmnd = getCommand(cix);
    if (!cmnd) return asynError;

    uint ni;
    const char** list = getCmndList(cix, &ni);

    if (!list) return(asynError);

    return getBinaryCh(cmnd, i, pix, list, ni);
}


asynStatus drvScope::getBinaryCh(const char* cmnd, int i, int pix, const char** list, int ni) {
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value for channel i and puts the obtained
 * value in parameter library at index pix.  The reply is a string and a
 * numeric value is the index in a list of strings.
 *---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    int val = 0;
    char str[32];
    int len = strlen(cmnd);

    if ((len > 30) || (i < 1) || (i > 4)) { 
        return asynError;
    }

    sprintf(str, cmnd, i);

    if (!strchr(str, '?')) {
        strcat(str, "?");
    }

    status = _wtrd(str, strlen(str), _rbuf, DBUF_LEN);
    if (status != asynSuccess) return asynError;

    val = _find(_rbuf, list, ni);
    if (val >= 0) {
        status = setIntegerParam(i-1, pix, val);
    }

    return status;
}


void drvScope::setBinaryCh(int ix, int ch, int cix) {
/*-----------------------------------------------------------------------------
 * ix is a bit position that was selected, this corresponds to an index
 * into list of size ni from which we extract the value needed to construct
 * the command.
 *---------------------------------------------------------------------------*/
    const char* cmnd = getCommand(cix);

    if (!cmnd) return;

    uint ni;
    const char** list = getCmndList(cix,&ni);

    if (list) {
        setBinaryCh(ix, ch, cmnd, list, ni);
    }
}


void drvScope::setBinaryCh(int ix, int ch, const char* cmnd, const char** list, int ni) {
/*-----------------------------------------------------------------------------
 * ix is a bit position that was selected, this corresponds to an index
 * into list of size ni from which we extract the value needed to construct
 * the command.
 *---------------------------------------------------------------------------*/
    char str[32];

    if ((ix < 0) || (ix >= ni)) return;

    sprintf(str, cmnd, ch+1);
    strcat(str, " ");
    strcat(str, list[ix]);
    command(str);
}


void drvScope::setBinary(int ix, int cix){
/*-----------------------------------------------------------------------------
 * ix is a bit position that was selected, this corresponds to an index
 * into list of size ni from which we extract the value needed to construct
 * the command.
 *---------------------------------------------------------------------------*/
    const char* cmnd = getCommand(cix);

    if (!cmnd) return;

    uint ni;
    const char** list = getCmndList(cix, &ni);

    if (list) {
        setBinary(ix, cmnd, list, ni);
    }
}


void drvScope::setBinary(int ix, const char* cmnd, const char** list, int ni) {
/*-----------------------------------------------------------------------------
 * ix is a bit position that was selected, this corresponds to an index
 * into list of size ni from which we extract the value needed to construct
 * the command.
 *---------------------------------------------------------------------------*/
    char str[32];

    if ((ix < 0) || (ix >= ni)) return;

    sprintf(str, "%s %s", cmnd, list[ix]);
    command(str);
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
    char str[32];

    sprintf(str, "%s %d", cmnd, v);

    if (debug) printf("drvScope::setInt: str=%s, cix=%d, pix=%d\n", str, cix, pix);

    command(str);

    if (!pix) return;

    getInt(cix, pix);
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


void drvScope::_selectChan(int ch) {
/*-----------------------------------------------------------------------------
 * Select channel ch, used to control channel trace position with a slider.
 *---------------------------------------------------------------------------*/
    int i, v;
    double y;

    _posInProg = 0;

    if ((ch < 0) || (ch >= MAX_ADDR)) return;

    getIntegerParam(ch, _boChOn, &v);

    if (!v) {
        setIntegerParam(ch, _boChSel, 1);
        setIntegerParam(ch, _boChSel, 0);
        callParamCallbacks(ch);
        return;
    }

    for(i=0; i<MAX_ADDR; i++) {
        if (i == ch) {
            v = 1;
        } else {
            v = 0;
        }
        setIntegerParam(i, _boChSel, v);
        callParamCallbacks(i);
    }

    getDoubleParam(ch, _aoChPos, &y);
    _setPosSlider(y);
    _chSel = ch;
}


void drvScope::_selectChannel() {
/*-----------------------------------------------------------------------------
 * The first enabled channel is selected to be controlled by the pos slider
 *---------------------------------------------------------------------------*/
    int i, v; 
    static int firsttime = 1;

    getIntegerParam(_chSel, _boChOn, &v);

    if (v && !firsttime) return;

    firsttime = 0;

    for(i=0; i<MAX_ADDR; i++) {
        _selectChan(i);
    }
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
            getBinaryCh(jx, ch, ix);
            break;
        case ixMbboChCpl:
            getBinaryCh(jx, ch, ix);
            break;
        case ixAoChScl:
            getFloatCh(jx, ch, ix);
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


asynStatus drvScope::writeOctet(asynUser* pasynUser, const char* v, size_t nc, size_t* nActual) {
/*-----------------------------------------------------------------------------
 * This method overrides the virtual method in asynPortDriver.
 *---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    int ix, addr;

    status = getAddress(pasynUser, &addr);
    if(status != asynSuccess) return status;

    ix = pasynUser->reason - _firstix;
    switch(ix) {
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


asynStatus drvScope::putIntCmnds(int ix, int addr, int v) {
/*-----------------------------------------------------------------------------
 * This routine is called from the pollerThread routine, when it receives
 * a message on the message queue.
 * ix is the index into parameter library (or the reason)
 * addr is channel or address
 * v is a possible integer set value.
 *---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    char cmnd[32]; 
    const char* pcmd;
    int jx = ix - _firstix;
    pcmd = getCommand(jx);

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
            setBinaryCh(v, addr, jx);
            break;
        case ixMbboChCpl:
            setIntegerParam(addr, ix, v);
            setBinaryCh(v, addr, jx);
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
            _chPosTimer->start(0.2);
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


void drvScope::setChanPosition() {
/*-----------------------------------------------------------------------------
 * gets called from time expire function at the end of slider move.
 *---------------------------------------------------------------------------*/
    setChanPos(_chSel, _chPos);
    //  getFloatCh(ixAoChPos,_chSel+1,_aoChPos);
    getChanPos(_chSel);
    getTrigLevl();
}


asynStatus drvScope::writeInt32(asynUser* pau, epicsInt32 v) {
/*-----------------------------------------------------------------------------
 * This method overrides the virtual method in asynPortDriver.  Here we service
 * all write requests comming from EPICS records.
 * Parameters:
 *  pau         (in) structure containing addr and reason.
 *  v           (in) this is the command index, which together with
 *              pau->reason define the command to be sent.
 *---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    int on, ix, jx, addr;

    status = getAddress(pau, &addr);
    if (status != asynSuccess) return(status);

    ix = pau->reason;
    jx = ix - _firstix;

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

asynStatus drvScope::putFltCmnds(int ix, int addr, float v) {
/*-----------------------------------------------------------------------------
 * This routine is called from the pollerThread routine, when it receives
 * a message on the message queue.
 * ix is the index into parameter library (or the reason)
 * addr is channel or address
 * v is a possible integer set value.
 *---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    char cmnd[32];
    int jx = ix-_firstix;
    const char* pcmd = getCommand(jx);

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


asynStatus drvScope::writeFloat64(asynUser* pau, epicsFloat64 v) {
/*-----------------------------------------------------------------------------
 * This method overrides the virtual method in asynPortDriver.  Here we service
 * all write requests comming from EPICS records.
 * Parameters:
 *  pau         (in) structure containing addr and reason.
 *  v           (in) this is the command index, which together with
 *              pau->reason define the command to be sent.
 *---------------------------------------------------------------------------*/
    asynStatus status = asynSuccess;
    int ix, addr;
    float fv = v;

    status = getAddress(pau,&addr);
    if (status != asynSuccess) return(status);

    ix = pau->reason - _firstix;
    switch(ix) {
        case ixAoPTMO:
            _pollT = v;
            break;
        default:
            putInMessgQ(enPutFlt, ix, addr, 0, fv);
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
    static int first = 1;
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
        getWaveform(0);
        getWaveform(1);
        getWaveform(2);
        getWaveform(3);
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

    first = 0;
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
    static int first[] = {1,1,1,1};
    int i = ch-1, v;
    double dv;

    getIntCh(ixBoChOn, ch ,_boChOn);
    _selectChannel();

    getIntegerParam(i ,_boChOn, &v);
    if ((!v) && (!first[i])) return;
    first[i] = 0;

    getBinaryCh(ixBoChImp, ch, _boChImp);
    getBinaryCh(ixMbboChCpl, ch, _mbboChCpl);
    getFloatCh(ixAoChScl, ch, _aoChScl);
    getChanScl(ch);
    getChanPos(ch-1);
    getDoubleParam(i, _aoChPos, &dv);
    if (i == _chSel) {
        _setPosSlider(dv);
    }
}


void drvScope::update() {
/*----------------------------------------------------------------------------
 * This is called at startup and periodically to stay in sync with user
 * changing settings on the instrument.
 *--------------------------------------------------------------------------*/
    int i, ch;
    static int firsttime = 1;
    double dv;

    getFloat(ixAiTimDiv, _aiTimDiv);
    getDoubleParam(_aiTimDiv, &dv);
    setTimePerDiv(dv);
    updateUser();

    for (i=0; i<MAX_ADDR; i++) {
        ch = i+1;
        _getChanOn(ch);
        callParamCallbacks(i);
    }

    getFloat(ixAoTimDly, _aoTimDly);
    getDoubleParam(_aoTimDly, &dv);
    timeDelayStr(dv);
    getInt(ixBoTimDlySt, _boTimDlySt);
    getFloat(ixAoTrPos, _aoTrPos);
    getFloat(ixAoTrLev, _aoTrLev);
    getTrigLevl();
    getFloat(ixAoTrHOff, _aoTrHOff);
    callParamCallbacks(0);
    firsttime = 0;
}

void drvScope::updateUser() {
/*----------------------------------------------------------------------------
 *--------------------------------------------------------------------------*/
}

void drvScope::afterInit() {
/*----------------------------------------------------------------------------
 *--------------------------------------------------------------------------*/
    const char* pcmd = getCommand(ixBoInit);

    if (pcmd) command(pcmd);

    putInMessgQ(enQuery, _wfIdn, 0, 0);
    putInMessgQ(enQuery, _siIpAddr, 0, 0);

    for (int i=0; i<MAX_ADDR; i++) {
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

