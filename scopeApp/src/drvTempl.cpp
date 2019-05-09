/* drvTempl.cc
 * This device driver is derived from asynPortDriver class written
 * by Mark Rivers at APS.
 * Started on 9/26/2012, zms.
 *---------------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

#include <cantProceed.h>
#include <epicsTypes.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <errlog.h>
#include <initHooks.h>
#include <epicsExit.h>
#include <epicsExport.h>
#include <envDefs.h>
#include <iocsh.h>
#include <asynOctetSyncIO.h>

#include "drvTempl.h"

#define STRLEN	128
static const char *dname="drvTempl";
static char _str[STRLEN];
static drvTempl* pthis;


extern "C"{
static void exitHndlC( void* pvt){
  drvTempl* pthis=(drvTempl*)pvt;
  pthis->exitHndl();
}
static void taskThreadC( void* p){
  drvTempl* pthis=(drvTempl*)p;
  pthis->taskThread();
}
}

char* drvTempl::_getTime(){
/*-----------------------------------------------------------------------------
 * constructs a time string and returns pointer to it as a function value.
 *---------------------------------------------------------------------------*/
  static char _time[64];
  ulong nsec; epicsTimeStamp etime;
  struct tm tt; int msec,year;

  epicsTimeGetCurrent( &etime);
  epicsTimeToTM( &tt,&nsec,&etime);
  msec=(nsec+999999)/1000000; year=tt.tm_year+1900;
  sprintf( _time,"%d-%02d-%02d-%02d-%02d-%02d",year,tt.tm_mon+1,
        tt.tm_mday,tt.tm_hour,tt.tm_min,tt.tm_sec);
  return(_time);
}
void drvTempl::_drvInit(){
  if(_isinit) return;
  _isinit=1;
  ellInit(&_alist);
}
void drvTempl::taskThread(){
/*-----------------------------------------------------------------------------
 * Process tasks, one at a time.
 *---------------------------------------------------------------------------*/
  static const char* iam="taskThread";
printf( "%s::%s:==================\n",dname,iam);
  epicsThreadSleep(1.0);
  while(1){
    epicsThreadSleep(_timeout);
  }
}
static void inithooks( initHookState state){
/*--------------------------------------------------------------------------*/
  switch(state){
    case initHookAtEnd:	pthis->afterInit(); break;
    default:		break;
  }
}
void drvTempl::exitHndl(){
/*-----------------------------------------------------------------------------
 *---------------------------------------------------------------------------*/
  errlogPrintf( "%s::exitHndl: Clear ID\n",dname);
}
void drvTempl::_message( char* p){
/*-----------------------------------------------------------------------------
 * Puts a null terminated string in p in the _wfMessage waveform record.
 *---------------------------------------------------------------------------*/
  int n=MIN(STRLEN,MAX(0,strlen(p)+1));
  if(!n) return;
  setStringParam( _wfMessage,p);
  callParamCallbacks();
  *p=0;
}
void drvTempl::afterInit(){
/*-----------------------------------------------------------------------------
 * Called from inithooks.  Here we read bunch of stuff needed to properly
 * initialize various records to be consistent with the state of the chassis.
 *---------------------------------------------------------------------------*/
  errlogPrintf( "%s::afterInit: all done\n",dname);
  return;
}
asynStatus drvTempl::_writeRead( const char* pw,size_t nw,char* pr,size_t nr,
        size_t* nbo,size_t* nbi){
/*-----------------------------------------------------------------------------
 * Here we perform an "atomic" write and read operation sequence.  Parameters:
 *  pw  buffer that has data to be written,
 *  nw  number of bytes of data in pw,
 *  pr  buffer into which data will be read in,
 *  nr  size of read buffer in bytes,
 *  nbo number of bytes of data actually written out,
 *  nbi number of bytes of data actually read.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int eom;

  stat=pasynOctetSyncIO->flush( _pvt);
//printf( "%s::_writeRead:flush: stat=%d\n",dname,stat);
  stat=pasynOctetSyncIO->writeRead( _pvt,pw,nw,pr,nr,1,nbo,nbi,&eom);
//printf( "%s::_writeRead: nr=%d,stat=%d,nbo=%d,nbi=%d,eom=%d\n",
//dname,nr,stat,*nbo,*nbi,eom);
  return(stat);
}
asynStatus drvTempl::_wtrd( const char* cmnd){
/*-----------------------------------------------------------------------------
 * sends cmnd string to the device, reads the reply into _dbuf and puts
 * the cleaned up null terminated version in _dbcl buffer.
 *---------------------------------------------------------------------------*/
  asynStatus stat; size_t nbo,nbi; char* p1; char* p2; char* p3;
  stat=_writeRead( cmnd,strlen(cmnd),_buf,NELM,&nbo,&nbi);
//printf( "%s::_wtrd: stat=%d,%s\n",dname,stat,_buf);
  if((stat!=asynSuccess)||!nbi||(nbi>NELM)) return(stat);
  p1=_buf; p3=_buf+NELM-1;
  p2=strchr(p1,'\r');
  if(!p2) p2=_buf+nbi;
  *p2=0;
  return(stat);
}
asynStatus drvTempl::readInt32( asynUser* pau,epicsInt32* v){
/*-----------------------------------------------------------------------------
 * Reimplementation of asynPortDriver virtual function.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int ix,addr;
  stat=getAddress(pau,&addr); if(stat!=asynSuccess) return(stat);
  if(addr<0||addr>=NCHAN) return(asynError);
  ix=pau->reason-_firstix;
  switch( ix){
    case ixLiPollTmo:	*v=_timeout*1000.0; break;
    default:    stat=asynError; break;
  }
  callParamCallbacks(0);
  return(stat);
}
asynStatus drvTempl::readFloat64( asynUser* pau,epicsFloat64* v){
/*-----------------------------------------------------------------------------
 * Reimplementation of asynPortDriver virtual function.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int ix,addr;
  stat=getAddress(pau,&addr); if(stat!=asynSuccess) return(stat);
  if(addr<0||addr>=NCHAN) return(asynError);
  ix=pau->reason-_firstix;
  switch( ix){
    default:	stat=asynError; break;
  }
  callParamCallbacks(0);
  return(stat);
}
asynStatus drvTempl::readInt16Array( asynUser* pau,epicsInt16* v,
                        size_t nelm,size_t* nin){
/*-----------------------------------------------------------------------------
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int ix,addr;

  stat=getAddress(pau,&addr); if(stat!=asynSuccess) return(stat);
  ix=pau->reason-_firstix;
  switch( ix){
    default:            stat=asynError; break;
  }
  return(stat);
}
asynStatus drvTempl::writeInt16Array( asynUser* pau,epicsInt16* v,size_t n){
/*-----------------------------------------------------------------------------
 * This overrides the function in asynPortDriver...
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int ix,addr;

  stat=getAddress(pau,&addr); if(stat!=asynSuccess) return(stat);
  ix=pau->reason-_firstix;
  switch(ix){
    default:	stat=asynError; break;
  }

  return(stat);
}
asynStatus drvTempl::writeInt8Array( asynUser* pau,epicsInt8* v,size_t n){
/*-----------------------------------------------------------------------------
 * Replace the function in asynPortDriver...
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int ix,addr;

  stat=getAddress(pau,&addr); if(stat!=asynSuccess) return(stat);
  ix=pau->reason-_firstix;

  switch(ix){
    default:		stat=asynError; break;
  }
  stat=callParamCallbacks();
  return(stat);
}
asynStatus drvTempl::writeInt32( asynUser* paUser,epicsInt32 v){
/*-----------------------------------------------------------------------------
 * This method queues a write message internally.  The actual write s done in
 * the ioTask.
 * Parameters:
 *  paUser	(in) structure containing addr and reason.
 *  v	(in) this is the command index, which together with
 *		paUser->reason define the command to be sent.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int reason=paUser->reason-_firstix;

//printf( "%s::writeInt32:reason=%d,v=%d\n",dname,reason,v);
  switch( reason){
    case ixLoPollTmo:	_timeout=(float)v/1000.0;
			setIntegerParam( _liPollTmo,v);
			break;
    default:		stat=asynError; break;
  }
  stat=callParamCallbacks();
  return(stat);
}
asynStatus drvTempl::writeFloat64( asynUser* pau,epicsFloat64 v){
/*-----------------------------------------------------------------------------
 * This method overrides the virtual method in asynPortDriver.  Here we service
 * all write requests comming from EPICS records.
 * Parameters:
 *  pau         (in) structure containing addr and reason.
 *  v           (in) this is the command index, which together with
 *              pau->reason define the command to be sent.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int ix,addr;

  stat=getAddress(pau,&addr); if(stat!=asynSuccess) return(stat);
  ix=pau->reason-_firstix;
  switch( ix){
    default:		stat=asynError; break;
  }
  return(stat);
}

drvTempl::drvTempl(const char* port,const char* udp,int addr):
	asynPortDriver( port,NCHAN,N_PARAMS,
		asynInt8ArrayMask|asynInt16ArrayMask|asynInt32Mask|
		asynFloat64Mask|asynOctetMask|asynDrvUserMask,
		asynInt8ArrayMask|asynInt16ArrayMask|asynInt32Mask|
		asynFloat64Mask|asynOctetMask,
		ASYN_CANBLOCK|ASYN_MULTIDEVICE,1,0,0){
/*-----------------------------------------------------------------------------
 * Constructor for the drvTempl class. Calls constructor for the asynPortDriver
 * base class. Where
 *   port The name of the asyn port driver to be created.
 *   udp  is the communication port.
 *   addr is address
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
  int nbts=strlen(port)+strlen(udp)+2,stat;
  _port=(char*)callocMustSucceed( nbts,sizeof(char),dname);
  _udp=(char*)(_port+strlen(port)+1);
  strcpy((char*)_port,port);
  strcpy((char*)_udp,udp);
  pthis=this;

  stat=pasynOctetSyncIO->connect( udp,0,&_pvt,0);
  if(stat!=asynSuccess)
    printf( "%s::%s:connect: failed to connect to port %s\n",
                dname,dname,udp);
  else  printf( "%s::%s:connect: connected to port %s\n",dname,dname,udp);

  createParam( siNameStr,	asynParamOctet,		&_siName);
  createParam( wfMessageStr,    asynParamOctet,		&_wfMessage);
  createParam( liPollTmoStr,	asynParamInt32,		&_liPollTmo);
  createParam( loPollTmoStr,	asynParamInt32,		&_loPollTmo);
  
  _drvInit();
  _timeout=0.5;
  _pmq=new epicsMessageQueue( NMSGQ,sizeof(msgq_t));
  epicsThreadCreate( dname,epicsThreadPriorityHigh,
                epicsThreadGetStackSize(epicsThreadStackMedium),
                (EPICSTHREADFUNC)taskThreadC,this);

  epicsAtExit( exitHndlC,this);
  printf( "%s::%s: _locPort=%s configured\n",dname,dname,_port);
}

// Configuration routine.  Called directly, or from the iocsh function below

extern "C" {

int drvTemplConfigure( const char* port,const char* udp,int addr){
/*------------------------------------------------------------------------------
 * EPICS iocsh callable function to call constructor for the drvTempl class.
 *  port The name of the asyn port driver to be created.
 *  udp  is the communication port name.
 *  addr is the SY403 addreess.
 *  nchan is the number of active (defined) channels.
 *----------------------------------------------------------------------------*/
  new drvTempl(port,udp,addr);
  return(asynSuccess);
}

static const iocshArg initArg0={"port",iocshArgString};
static const iocshArg initArg1={"udp",iocshArgString};
static const iocshArg initArg2={"addr",iocshArgInt};
static const iocshArg* const initArgs[]={&initArg0,&initArg1,&initArg2};
static const iocshFuncDef initFuncDef={"drvTemplConfigure",3,initArgs};
static void initCallFunc(const iocshArgBuf *args){
  drvTemplConfigure( args[0].sval,args[1].sval,args[2].ival);
}

void drvTemplRegister(void){
  iocshRegister(&initFuncDef,initCallFunc);
  initHookRegister(&inithooks);
}
epicsExportRegistrar(drvTemplRegister);
}
