/* drvTDS.cpp
 * Asyn driver to control Tektronix TDS 3000 series scopes.  This is a
 * subclass of asynPortDriver, which was created by Mark Rivers.
 * Started on 10/28/2011, zms.
 *---------------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

#include <ellLib.h>
#include <cantProceed.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsExport.h>
#include <initHooks.h>
#include <iocsh.h>
#include <asynOctetSyncIO.h>

#include "asynPortDriver.h"
#include "drvTDS.h"

static const char* EnablCmnd=	"SEL:CH%d";
static const char* PositCmnd=	"CH%d:POS";
static const char* PositCmndW=	"CH%d:POS %f";
static const char* OffstCmnd=	"CH%d:OFFS";
static const char* ImpedCmnd=	"CH%d:IMP";
static const char* impedList[]={"FIFTY","MEG"};
static const char* CouplCmnd=	"CH%d:COUP";
static const char* couplList[]={"DC","AC","GND"};
static const char* ScaleCmnd=	"CH%d:SCA";
static const char* scaleList[]={"1.0E-3","2.0E-3","5.0E-3","1.0E-2","2.0E-2",
	"5.0E-2","1.0E-1","2.0E-1","5.0E-1","1.0E0","2.0E0","5.0E0","1.0E1"};
static const char* TraceCmnd=	"DAT:SOU CH%d; :WAVF";

static const char* TDlyCmnd=	"HOR:DEL:TIM";
static const char* TDlyStCmnd=	"HOR:DEL:STATE";
static const char* TDivCmnd=	"HOR:MAI:SCA";
static const char* tdivUList[]={"ns","us","ms","s"};
static const char* tdivVList[]={"1","2","4","10","20","40","100","200","400"};
static const char* TRefCmnd=	"HOR:TRIG:POS";
static const char* TrigLevCmnd=	"TRIG:A:LEV";
static const char* HldOffCmnd=	"TRIG:A:HOL";
static const char* TModeCmnd=	"TRIG:A:MOD";
static const char* tmodeList[]={"NORMAL","AUTO"};
static const char* TrigSoCmnd=	"TRIG:A:EDG:SOU";
static const char* trigSoList[]={"CH1","CH2","CH3","CH4","LINE","VERTICAL",
				"EXT10","EXT"};
static const char* TrigSloCmnd=	"TRIG:A:EDG:SLO";
static const char* trigSloList[]={"FALL","RISE"};
static const char* AcqStatCmnd=	"ACQ:STATE";
//static const char* acqStatList[]={"RUNSTOP","SEQUESCE"};
static const char* TrigStaCmnd=	"TRIG:STATE";
static const char* trigStaList[]={"AUTO","ARMED","READY","SAVE","TRIGGER"};
static const char* RunCmnd=	"ACQ:STATE RUN";
static const char* StopCmnd=	"ACQ:STATE STOP";
static const char* FLockCmnd=	"LOCK";
static const char* fLockList[]={"ALL","NONE"};
static const char* BusyCmnd=	"BUSY";
static const char* ResetCmnd=	"*RST";
//static const char* SetupCmnd=	"SET";
//static const char* WSetupCmnd=	" ";
//static const char* RecallCmnd=	"*RCL";
//static const char* StoreCmnd=	"*SAV";
static const char* DSourceCmnd=	"DATA:SOU";
static const char* DAutoCCmnd=	"DIS:PIC:AUTOC";
static const char* DBrightCmnd=	"DIS:PIC:BRI";
static const char* DPictStCmnd=	"DIS:PIC:STATE";
static const char* DIntensCmnd=	"DIS:INTENSIT:BACKL";
static const char* dIntensList[]={"HIGH","MEDIUM","LOW"};
static const char* IntensWfCmnd="DIS:INTENSIT:WAVE";
static const char* InitCmnd=	"*CLS; :DAT:ENC RIB; :HOR:RECORDL 500; :HEAD OFF; :VERB ON; :DAT:STAR 1; :DAT:STOP 500; HOR:DEL:STATE 1";
static const char* DEncodCmnd=	"DAT:ENC";
static const char* dEncodList[]={"ASCII","RIBINARY",
			"RPBINARY","SRIBINARY","SRPBINARY"};
static const char* RecLenCmnd=	"HOR:RECO";
static const char* HeaderCmnd=	"HEAD";
static const char* DWidthCmnd=	"DAT:WID";
static const char* DStartCmnd=	"DAT:STAR";
static const char* DStopCmnd=	"DAT:STOP";
static const char* DeseCmnd=	"DESE";
static const char* EsrCmnd=	"*ESR";
static const char* EseCmnd=	"*ESE";
static const char* EvQCmnd=	"EVENT";
static const char* ClsCmnd=	"*CLS";
static const char* OPCCmnd=	"*OPC";
static const char* StbCmnd=	"*STB";
//static const char* EventCmnd=	"ALLE";
//static const char* MessgCmnd=	"EVM";
static const char* EvQtyCmnd=	"EVQ";

static const int hsc[]={   1,   2,   4,  10};
static const int hsx[]={ 225, 200, 150,   0};
static const int hnp[]={  51, 101, 201, 500};

static drvTDS*	_this;
static const char *dname="drvTDS";

extern "C"{
static void pollerThreadC( void* pPvt){
  drvTDS* _this=(drvTDS*)pPvt;
  _this->pollerThread();
}
}
static void __getHSParams( double hs,int* x0,int* np){
/*-----------------------------------------------------------------------------
 * Returns starting point in x0 and number of points in np where data should
 * be extracted from the raw waveform data.  hs is the horizontal scale in
 * seconds.
 *---------------------------------------------------------------------------*/
  int ix,h=((hs*1e9)+0.5);
  for( ix=0; ix<4&&(hsc[ix]!=h); ix++);
  if(ix>=4) ix=3;
  *x0=hsx[ix];
  *np=hnp[ix];
}

static void inithooks( initHookState state){
/*--------------------------------------------------------------------------*/
  switch(state){
    case initHookAtEnd: _this->afterInit(); break;
    default:            break;
  }
}

void drvTDS::pollerThread(){
/*-----------------------------------------------------------------------------
 * This function runs in a separate thread.  It waits for the poll time.  
 * Reads a list of registers and it does callbacks to all
 * clients that have registered with registerDevCallback
 *---------------------------------------------------------------------------*/
  static const char* iam="pollerThread"; msgq_t msgq; int stat;
printf( "%s::%s:================================\n",dname,iam);
  while(1){
    stat=_pmq->tryReceive( &msgq,sizeof(msgq));
    if(stat==-1){
      if(_rdtraces) _getTraces();
      epicsThreadSleep( _pollT);
    }
    else switch(msgq.type){
      case enPutInt:    _putIntCmnds( msgq.ix,msgq.addr,msgq.ival);
                        break;
      case enQuery:     _getCmnds( msgq.ix,msgq.addr);
			break;
      case enPutFlt:    _putFltCmnds(msgq.ix,msgq.addr,msgq.fval);
			break;
    }
  }
}
void drvTDS::_message( const char* m){
/*-----------------------------------------------------------------------------
 * Constructs and posts a message.
 *---------------------------------------------------------------------------*/
  int len;
  strncpy( _mbuf,_putil->getTime(),MSGNB); _mbuf[MSGNB-1]=0;
  if(strlen(_mbuf)<(MSGNB-20)){
    strcat( _mbuf," ");
    len=strlen(_mbuf);
    strncat( _mbuf,m,MSGNB-len-1);
    _mbuf[MSGNB-1]=0;
  }
  setStringParam( _wfMessg,_mbuf);
  callParamCallbacks();
}
void drvTDS::_putInMessgQ( int tp,int ix,int addr,int iv,float fv){
/*-----------------------------------------------------------------------------
 * Construct a message and put in the message queue.
 *---------------------------------------------------------------------------*/
  int stat; msgq_t messg;
  messg.type=tp;
  messg.ix=ix;
  messg.addr=addr;
  messg.ival=iv;
  messg.fval=fv;
  stat=_pmq->trySend( &messg,sizeof(messg));
  if(!stat) _mqSent++; else _mqFailed++;
  setIntegerParam( _liMsgQS,_mqSent);
  setIntegerParam( _liMsgQF,_mqFailed);
  callParamCallbacks(0);
}
asynStatus drvTDS::_write( const char* pw,size_t nw){
/*-----------------------------------------------------------------------------
 * Here we perform an "atomic" write.  Parameters:
 *  pw  buffer that has data to be written,
 *  nw  number of bytes of data in pwb,
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; size_t nbw;

  stat=pasynOctetSyncIO->flush( _aPvt);
  stat=pasynOctetSyncIO->write( _aPvt,pw,nw,1,&nbw);
  return(stat);
}
asynStatus drvTDS::_wtrd( const char* pw,size_t nw,char* pr,size_t nr){
/*-----------------------------------------------------------------------------
 * Here we perform an "atomic" write and read operation sequence.  Parameters:
 *  pw  buffer that has data to be written,
 *  nw  number of bytes of data in pwb,
 *  pr  buffer into which data will be read in,
 *  nr  size of read buffer in bytes,
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int eom; size_t nbw,nbr;

  stat=pasynOctetSyncIO->flush( _aPvt);
  stat=pasynOctetSyncIO->writeRead( _aPvt,pw,nw,pr,nr,1,&nbw,&nbr,&eom);
  return(stat);
}
int drvTDS::_opc(){
/*-----------------------------------------------------------------------------
 * OPC query and returns as a function value result returned.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int ct=0,i=0,v=0; char* p;
  _rbuf[0]=0;
  while(1){
    stat=_wtrd("*OPC?",5,_rbuf,DBUF_LEN);
    if(stat==asynSuccess){
      p=strchr( _rbuf,'\n');
      if(p) *p=0;
      v=atoi(_rbuf);
      if(v==1) break;
      if((++ct)>10){ v=0; break;}
      epicsThreadSleep(0.01);
    }
    else if((++i)>1){
      errlogPrintf( "%s::_opc: failed in _wtrd after %d tries\n",dname,i);
      break;
    }
  }
  return(v);
}
void drvTDS::_getIdn(){
/*---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; char* p;
  stat=_wtrd( "*IDN?",5,_rbuf,DBUF_LEN);
  if(stat==asynSuccess){
    p=strchr( _rbuf,'\n');
    if(p) *p=0;
    stat=setStringParam( _wfIdn,_rbuf);
  }
}
void drvTDS::_getIpAddr(){
/*---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; char* p0; char* p;
  stat=_wtrd( "ETHER:IPADD?",12,_rbuf,DBUF_LEN);
  if(stat==asynSuccess){
    p0=_rbuf;
    p0=strchr( p0,'"');
    if(p0){
      p=strchr( ++p0,'"');
      if(p) *p=0;
    }
    else{
      p=strchr( p0,'\n');
      if(p) *p=0;
    }
    stat=setStringParam( _siIpName,p0);
  }
}
void drvTDS::_setTimePerDiv( epicsFloat64 v){
/*-----------------------------------------------------------------------------
 * v is an analog time/div value.  This is parsed and two integer values are
 * calculated, one is "mantissa" like 1,2,4,10,20,40,100,200,400, while the
 * other is an index to the unit (ns,us,ms,s).  Both are set in the parameter
 * library for the corresponding mbbi/mbbo records.
 * Changed 1.0e-6 to 0.9e-6, etc, because if(v<1.0e-6) was returning true when
 * v=1e-6, therefore calculating wrong mantisa and exponent.
 *---------------------------------------------------------------------------*/
  int mix,uix,m,x0,np; char str[16];
  if(v<0.9e-6){ uix=0; m=(int)((v*1000.0)*1000000.0+0.5);}
  else if(v<0.9e-3){ uix=1; m=(int)(v*1000000.0+0.5);}
  else if(v<0.9){ uix=2; m=(int)(v*1000.0+0.5);}
  else{ uix=3; m=(int)(v+0.5);}
  if(m<=1) mix=0;
  else if(m<=2) mix=1;
  else if(m<=4) mix=2;
  else if(m<=10) mix=3;
  else if(m<=20) mix=4;
  else if(m<=40) mix=5;
  else if(m<=100) mix=6;
  else if(m<=200) mix=7;
  else mix=8;
  sprintf( str,"%d %s/div",m,tdivUList[uix]);
  setIntegerParam( _mbbiTDivV,mix);
  setIntegerParam( _mbbiTDivU,uix);
  setIntegerParam( _mbboTDivV,mix);
  setIntegerParam( _mbboTDivU,uix);
  setStringParam( _siTDiv,str);
  __getHSParams( v,&x0,&np);		// EDM needs to know how many x point
  setIntegerParam( _liXNpts,np);	// of data to dislay.
}
void drvTDS::_setTimePerDiv( uint vix,uint uix){
/*-----------------------------------------------------------------------------
 * Either of the two time per division mbbo records changed.  Indeces to the
 * value (vix) and unit (uix) are used to calculate the new analog value of
 * horizontal time per division.
 *---------------------------------------------------------------------------*/
  char str[16]; int m; double v; char cmnd[32];
  if(vix<0||vix>=SIZE(tdivVList)||uix<0||uix>=SIZE(tdivUList)) return;
  m=atoi(tdivVList[vix]);
  switch(uix){
    case 0:	v=m*1.0e-9; break;
    case 1:	v=m*1.0e-6; break;
    case 2:	v=m*1.0e-3; break;
    case 3:	v=m; break;
  }
  sprintf( str,"%s %s/div",tdivVList[vix],tdivUList[uix]);
  setStringParam( _siTDiv,str);
  setDoubleParam( _aiTDiv,v);
  sprintf( cmnd,"%s %e",TDivCmnd,v);
  _command( cmnd);
}
void drvTDS::_timeDelayStr( double td){
/*-----------------------------------------------------------------------------
 *---------------------------------------------------------------------------*/
  int uix,m; char str[32]; int sign=1;
  if(td<0.0){ sign=(-1); td*=(-1.0);}
  if(td<1.0e-6){ uix=0; m=(int)((td*1000.0)*1000000.0+0.5);}
  else if(td<1.0e-3){ uix=1; m=(int)(td*1000000.0+0.5);}
  else if(td<1.0){ uix=2; m=(int)(td*1000.0+0.5);}
  else{ uix=3; m=(int)(td+0.5);}
  m*=sign;
  sprintf( str,"%d %s",m,tdivUList[uix]);
  setStringParam( _siTDly,str);
}
void drvTDS::_setTimeDelayStr(){
/*-----------------------------------------------------------------------------
 *---------------------------------------------------------------------------*/
 char str[32]; double td;
  getDoubleParam( _aiTDly,&td);
  _timeDelayStr(td);
  sprintf( str,"%s %g",TDlyCmnd,td);
  _command( str);
}
char* drvTDS::_makeQuery( const char* cmnd){
/*-----------------------------------------------------------------------------
 * Appends '?' to cmnd and returns pointer to the query string.
 *---------------------------------------------------------------------------*/
  int len=strlen(cmnd);
  if(len>CMND_LEN-2) return(0);
  strcpy( _cmnd,cmnd); strcat( _cmnd,"?");
  return(_cmnd);
}
asynStatus drvTDS::_getString( const char* cmnd,int pix){
/*-----------------------------------------------------------------------------
 * Issues a query for a string value and puts the obtained value in
 * parameter library at index pix.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; char* p;
  char str[32]; int len=strlen(cmnd);
  if(len>30) return(asynError);
  strcpy( str,cmnd); strcat( str,"?");
  stat=_wtrd( str,strlen(str),_rbuf,DBUF_LEN);
  if(stat==asynSuccess){
    p=strchr( _rbuf,'\n');
    if(p) *p=0;
    stat=setStringParam( pix,_rbuf);
  }
  return(stat);
}
asynStatus drvTDS::_command( const char* cmnd){
/*-----------------------------------------------------------------------------
 * Issues a command and puts the reply string, if any, in parameter library.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; char* p;
  if(!cmnd) return(asynError);
  int len=strlen(cmnd);
  if(strchr(cmnd,'?')){
    stat=_wtrd( cmnd,len,_rbuf,DBUF_LEN);
    if(stat==asynSuccess){
      p=strchr( _rbuf,'\n');
      if(p) *p=0;
      stat=setStringParam( _wfReply,_rbuf);
    }
  }
  else{
    stat=_write( cmnd,len);
  }
//printf( "%s::_command:cmnd=%s,stat=%d\n",dname,cmnd,stat);
  return(stat);
}
asynStatus drvTDS::_getInt( const char* cmnd,int pix){
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value and puts the obtained value in
 * parameter library at index pix.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int val;
  stat=_command( _makeQuery(cmnd));
  if(stat==asynSuccess){
    val=atoi(_rbuf);
    stat=setIntegerParam( pix,val);
  }
  return(stat);
}
asynStatus drvTDS::_getFloat( const char* cmnd,int pix){
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value and puts the obtained value in
 * parameter library at index pix.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; double val;
  char str[32]; int len=strlen(cmnd);
  if(len>30) return(asynError);
  strcpy( str,cmnd); strcat( str,"?");
  stat=_wtrd( str,strlen(str),_rbuf,DBUF_LEN);
  if(stat==asynSuccess){
    val=atof(_rbuf);
    stat=setDoubleParam( pix,val);
  }
  return(stat);
}
int drvTDS::_find( const char* item,const char** list,int n){
/*-----------------------------------------------------------------------------
 * Returns an index in list where item matches an element in the list.
 * If no match is found, returns -1.
 *---------------------------------------------------------------------------*/
  int i=0,m=strlen(item); char* p=(char*)strchr( item,'\n');
  if(p) *p=0;
  while(strncmp(item,list[i],m)){
    if((++i)>=n) return(-1);
  }
  return(i);
}
asynStatus drvTDS::_getBinary( const char* cmnd,int pix,
			const char** list,int n){
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value and puts the obtained value in
 * parameter library at index pix.  The reply is a string and a numeric
 * value is the index in a list of strings.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int val;
  char str[32]; int len=strlen(cmnd);
  if(len>30) return(asynError);
  strcpy( str,cmnd); strcat( str,"?");
  stat=_wtrd( str,strlen(str),_rbuf,DBUF_LEN);
  if(stat==asynSuccess){
    val=_find(_rbuf,list,n);
    if(val>=0) stat=setIntegerParam( pix,val);
  }
  return(stat);
}
asynStatus drvTDS::_getIntCh( const char* cmnd,int i,int pix){
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value for channel i and puts the obtained
 * value in parameter library at index pix.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int val;
  char str[32]; int len=strlen(cmnd);
  if((len>30)||(i<1)||(i>4)) return(asynError);
  sprintf( str,cmnd,i); strcat( str,"?");
  stat=_wtrd( str,strlen(str),_rbuf,DBUF_LEN);
  if(stat==asynSuccess){
    val=atoi(_rbuf);
    stat=setIntegerParam( i-1,pix,val);
  }
  return(stat);
}
asynStatus drvTDS::_getFloatCh( const char* cmnd,int i,int pix){
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value for channel i and puts the obtained
 * value in parameter library at index pix.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; double val;
  char str[32]; int len=strlen(cmnd);
  if((len>30)||(i<1)||(i>4)) return(asynError);
  sprintf( str,cmnd,i); strcat( str,"?");
  stat=_wtrd( str,strlen(str),_rbuf,DBUF_LEN);
  if(stat==asynSuccess){
    val=atof(_rbuf);
    stat=setDoubleParam( i-1,pix,val);
  }
  return(stat);
}
asynStatus drvTDS::_getBinaryCh( const char* cmnd,int i,int pix,
			const char** list,int n){
/*-----------------------------------------------------------------------------
 * Issues a query for an integer value for channel i and puts the obtained
 * value in parameter library at index pix.  The reply is a string and a numeric
 * value is the index in a list of strings.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int val=0;
  char str[32]; int len=strlen(cmnd);
  if((len>30)||(i<1)||(i>4)) return(asynError);
  sprintf( str,cmnd,i); strcat( str,"?");
  stat=_wtrd( str,strlen(str),_rbuf,DBUF_LEN);
  if(stat==asynSuccess){
    val=_find(_rbuf,list,n);
    if(val>=0) stat=setIntegerParam( i-1,pix,val);
  }
  return(stat);
}
void drvTDS::_setBinaryCh( int ix,int ch,const char* cmnd,
				const char** list,int n){
/*-----------------------------------------------------------------------------
 * ix is a bit position that was selected, this corresponds to an index
 * into list of size n from which we extract the value needed to construct
 * the command.
 *---------------------------------------------------------------------------*/
  if(ix<0||ix>=n) return;
  char str[32];
  sprintf( str,cmnd,ch+1); strcat( str," ");
  strcat( str,list[ix]);
  _command(str);
}
void drvTDS::_setBinary( int ix,const char* cmnd,const char** list,int n){
/*-----------------------------------------------------------------------------
 * ix is a bit position that was selected, this corresponds to an index
 * into list of size n from which we extract the value needed to construct
 * the command.
 *---------------------------------------------------------------------------*/
  if(ix<0||ix>=n) return;
  char str[32];
  sprintf( str,"%s %s",cmnd,list[ix]);
  _command(str);
}
void drvTDS::_setInt( const char* cmnd,int v,int pix){
/*-----------------------------------------------------------------------------
 * Constructs a string command to set an integer value and sends it.
 *---------------------------------------------------------------------------*/
  char str[32];
  sprintf( str,"%s %d",cmnd,v);
  _command(str);
  if(!pix) return;
  _getInt( cmnd,pix);
}
int drvTDS::_wfPreamble( char* p,int* ln,int* nb,
		double* ym,double* yz,double* yo){
/*-----------------------------------------------------------------------------
 * Unpacks the waveform preamble string.  Returns the length of the preamble
 * as a function value.
 *---------------------------------------------------------------------------*/
  int i=0,j,n,wd; int nbyt,nbit,len,ptof,chn; char chs[16];
  char enc[20],bfmt[20],bord[20],ids[80],ptfm[20],xunt[20],yunt[20];
  double xinc,xzr,ymult,yzr,yof;
  n=sscanf( p,"%d;%d;%19[^;];%19[^;];%19[^;];%d;%79[^;];%19[^;];"
		"%lg;%d;%lg;%19[^;];%lg;%lg;%lg;%19[^;];%n",
		&nbyt,&nbit,enc,bfmt,bord,&len,ids,ptfm,
		&xinc,&ptof,&xzr,xunt,&ymult,&yzr,&yof,yunt,&i);
  if(n!=16){
    if(n!=5){
      printf( "_wfPreamble: n=%d, failed to unpack preamble\n",n); return(-1);}
    return(0);
  }
  if((n=sscanf( &p[i],"#%1d",&wd))!=1){
    printf( "_wfPreamble: n=%d, failed to get width\n",n);
    return(-1);
  }
  j=i; i+=2+wd;
  strncpy( chs,&ids[1],3); chs[3]=0; sscanf( chs,"Ch%d",&chn);
//printf( "_wfPreamble: nbyt=%d,nbit=%d,enc=%s,bfmt=%s\n",nbyt,nbit,enc,bfmt);
//printf( "_wfPreamble: bord=%s,len=%d,ids=%s,ptfm=%s\n",bord,len,ids,ptfm);
//printf( "_wfPreamble: xinc=%g,ptof=%d,xzr=%g,xunt=%s\n",xinc,ptof,xzr,xunt);
//printf( "_wfPreamble: ymult=%g,yzr=%g,yof=%g,yunt=%s\n",ymult,yzr,yof,yunt);
//printf( "_wfPreamble: j=%d,i=%d,wd=%d\n",j,i,wd);
  *ln=len; *nb=nbyt; *ym=ymult; *yz=yzr; *yo=yof;
  return(i);
}
void drvTDS::_getWaveform( int ch){
/*-----------------------------------------------------------------------------
 * Requests waveform data for channel ch (0..3).  This gets waveform preamble
 * and waveform data.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int i,j,chon,len,n,nbyte,x0,np=0;
  double hs,pos,vdiv,ymult,yzr,yof; float* pwr=_wfraw;
  char str[32]; char* pb; short* pw; float ftmp; float* pwf=_wfbuf;

  getIntegerParam( ch,_biEnabl,&chon);
  if(chon){
    sprintf( str,TraceCmnd,ch+1); strcat( str,"?");
    stat=_wtrd( str,strlen(str),_rbuf,DBUF_LEN);
    if(stat!=asynSuccess) return;
    getDoubleParam( ch, _aiPosit,&pos);
    getDoubleParam( ch,_aiVDiv,&vdiv);
    getDoubleParam( 0,_aiTDiv,&hs);
    i=_wfPreamble( _rbuf,&len,&nbyte,&ymult,&yzr,&yof);
    if(i<=0) return;
    if(vdiv<0) vdiv=1.0;
    pb=(&_rbuf[i]);
    pw=(short*)pb;
    __getHSParams( hs,&x0,&np);
    n=WF_LEN;
    n=len<n?len:n;
    n=n<0?0:n;
    for( i=j=0; i<n; i++,pb++,pw++){
      if(nbyte==1) ftmp=(*pb); else ftmp=(*pw);
      if(i>=x0&&j<=np){
	*pwr=(ftmp-yof)*ymult+yzr;
	*pwf=((ftmp-yof)*ymult+yzr)/vdiv+pos;
	pwf++; pwr++; j++;
      }
    }
    if(_analize[ch]){
      _area[ch]=0.0;
      for( i=_mix1[ch]; i<=_mix2[ch]; i++){
	_area[ch]+=_wfraw[i];
      }
      if(_doPeds[ch]){ _doPeds[ch]=0; _pedestal[ch]=_area[ch];}
      else _area[ch]-=_pedestal[ch];
      setDoubleParam( ch,_aiArea,_area[ch]);
      setDoubleParam( ch,_aiPed,_pedestal[ch]);
      callParamCallbacks( ch);
    }
  }
  else for(i=0; i<WF_LEN; i++,pwf++) *pwf=1000.0;
  doCallbacksFloat32Array( _wfbuf,np,_wfTrace,ch);
}
void drvTDS::_getTrigLevel(){
/*-----------------------------------------------------------------------------
 * Setup slider for the trigger level value.
 *---------------------------------------------------------------------------*/
  int ch,sv,en; double levl,y,scl;
  getIntegerParam( 0,_mbbiTrigSo,&ch);
  if(ch<0||ch>=MAX_ADDR) return;
  getIntegerParam( ch,_biEnabl,&en);
  if(!en) return;
  getDoubleParam( 0,_aiTrigLev,&levl);
  getDoubleParam( ch,_aiPosit,&y);
  getDoubleParam( ch,_aiVDiv,&scl);
  if(-0.000001<scl&&scl<0.000001) scl=1.0;
  sv=(y+levl/scl)*100;
  setIntegerParam( 0,_liTLevl,sv);
}
void drvTDS::_setTrigLevel( int v){
/*-----------------------------------------------------------------------------
 * trigget level request from a slider.  v is slider value.
 *---------------------------------------------------------------------------*/
  int ch; double levl,y,scl; char cmnd[32];
  getIntegerParam( 0,_mbbiTrigSo,&ch);
  if(ch<0||ch>=MAX_ADDR) return;
  getDoubleParam( ch,_aiPosit,&y);
  getDoubleParam( ch,_aiVDiv,&scl);
  levl=(v/100.0-y)*scl;
  sprintf( cmnd,"%s %f",TrigLevCmnd,levl);
  _command( cmnd);
  setDoubleParam( 0,_aiTrigLev,levl);
}
void drvTDS::_setPosSlider( double v){
/*-----------------------------------------------------------------------------
 * Set slider position to v division.
 *---------------------------------------------------------------------------*/
  setIntegerParam( 0,_liChPos,v*100);
  callParamCallbacks(0);
}
void drvTDS::_selectChan( int ch){
/*-----------------------------------------------------------------------------
 * Select channel ch, used to control channel trace position with a slider.
 *---------------------------------------------------------------------------*/
  int i,v; double y;
  if(ch<0||ch>=MAX_ADDR) return;
  getIntegerParam( ch,_biEnabl,&v);
  if(!v){
    setIntegerParam( ch,_biChSel,1);
    setIntegerParam( ch,_biChSel,0);
    callParamCallbacks(ch);
    return;
  }
  for( i=0; i<MAX_ADDR; i++){
    if(i==ch) v=1; else v=0;
    setIntegerParam( i,_biChSel,v);
    callParamCallbacks(i);
  }
  getDoubleParam( ch,_aiPosit,&y);
  _setPosSlider(y);
  _chSel=ch;
}
void drvTDS::_selectChannel(){
/*-----------------------------------------------------------------------------
 * Select the first on channel ch,
 * used to control channel trace position with a slider.
 *---------------------------------------------------------------------------*/
  int i,v; static int firsttime=1;
  getIntegerParam( _chSel,_biEnabl,&v);
  if(v&&!firsttime) return;
  firsttime=0;
  for( i=0; i<MAX_ADDR; i++){
    _selectChan(i);
  }
}
asynStatus drvTDS::_getCmnds( int ix,int addr){
/*-----------------------------------------------------------------------------
 * This routine is called from the pollerThread routine, when it receives
 * a message on the message queue.
 * ix is the index into parameter library (or the reason)
 * addr is channel or address
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int ch=addr+1,jx=ix-_firstix; double dv;

  _opc();
  switch( jx){
    case ixWfIdn:	_getIdn(); break;
    case ixSiIpName:	_getIpAddr(); break;
    case ixAiTDly:	_getFloat( TDlyCmnd,ix);
			getDoubleParam( _aiTDly,&dv);
			_timeDelayStr(dv);
			break;
    case ixAiTDiv:	_getFloat( TDivCmnd,ix); break;
    case ixAiTRef:	_getFloat( TRefCmnd,ix); break;
    case ixAiTrigLev:	_getFloat( TrigLevCmnd,ix);
			_getTrigLevel();
			break;
    case ixAiHldOff:	_getFloat( HldOffCmnd,ix); break;
    case ixBiTMode:	_getBinary( TModeCmnd,ix,tmodeList,SIZE(tmodeList));
			break;
    case ixMbbiTrigSo:	_getBinary( TrigSoCmnd,ix,trigSoList,SIZE(trigSoList));
			break;
    case ixBiTrigSlo:	_getBinary( TrigSloCmnd,ix,
				trigSloList,SIZE(trigSloList)); break;
    case ixMbbiTrigSta:	_getBinary( TrigStaCmnd,ix,
				trigStaList,SIZE(trigStaList)); break;
    case ixMbbiDIntens:	_getBinary( DIntensCmnd,ix,
				dIntensList,SIZE(dIntensList)); break;
    case ixMbbiDEncod:	_getBinary( DEncodCmnd,ix,
				dEncodList,SIZE(dEncodList)); break;
    case ixBiEnabl:	_getIntCh( EnablCmnd,ch,ix);
			_selectChannel();
			break;
    case ixAiPosit:	_getFloatCh( PositCmnd,ch,ix);
			getDoubleParam( addr,ix,&dv);
			if(addr==_chSel) _setPosSlider(dv);
			break;
    case ixAiOffst:	_getFloatCh( OffstCmnd,ch,ix); break;
    case ixBiImped:	_getBinaryCh( ImpedCmnd,ch,ix,
				impedList,SIZE(impedList)); break;
    case ixMbbiCoupl:	_getBinaryCh( CouplCmnd,ch,ix,
				couplList,SIZE(couplList)); break;
    case ixAiVDiv:	_getFloatCh( ScaleCmnd,ch,ix); break;
    case ixMbbiScl:	_getBinaryCh( ScaleCmnd,ch,ix,
				scaleList,SIZE(scaleList)); break;
    case ixSiDSource:	_getString( DSourceCmnd,ix); break;
    case ixSiHeader:	_getString( HeaderCmnd,ix); break;
    case ixSiDEncod:	_getString( DEncodCmnd,ix); break;
    case ixSiOPC:	_getString( OPCCmnd,ix); break;
    case ixLiDBright:	_getInt( DBrightCmnd,ix); break;
    case ixLiIntensWf:	_getInt( IntensWfCmnd,ix); break;
    case ixLiRecLen:	_getInt( RecLenCmnd,ix); break;
    case ixLiDWidth:	_getInt( DWidthCmnd,ix); break;
    case ixLiDStart:	_getInt( DStartCmnd,ix); break;
    case ixLiDStop:	_getInt( DStopCmnd,ix); break;
    case ixLiDese:	_getInt( DeseCmnd,ix); break;
    case ixLiEsr:	_getInt( EsrCmnd,ix); break;
    case ixLiEse:	_getInt( EseCmnd,ix); break;
    case ixLiEvQ:	_getInt( EvQCmnd,ix); break;
    case ixLiStb:	_getInt( StbCmnd,ix); break;
    case ixLiEvQty:	_getInt( EvQtyCmnd,ix); break;
    case ixBiAcqStat:	_getInt( AcqStatCmnd,ix); break;
    case ixBiFLock:	_getBinary( FLockCmnd,ix,fLockList,SIZE(fLockList));
			break;
    case ixBiBusy:	_getInt( BusyCmnd,ix); break;
    case ixBiDAutoC:	_getInt( DAutoCCmnd,ix); break;
    case ixBiDPictSt:	_getInt( DPictStCmnd,ix); break;

    default:            stat=asynError; break;
  }
  callParamCallbacks( addr);
  return(stat);
}
asynStatus drvTDS::writeOctet( asynUser* paUser,const char* v,
			size_t nc,size_t* nActual){
/*-----------------------------------------------------------------------------
 * This method overrides the virtual method in asynPortDriver.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int ix,addr;
  stat=getAddress(paUser,&addr);
  if(stat!=asynSuccess) return(stat);
  ix=paUser->reason-_firstix;
  switch( ix){
    case ixSoCmnd:	stat=_command(v);
			*nActual=nc;
    default:		break;
  }
  stat=asynPortDriver::writeOctet( paUser,v,nc,nActual);
  callParamCallbacks();
  return(stat);
}
asynStatus drvTDS::_putIntCmnds( int ix,int addr,int v){
/*-----------------------------------------------------------------------------
 * This routine is called from the pollerThread routine, when it receives
 * a message on the message queue.
 * ix is the index into parameter library (or the reason)
 * addr is channel or address
 * v is a possible integer set value.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; char cmnd[32];
  int jx=ix-_firstix; double y;

  switch( jx){
    case ixBoUpdt:	_update(); break;
    case ixBoEnabl:	sprintf( cmnd,EnablCmnd,addr+1);
			if(v) strcat(cmnd," ON");
			else strcat( cmnd," OFF");
			_command( cmnd);
			setIntegerParam( addr,_biEnabl,v);
			_selectChannel();
			break;
    case ixBoImped:	setIntegerParam( addr,_biImped,v);
			_setBinaryCh( v,addr,ImpedCmnd,
				impedList,SIZE(impedList));
			break;
    case ixMbboCoupl:	setIntegerParam( addr,_mbbiCoupl,v);
			_setBinaryCh( v,addr,CouplCmnd,
				couplList,SIZE(couplList));
			break;
    case ixMbboScl:	setIntegerParam( addr,_mbbiScl,v);
			_setBinaryCh( v,addr,ScaleCmnd,
				scaleList,SIZE(scaleList));
			_getFloatCh( ScaleCmnd,addr+1,_aiVDiv);
			_getTrigLevel();
			break;
    case ixBoTDlySt:	sprintf( cmnd,"%s %d",TDlyStCmnd,v);
			_command( cmnd);
			setIntegerParam( addr,_biTDlySt,v);
			break;
    case ixMbboTDivV:	stat=getIntegerParam( _mbboTDivU,&jx);
			_setTimePerDiv( v,jx);
			break;
    case ixMbboTDivU:	getIntegerParam( _mbboTDivV,&jx);
			_setTimePerDiv( jx,v);
			break;
    case ixBoGetWf:	_getWaveform( addr);
			break;
    case ixBoTMode:	_setBinary(v,TModeCmnd,tmodeList,SIZE(tmodeList));
			setIntegerParam( _biTMode,v);
			break;
    case ixMbboTrigSo:	_setBinary(v,TrigSoCmnd,trigSoList,SIZE(trigSoList));
			setIntegerParam( _mbbiTrigSo,v);
			_getTrigLevel();
			break;
    case ixBoTrigSlo:	_setBinary(v,TrigSloCmnd,trigSloList,SIZE(trigSloList));
			setIntegerParam( _biTrigSlo,v);
			break;
    case ixBoRun:	_command( RunCmnd);
			_getInt( AcqStatCmnd,_biAcqStat);
			break;
    case ixBoStop:	_command( StopCmnd);
			_getInt( AcqStatCmnd,_biAcqStat);
			break;
    case ixBoReset:	_command( ResetCmnd); break;
    case ixBoFLock:
    case ixBoDAutoC:
    case ixLoRecall:	sprintf( cmnd,"*RCL %d",v);
			_command( cmnd); break;
    case ixLoStore:	sprintf( cmnd,"*SAV %d",v);
			_command( cmnd); break;
    case ixBoInit:	_command( InitCmnd); break;
    case ixLoDBright:	_setInt( DBrightCmnd,v,_liDBright); break;
    case ixBoDPictSt:	_setInt( DPictStCmnd,v,_biDPictSt); break;
    case ixMbboDEncod:	break;
    case ixLoRecLen:	_setInt( RecLenCmnd,v,_liRecLen); break;
    case ixLoDWidth:	_setInt( DWidthCmnd,v,_liDWidth); break;
    case ixLoDStart:	_setInt( DStartCmnd,v,_liDStart); break;
    case ixLoDStop:	_setInt( DStopCmnd,v,_liDStop); break;
    case ixLoDese:	_setInt( DeseCmnd,v,_liDese); break;
    case ixLoEse:	_setInt( EseCmnd,v,_liEse); break;
    case ixBoCls:	_command( ClsCmnd); break;
    case ixBoAPed:	_doPeds[addr]=1; break;
    case ixBoChSel:	_selectChan(addr); break;
    case ixLoChPos:	y=v/100.0;
			sprintf( cmnd,PositCmndW,_chSel+1,y);
			_command(cmnd);
			setDoubleParam( _chSel,_aiPosit,y);
			_getTrigLevel();
			break;
    case ixLoTLevl:	_setTrigLevel( v); break;
    default:		stat=asynError; break;
  }
  callParamCallbacks(addr);
  return(stat);
}
asynStatus drvTDS::writeInt32( asynUser* pau,epicsInt32 v){
/*-----------------------------------------------------------------------------
 * This method overrides the virtual method in asynPortDriver.  Here we service
 * all write requests comming from EPICS records.
 * Parameters:
 *  pau         (in) structure containing addr and reason.
 *  v           (in) this is the command index, which together with
 *              pau->reason define the command to be sent.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int on,ix,jx,addr;

  stat=getAddress(pau,&addr); if(stat!=asynSuccess) return(stat);
  ix=pau->reason;
  jx=ix-_firstix;
  switch( jx){
    case ixBoAnal:	getIntegerParam( addr,_biEnabl,&on);
			if(on) _analize[addr]=v;
			else _analize[addr]=0;
			setIntegerParam( addr,_biAnal,1-_analize[addr]);
			setIntegerParam( addr,_biAnal,_analize[addr]);
			break;
    case ixMbboMChan:	_markchan=MIN(NCHAN-1,MAX(v,0));
			setIntegerParam( _liMark1,_mix1[_markchan]+1);
			setIntegerParam( _liMark1,_mix1[_markchan]);
			setIntegerParam( _liMark2,_mix2[_markchan]+1);
			setIntegerParam( _liMark2,_mix2[_markchan]);
			break;
    case ixLoMark1:	_mix1[_markchan]=v; break;
    case ixLoMark2:	_mix2[_markchan]=v; break;
    case ixMbboTracMod:	_tracemode=v;
			setIntegerParam( 0,_mbbiTracMod,v); break;
    case ixBoGetWfA:	_getTraces(); break;
//    case ixBoUpdt:	_doUpdate(); break;
    case ixLoPCtrl:	_pollCtrl=v; break;
    case ixBoRdTraces:	_rdtraces=v; break;
    default:		_putInMessgQ( enPutInt,ix,addr,v); break;
  }
  callParamCallbacks(addr);
  return(stat);
}
asynStatus drvTDS::_putFltCmnds( int ix,int addr,float v){
/*-----------------------------------------------------------------------------
 * This routine is called from the pollerThread routine, when it receives
 * a message on the message queue.
 * ix is the index into parameter library (or the reason)
 * addr is channel or address
 * v is a possible integer set value.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; char cmnd[32]; int jx=ix-_firstix;
  switch( jx){
    case ixAoCTDiv:	_setTimePerDiv( v); break;
    case ixAoPosit:	sprintf( cmnd,PositCmndW,addr+1,v);
			_command(cmnd);
			setDoubleParam( addr,_aiPosit,v);
			if(addr==_chSel) _setPosSlider( v);
			_getTrigLevel();
			break;
    case ixAoOffst:	setDoubleParam( addr,_aiOffst,v);
			break;
    case ixAoTDly:	setDoubleParam( addr,_aiTDly,v);
			_setTimeDelayStr();
			break;
    case ixAoTRef:	sprintf( cmnd,"%s %d",TRefCmnd,(int)v);
			_command(cmnd);
			break;
    case ixAoTrigLev:	sprintf( cmnd,"%s %f",TrigLevCmnd,v);
			_command( cmnd);
			setDoubleParam( addr,_aiTrigLev,v);
			_getTrigLevel();
			break;
    case ixAoHldOff:
    default:		stat=asynError; break;
  }
  callParamCallbacks(addr);
  return(stat);
}
asynStatus drvTDS::writeFloat64( asynUser* pau,epicsFloat64 v){
/*-----------------------------------------------------------------------------
 * This method overrides the virtual method in asynPortDriver.  Here we service
 * all write requests comming from EPICS records.
 * Parameters:
 *  pau         (in) structure containing addr and reason.
 *  v           (in) this is the command index, which together with
 *              pau->reason define the command to be sent.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int ix,addr; float fv=v;

  stat=getAddress(pau,&addr); if(stat!=asynSuccess) return(stat);
  ix=pau->reason-_firstix;
  switch( ix){
    case ixAoPTMO:	_pollT=v;
			break;
    default:		_putInMessgQ( enPutFlt,ix,addr,0,fv); break;
  }
  return(stat);
}
void drvTDS::_getTraces(){
/*-----------------------------------------------------------------------------
 * Initiate getting waveform trace data for all channels.  Traces will be
 * read in asynchronously or synchronously depending on the value of the
 * _tracemode variable.  Synchronous mode is when all four traces are obtained
 * for the same event.
 *---------------------------------------------------------------------------*/
  if(_tracemode==enTMNone) return;
  if(_tracemode==enTMSync) _command( StopCmnd);
  _getWaveform(0);
  _getWaveform(1);
  _getWaveform(2);
  _getWaveform(3);
 _command( RunCmnd);
  setIntegerParam( _biCtGets,0);
  setIntegerParam( _biCtGets,1);
  callParamCallbacks(0);
}
void drvTDS::_update(){
/*----------------------------------------------------------------------------
 * This is called at startup and periodically to stay in sync with user
 * changing settings on the instrument.
 *--------------------------------------------------------------------------*/
  int i,ch,v; static int firsttime=1; double dv;
  _getBinary( TrigSoCmnd,_mbbiTrigSo,trigSoList,SIZE(trigSoList));
  _getBinary( TrigSloCmnd,_biTrigSlo,trigSloList,SIZE(trigSloList));
  _getBinary( TrigStaCmnd,_mbbiTrigSta,trigStaList,SIZE(trigStaList));
  for( i=0; i<MAX_ADDR; i++){
    ch=i+1;
    _getIntCh( EnablCmnd,ch,_biEnabl); _selectChannel();
    getIntegerParam( i,_biEnabl,&v);
    int test=v||firsttime;
    if(!test) continue;
    _getFloatCh( PositCmnd,ch,_aiPosit);
    getDoubleParam( i,_aiPosit,&dv);
    if(i==_chSel) _setPosSlider(dv);
    _getBinaryCh( ImpedCmnd,ch,_biImped,impedList,SIZE(impedList));
    _getBinaryCh( CouplCmnd,ch,_mbbiCoupl,couplList,SIZE(couplList));
    _getFloatCh( ScaleCmnd,ch,_aiVDiv);
    _getBinaryCh( ScaleCmnd,ch,_mbbiScl,scaleList,SIZE(scaleList));
    callParamCallbacks(i);
  }
  _getFloat( TDlyCmnd,_aiTDly); getDoubleParam( _aiTDly,&dv);
  _timeDelayStr(dv);
  _getInt( TDlyStCmnd,_biTDlySt);
  _getFloat( TDivCmnd,_aiTDiv);
  _getFloat( TRefCmnd,_aiTRef);
  _getFloat( TrigLevCmnd,_aiTrigLev); _getTrigLevel();
  _getFloat( HldOffCmnd,_aiHldOff);
  _getBinary( TModeCmnd,_biTMode,tmodeList,SIZE(tmodeList));
  firsttime=0;
}
void drvTDS::_getUpdate(){
/*----------------------------------------------------------------------------
 * This is called at startup and periodically to stay in sync with user
 * changing settings on the instrument.
 *--------------------------------------------------------------------------*/
  int i,v; static int firsttime=1;
  _putInMessgQ( enQuery,_mbbiTrigSo,0,0);
  _putInMessgQ( enQuery,_biTrigSlo,0,0);
  _putInMessgQ( enQuery,_mbbiTrigSta,0,0);
//  _putInMessgQ( enQuery,_mbbiDIntens,0,0);
//  _putInMessgQ( enQuery,_mbbiDEncod,0,0);
  for( i=0; i<MAX_ADDR; i++){
    _putInMessgQ( enQuery,_biEnabl,i,0);
    getIntegerParam( i,_biEnabl,&v);
    if(!v&&!firsttime) continue;
    _putInMessgQ( enQuery,_aiPosit,i,0);
//    _putInMessgQ( enQuery,_aiOffst,i,0);
    _putInMessgQ( enQuery,_biImped,i,0);
    _putInMessgQ( enQuery,_mbbiCoupl,i,0);
    _putInMessgQ( enQuery,_aiVDiv,i,0);
    _putInMessgQ( enQuery,_mbbiScl,i,0);
  }
  _putInMessgQ( enQuery,_aiTDly,0,0);
  _putInMessgQ( enQuery,_aiTDiv,0,0);
  _putInMessgQ( enQuery,_aiTRef,0,0);
  _putInMessgQ( enQuery,_aiTrigLev,0,0);
  _putInMessgQ( enQuery,_aiHldOff,0,0);
  _putInMessgQ( enQuery,_biTMode,0,0);
  firsttime=0;
}
void drvTDS::_doUpdate(){
/*----------------------------------------------------------------------------
 *--------------------------------------------------------------------------*/
  int n=_pmq->pending();
  if(n) return;
  _getUpdate();
}
void drvTDS::afterInit(){
/*----------------------------------------------------------------------------
 *--------------------------------------------------------------------------*/
  _command( InitCmnd);
  _putInMessgQ( enQuery,_wfIdn,0,0);
  _putInMessgQ( enQuery,_siIpName,0,0);
  _getUpdate();
  _putInMessgQ( enQuery,_siDSource,0,0);
  _putInMessgQ( enQuery,_siHeader,0,0);
  _putInMessgQ( enQuery,_siDEncod,0,0);
  _putInMessgQ( enQuery,_mbbiDEncod,0,0);
  _putInMessgQ( enQuery,_siOPC,0,0);
  _putInMessgQ( enQuery,_liDBright,0,0);
  _putInMessgQ( enQuery,_liIntensWf,0,0);
  _putInMessgQ( enQuery,_liRecLen,0,0);
  _putInMessgQ( enQuery,_liDWidth,0,0);
  _putInMessgQ( enQuery,_liDStart,0,0);
  _putInMessgQ( enQuery,_liDStop,0,0);
  _putInMessgQ( enQuery,_liDese,0,0);
  _putInMessgQ( enQuery,_liEsr,0,0);
  _putInMessgQ( enQuery,_liEse,0,0);
  _putInMessgQ( enQuery,_liEvQ,0,0);
  _putInMessgQ( enQuery,_liStb,0,0);
  _putInMessgQ( enQuery,_liEvQty,0,0);
  _putInMessgQ( enQuery,_biAcqStat,0,0);
  _putInMessgQ( enQuery,_biFLock,0,0);
  _putInMessgQ( enQuery,_biBusy,0,0);
  _putInMessgQ( enQuery,_biDAutoC,0,0);
  _putInMessgQ( enQuery,_biDPictSt,0,0);
  _putInMessgQ( enQuery,_aiTrigLev,0,0);

//  _setTimeDelayStr();
//  _selectChannel();
}
drvTDS::drvTDS(const char* port,const char* udp):
  asynPortDriver( port,MAX_ADDR,N_PARAMS,
	asynInt32Mask|asynFloat64Mask|asynFloat32ArrayMask|
		asynOctetMask|asynDrvUserMask,
	asynInt32Mask|asynFloat64Mask|asynFloat32ArrayMask|asynOctetMask,
	ASYN_CANBLOCK|ASYN_MULTIDEVICE,1,0,0){
/*------------------------------------------------------------------------------
 * Constructor for the drvTDS class. Calls constructor for the asynPortDriver
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
  int i,status=asynSuccess,nbts,st=0;

  _this=this;
  nbts=strlen(port)+strlen(udp)+2;
  _port=(char*)callocMustSucceed( nbts,sizeof(char),dname);
  _udpp=(char*)(_port+strlen(port)+1);
  strcpy((char*)_port,port);
  strcpy((char*)_udpp,udp);
  _ncmnds=0; _pollT=0.1; _markchan=0; _chSel=0; _tracemode=0;
  for( i=0; i<NCHAN; i++){
    _analize[i]=_mix1[i]=_mix2[i]=0; _area[i]=_pedestal[i]=0.0;
  }
  _rdtraces=1;
  _pollCtrl=POLLWF|POLLCTRLS;
  _putil=new Utils(1);
  status=pasynOctetSyncIO->connect( udp,0,&_aPvt,0);
  if(status!=asynSuccess)
    printf( "%s::%s:connect: failed to connect to port %s\n",
		dname,dname,udp);
  else{
    printf( "%s::%s:connect: connected to port %s\n",dname,dname,udp);
    st=1;
  }
  createParam( siNameStr,	asynParamOctet,		&_siName);
  createParam( biEnablStr,	asynParamInt32,		&_biEnabl);
  createParam( boEnablStr,	asynParamInt32,		&_boEnabl);
  createParam( aiPositStr,	asynParamFloat64,	&_aiPosit);
  createParam( aoPositStr,	asynParamFloat64,	&_aoPosit);

  createParam( aiOffstStr,	asynParamFloat64,	&_aiOffst);
  createParam( aoOffstStr,	asynParamFloat64,	&_aoOffst);
  createParam( biImpedStr,	asynParamInt32,		&_biImped);
  createParam( boImpedStr,	asynParamInt32,		&_boImped);
  createParam( mbbiCouplStr,	asynParamInt32,		&_mbbiCoupl);

  createParam( mbboCouplStr,	asynParamInt32,		&_mbboCoupl);
  createParam( aiVDivStr,	asynParamFloat64,	&_aiVDiv);
  createParam( mbboSclStr,	asynParamInt32,		&_mbboScl);
  createParam( mbbiSclStr,	asynParamInt32,		&_mbbiScl);
  createParam( wfTraceStr,	asynParamFloat32Array,	&_wfTrace);

  createParam( biCtGetsStr,	asynParamInt32,		&_biCtGets);
  createParam( boGetWfStr,	asynParamInt32,		&_boGetWf);
  createParam( boGetWfAStr,	asynParamInt32,		&_boGetWfA);
  createParam( boUpdtStr,	asynParamInt32,		&_boUpdt);
  createParam( aiTDlyStr,	asynParamFloat64,	&_aiTDly);

  createParam( aoTDlyStr,	asynParamFloat64,	&_aoTDly);
  createParam( siTDlyStr,	asynParamOctet,		&_siTDly);
  createParam( biTDlyStStr,	asynParamInt32,		&_biTDlySt);
  createParam( boTDlyStStr,	asynParamInt32,		&_boTDlySt);
  createParam( aiTDivStr,	asynParamFloat64,	&_aiTDiv);

  createParam( siTDivStr,	asynParamOctet,		&_siTDiv);
  createParam( aoCTDivStr,	asynParamFloat64,	&_aoCTDiv);
  createParam( mbbiTDivVStr,	asynParamInt32,		&_mbbiTDivV);
  createParam( mbboTDivVStr,	asynParamInt32,		&_mbboTDivV);
  createParam( mbbiTDivUStr,	asynParamInt32,		&_mbbiTDivU);

  createParam( mbboTDivUStr,	asynParamInt32,		&_mbboTDivU);
  createParam( aiTRefStr,	asynParamFloat64,	&_aiTRef);
  createParam( aoTRefStr,	asynParamFloat64,	&_aoTRef);
  createParam( aiTrigLevStr,	asynParamFloat64,	&_aiTrigLev);
  createParam( aoTrigLevStr,	asynParamFloat64,	&_aoTrigLev);

  createParam( aiHldOffStr,	asynParamFloat64,	&_aiHldOff);
  createParam( aoHldOffStr,	asynParamFloat64,	&_aoHldOff);
  createParam( biTModeStr,	asynParamInt32,		&_biTMode);
  createParam( boTModeStr,	asynParamInt32,		&_boTMode);
  createParam( mbbiTrigSoStr,	asynParamInt32,		&_mbbiTrigSo);

  createParam( mbboTrigSoStr,	asynParamInt32,		&_mbboTrigSo);
  createParam( biTrigSloStr,	asynParamInt32,		&_biTrigSlo);
  createParam( boTrigSloStr,	asynParamInt32,		&_boTrigSlo);
  createParam( biAcqStatStr,	asynParamInt32,		&_biAcqStat);
  createParam( mbbiTrigStaStr,	asynParamInt32,		&_mbbiTrigSta);

  createParam( boRunStr,	asynParamInt32,		&_boRun);
  createParam( boStopStr,	asynParamInt32,		&_boStop);
  createParam( biFLockStr,	asynParamInt32,		&_biFLock);
  createParam( boFLockStr,	asynParamInt32,		&_boFLock);
  createParam( biBusyStr,	asynParamInt32,		&_biBusy);

  createParam( boResetStr,	asynParamInt32,		&_boReset);
  createParam( wfSetupStr,	asynParamOctet,		&_wfSetup);
  createParam( wfWSetupStr,	asynParamOctet,		&_wfWSetup);
  createParam( loRecallStr,	asynParamInt32,		&_loRecall);
  createParam( loStoreStr,	asynParamInt32,		&_loStore);

  createParam( siDSourceStr,	asynParamOctet,		&_siDSource);
  createParam( biDAutoCStr,	asynParamInt32,		&_biDAutoC);
  createParam( boDAutoCStr,	asynParamInt32,		&_boDAutoC);
  createParam( liDBrightStr,	asynParamInt32,		&_liDBright);
  createParam( loDBrightStr,	asynParamInt32,		&_loDBright);

  createParam( biDPictStStr,	asynParamInt32,		&_biDPictSt);
  createParam( boDPictStStr,	asynParamInt32,		&_boDPictSt);
  createParam( mbbiDIntensStr,	asynParamInt32,		&_mbbiDIntens);
  createParam( mbboDIntensStr,	asynParamInt32,		&_mbboDIntens);
  createParam( liIntensWfStr,	asynParamInt32,		&_liIntensWf);

  createParam( loIntensWfStr,	asynParamInt32,		&_loIntensWf);
  createParam( boInitStr,	asynParamInt32,		&_boInit);
  createParam( mbbiDEncodStr,	asynParamInt32,		&_mbbiDEncod);
  createParam( mbboDEncodStr,	asynParamInt32,		&_mbboDEncod);
  createParam( siDEncodStr,	asynParamOctet,		&_siDEncod);

  createParam( liRecLenStr,	asynParamInt32,		&_liRecLen);
  createParam( loRecLenStr,	asynParamInt32,		&_loRecLen);
  createParam( siHeaderStr,	asynParamOctet,		&_siHeader);
  createParam( liDWidthStr,	asynParamInt32,		&_liDWidth);
  createParam( loDWidthStr,	asynParamInt32,		&_loDWidth);

  createParam( liDStartStr,	asynParamInt32,		&_liDStart);
  createParam( loDStartStr,	asynParamInt32,		&_loDStart);
  createParam( liDStopStr,	asynParamInt32,		&_liDStop);
  createParam( loDStopStr,	asynParamInt32,		&_loDStop);
  createParam( wfIdnStr,	asynParamOctet,		&_wfIdn);

  createParam( siIpNameStr,	asynParamOctet,		&_siIpName);
  createParam( soCmndStr,	asynParamOctet,		&_soCmnd);
  createParam( wfReplyStr,	asynParamOctet,		&_wfReply);
  createParam( aoPTMOStr,	asynParamFloat64,	&_aoPTMO);
  createParam( loPCtrlStr,	asynParamInt32,		&_loPCtrl);

  createParam( boAnalStr,	asynParamInt32,		&_boAnal);
  createParam( biAnalStr,	asynParamInt32,		&_biAnal);
  createParam( boAPedStr,	asynParamInt32,		&_boAPed);
  createParam( aiAreaStr,	asynParamFloat64,	&_aiArea);
  createParam( aiPedStr,	asynParamFloat64,	&_aiPed);

  createParam( mbboMChanStr,	asynParamInt32,		&_mbboMChan);
  createParam( loMark1Str,	asynParamInt32,		&_loMark1);
  createParam( loMark2Str,	asynParamInt32,		&_loMark2);
  createParam( liMark1Str,	asynParamInt32,		&_liMark1);
  createParam( liMark2Str,	asynParamInt32,		&_liMark2);

  createParam( liDeseStr,	asynParamInt32,		&_liDese);
  createParam( loDeseStr,	asynParamInt32,		&_loDese);
  createParam( liEsrStr,	asynParamInt32,		&_liEsr);
  createParam( liEseStr,	asynParamInt32,		&_liEse);
  createParam( loEseStr,	asynParamInt32,		&_loEse);

  createParam( liEvQStr,	asynParamInt32,		&_liEvQ);
  createParam( boClsStr,	asynParamInt32,		&_boCls);
  createParam( siOPCStr,	asynParamOctet,		&_siOPC);
  createParam( liStbStr,	asynParamInt32,		&_liStb);
  createParam( wfEventStr,	asynParamOctet,		&_wfEvent);

  createParam( wfMessgStr,	asynParamOctet,		&_wfMessg);
  createParam( biChSelStr,	asynParamInt32,		&_biChSel);
  createParam( boChSelStr,	asynParamInt32,		&_boChSel);
  createParam( liChPosStr,	asynParamInt32,		&_liChPos);
  createParam( loChPosStr,	asynParamInt32,		&_loChPos);

  createParam( liTLevlStr,	asynParamInt32,		&_liTLevl);
  createParam( loTLevlStr,	asynParamInt32,		&_loTLevl);
  createParam( liMQSuccsStr,	asynParamInt32,		&_liMsgQS);
  createParam( liMQFailStr,	asynParamInt32,		&_liMsgQF);
  createParam( mbbiTracModStr,	asynParamInt32,		&_mbbiTracMod);

  createParam( mbboTracModStr,	asynParamInt32,		&_mbboTracMod);
  createParam( liXNptsStr,	asynParamInt32,		&_liXNpts);
  createParam( biStateStr,	asynParamInt32,		&_biState);
  createParam( boRdTracesStr,	asynParamInt32,		&_boRdTraces);
  createParam( liEvQtyStr,	asynParamInt32,		&_liEvQty);
  _firstix=_siName;

  setStringParam( _siName,dname);
  setIntegerParam( _biState,st);
  setIntegerParam( _boRdTraces,_rdtraces);
  if(st) _message( "Constructor drvTDS success");
  else _message( "Constructor drvTDS failed");
  callParamCallbacks( 0);
  _pmq=new epicsMessageQueue( NMSGQ,MSGQNB);
  errlogPrintf( "%s::%s: messageQueue created, id=0x%p\n",dname,dname,_pmq);
  epicsThreadCreate(dname,epicsThreadPriorityHigh,
                epicsThreadGetStackSize(epicsThreadStackMedium),
                (EPICSTHREADFUNC)pollerThreadC,this);
}
// Configuration routines.  Called directly, or from the iocsh function below
extern "C" {

int drvTDSConfigure( const char* port,const char* udp){
/*-----------------------------------------------------------------------------
 * EPICS iocsh callable function to call constructor for the drvTDS class.
 *  port The name of the asyn port driver to be created.
 *  udp is the IO port.
 *---------------------------------------------------------------------------*/
  new drvTDS(port,udp);
  return(asynSuccess);
}

/* EPICS iocsh shell commands */

static const iocshArg initArg0 = { "port",iocshArgString};
static const iocshArg initArg1 = { "udp",iocshArgString};
static const iocshArg * const initArgs[] = {&initArg0,&initArg1};
static const iocshFuncDef initFuncDef = {"drvTDSConfigure",2,initArgs};
static void initCallFunc(const iocshArgBuf *args){
  drvTDSConfigure(args[0].sval, args[1].sval);
}
void drvTDSRegister(void){
  iocshRegister(&initFuncDef,initCallFunc);
  initHookRegister(&inithooks);
}
epicsExportRegistrar(drvTDSRegister);
}
