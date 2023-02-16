/* drvDS1x.cpp
 * This driver class is derived from drvScope base class.  Here we implement
 * details needed to control and monitor the Rigol DS4000 and DS6000 series
 * oscilloscopes.
 * Started on 05/28/2015, zms
 *---------------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <cantProceed.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsExport.h>
#include <errlog.h>
#include <initHooks.h>
#include <iocsh.h>

#include "drvDS1x.h"

const char* NullCmnd	 ="";
// channel
const char* ChCplCmnd    =":CHAN%d:COUP";
const char* ChOnCmnd     =":CHAN%d:DISP";
const char* ChPosCmnd    =":CHAN%d:OFFS";
const char* ChSclCmnd    =":CHAN%d:SCAL";
const char* ChUnitCmnd   =":CHAN%d:UNIT";
// Waveform
const char* WfDatCmnd    =":WAV:SOUR CHAN%d; :WAV:DATA?";
const char* WfPreCmnd    =":WAV:SOUR CHAN%d; :WAV:PRE?";
const char* WfFormCmnd   =":WAV:FORM";
const char* WfNptCmnd    =":WAV:POIN";
// Time Base
const char* TmModeCmnd   =":TIM:MODE";
const char* TmSclCmnd    =":TIM:SCAL";
const char* TmOfstCmnd   =":TIM:OFFS";
const char* TmDlyOfsCmnd =":TIM:DEL:OFFS";
const char* TmDlySclCmnd =":TIM:DEL:SCAL";
const char* TmFormCmnd   =":TIM:FORM";
// Trigger
const char* TrigModeCmnd =":TRIG:MODE";
const char* TrigSouCmnd  =":TRIG:EDGE:SOUR";
const char* TrigLevCmnd  =":TRIG:EDGE:LEV";
const char* TrigSweCmnd  =":TRIG:EDGE:SWE";
const char* TrigCplCmnd	 =":TRIG:COUP";
const char* TrigHoldCmnd =":TRIG:HOLD";
const char* TrigStCmnd   =":TRIG:STAT?";
const char* TrigSloCmnd  =":TRIG:EDGE:SLOP";
// General
const char* IdnCmnd      ="*IDN?";
const char* ResetCmnd    ="*RST";
const char* LearnCmnd    ="*LRN?";
const char* OpcCmnd      ="*OPC?";
// System
const char* RunCmnd      =":RUN";
const char* StopCmnd     =":STOP";
const char* AutoCmnd     =":AUTO";
const char* ErrMsgCmnd   =":SYST:ERR?";
const char* SetupCmnd    =":SYST:SET";
// Acquire
const char* AcqTypeCmnd  =":ACQ:TYPE";
const char* AcqModeCmnd  =":ACQ:MODE";
const char* AcqAvrgCmnd  =":ACQ:AVER";
const char* AcqSRateCmnd =":ACQ:SRAT?";

const char* InitCmnd     =":WAV:POIN 600;";

// cmnds is a list of commands understood by the instrument that we implement.
// The order is important and must agree with the order of enumerated names
// defined in drvScope.h header file, where the first item is ixBoChOn and
// which corresponds here with the command "SEL:CH%d".
static const char* cmnds[]={
  ChOnCmnd,    ChPosCmnd,   NullCmnd,   ChCplCmnd,  ChSclCmnd,
  WfDatCmnd,   WfNptCmnd,   NullCmnd,   NullCmnd,   WfFormCmnd,
  TmDlyOfsCmnd,NullCmnd,    TmSclCmnd,  NullCmnd,   TrigLevCmnd,
  TrigHoldCmnd,RunCmnd,     StopCmnd,   NullCmnd,   NullCmnd,
  NullCmnd,    OpcCmnd,     NullCmnd,   ResetCmnd,  IdnCmnd,
  NullCmnd,    InitCmnd,    NullCmnd,   ErrMsgCmnd, NullCmnd};

// some of the commands listed above return or require specific keywords.  What
// follows are lists of these keywords for some of the commands.
static const char* chanCpl[] ={"DC","AC","GND"};
static const char* wfForm[]  ={"BYTE","WORD","ASC"};
static const char* trgSwee[] ={"AUTO","NORMAL","SINGLE"};
static const char* trigSou[] ={"CHANNEL1","CHANNEL2","CHANNEL3","CHANNEL4",
				 "EXT",  "EXT5", "ACLINE" };
static const char* trigSlo[] ={"POSITIVE","NEGATIVE","ALTTERNATION"};
static const char* timDivU[] ={"ns","us","ms","s"};
static const char* trigCpl[] ={"AC","DC","LF"};
static const char* trgMode[] ={"EDGE","PULS","VIDEO"};

static const char* tmMode[]  ={"MAIN","DELAYED"};
static const char* acqType[] ={"NORMAL","AVERAGE","PEAKDETECT"};
static const char* acqMode[] ={"RTIME","ETIME"};

// here we construct a list of lists.  Again the order is important.  The total
// number of items in this list must agree with the number of commands above.
// Zero is entered for the command, which does not use list of keywords.
static const char** listIx[]={
        0,0,0,chanCpl,0,
        0,0,0,0,wfForm,
	0,0,0,0,0,
        0,0,0,0,0,
        0,0,0,0,0,
        0,0,0,0,0};
// finally, we construct a list of number of keywords in each list.  Again,
// order is important
static int itemSz[]={
        0,0,0,3,0,  0,0,0,0,3,  0,0,0,0,0,  0,0,0,0,0,
        0,0,0,0,0,  0,0,0,0,0};

static drvDS1x*	_this;
static const char *dname="drvDS1x";

static void inithooks( initHookState state){
/*--------------------------------------------------------------------------*/
  switch(state){
    case initHookAtEnd: _this->postInit(); break;
    default:            break;
  }
}
void drvDS1x::postInit(){
/*-----------------------------------------------------------------------------
 * After IOC init initialization of records from the instrument.
 *---------------------------------------------------------------------------*/
  putInMessgQ( enQuery,_mbboTrCpl,0,0);
  putInMessgQ( enQuery,_mbboTrSwe,0,0);
  putInMessgQ( enQuery,_mbboTrSlo,0,0);
  putInMessgQ( enQuery,_mbboTrSou,0,0);
  putInMessgQ( enQuery,_siTrSta,0,0);
  putInMessgQ( enQuery,_boTmMode,0,0);
  putInMessgQ( enQuery,_mbboAcqTp,0,0);
  putInMessgQ( enQuery,_boAcqMode,0,0);
  putInMessgQ( enQuery,_loAcqAve,0,0);
  putInMessgQ( enQuery,_aoTmOfst,0,0);
  putInMessgQ( enQuery,_aoTmDlyScl,0,0);
  putInMessgQ( enQuery,_aoTmDlyOfs,0,0);
  putInMessgQ( enQuery,_siSRate,0,0);
  putInMessgQ( enQuery,_mbboWfFmt,0,0);
  drvScope::afterInit();
}
/*--- reimplemented virtual functions ---------------------------------------*/
void drvDS1x::updateUser(){
/*-----------------------------------------------------------------------------
 * This is a re-implementation of a virtual function in the base class.
 *---------------------------------------------------------------------------*/
  getBinary( TrigSouCmnd,_mbboTrSou,trigSou,SIZE(trigSou));
  getBinary( TrigSloCmnd,_mbboTrSlo,trigSlo,SIZE(trigSlo));
  getBinary( TrigModeCmnd,_mbboTrMode,trgMode,SIZE(trgMode));
  getBinary( TrigSweCmnd,_mbboTrSwe,trgSwee,SIZE(trgSwee));
}
void drvDS1x::getChanPos( int addr){
/*-----------------------------------------------------------------------------
 * Channel trace position on the screen is controlled from GUI controls and
 * the units are per major screen grid line.  The Rigol scope takes the
 * value in Volts.  Here we account for the difference.
 * addr is parameter library index or addr+1 channel.
 *---------------------------------------------------------------------------*/
  char cmnd[32]; asynStatus stat; double scl,v;
  sprintf( cmnd,ChPosCmnd,addr+1); strcat( cmnd,"?");
  stat=writeRd( cmnd,_rbuf,DBUF_LEN);
  if(stat!=asynSuccess){
    errlogPrintf( "%s::getChanPos:writeRd: failed\n",dname);
    return;
  }
  v=atof(_rbuf);
  getDoubleParam( addr,_aoChScl,&scl);
  if(scl<=0.001){
    errlogPrintf( "%s::getChanPos:(addr=%d) scl=%.2f bad value\n",
		dname,addr,scl);
    return;
  }
  v/=scl;
  setDoubleParam( addr,_aoChPos,v);
  callParamCallbacks(addr);
}
void drvDS1x::setChanPos( int addr,double v){
/*-----------------------------------------------------------------------------
 * v is the trace position in units of grid division in the graph.  Rigol
 * scopes require position in units of Volts.  Here we convert the position
 * value and write it out.
 * addr is parameter library index or addr+1 channel
 *---------------------------------------------------------------------------*/
  char cmnd[32]; double scl,pos;
  getDoubleParam( addr,_aoChScl,&scl);
  if(scl<=0.001){
    errlogPrintf( "%s::setChanPos:(addr=%d) scl=%.2f bad value\n",
		dname,addr,scl);
    return;
  }
  pos=v*scl;
  sprintf( cmnd,ChPosCmnd,addr+1);
  sprintf( cmnd,"%s %f",cmnd,pos);
  command(cmnd);
}
const char** drvDS1x::getCmndList( int cix,uint* ni){
/*-----------------------------------------------------------------------------
 * Overides the empty virtual function in the base class.  It returns a pointer
 * to a list of command items choices for the cix index.  If list is not null,
 * it also returns in ni, number of items in the list.
 *---------------------------------------------------------------------------*/
  int n=SIZE(listIx);
  *ni=0;
  if((cix<0)||(cix>=n)) return(NULL);
  if(listIx[cix]) *ni=itemSz[cix];
  return(listIx[cix]);
}
const char* drvDS1x::getCommand( int cix){
/*-----------------------------------------------------------------------------
 * Overrides the virtual function in the base class.  This function returns
 * a pointer to a command string for the index cix into the list commands.
 * Returns a null pointer if index is out of range.
 *---------------------------------------------------------------------------*/
  uint ix=(uint)cix;
  if(ix>=SIZE(cmnds)) return(NULL);
  if(!strlen(cmnds[ix])) return(NULL);
  return(cmnds[ix]);
}

void drvDS1x::setTimePerDiv( double v){
/*-----------------------------------------------------------------------------
 * v is an analog time/div value.  This is parsed and two integer values are
 * calculated, one is "mantissa" (1..999), while the other is an index to the
 * unit (ns,us,ms,s).  Both are set in the parameter library for the
 * corresponding lo & mbbo records.
  *---------------------------------------------------------------------------*/
  int uix,m; char str[16];
  if(v<0.9999e-6){ uix=0; m=(int)((v*1000.0)*1000000.0+0.5);}
  else if(v<0.9999e-3){ uix=1; m=(int)(v*1000000.0+0.5);}
  else if(v<0.9999){ uix=2; m=(int)(v*1000.0+0.5);}
  else{ uix=3; m=(int)(v+0.5);}
  const char** list=timDivU;
  if(list){
    sprintf( str,"%d %s/div",m,list[uix]);
    setIntegerParam( _loTimDivV,m);
  }
  setIntegerParam( _mbboTimDivU,uix);
  setStringParam( _siTimDiv,str);
  setDoubleParam( _aoTimDiv,v);
}
void drvDS1x::_setTimePerDiv( uint uix){
/*-----------------------------------------------------------------------------
 * Either of the two time base records changed.  Here we recalculate the time
 * base  analog value of horizontal time per division.
 *---------------------------------------------------------------------------*/
  char str[16],cmnd[32]; double v; int m;
  if(uix<0||uix>=SIZE(timDivU)) return;
  getIntegerParam( _loTimDivV,&m);
  switch(uix){
    case 0:	v=m*1.0e-9; break;
    case 1:	v=m*1.0e-6; break;
    case 2:	v=m*1.0e-3; break;
    case 3:	v=m; break;
  }
  sprintf( str,"%d %s/div",m,timDivU[uix]);
  setStringParam( _siTimDiv,str);
  setDoubleParam( _aiTimDiv,v);
  sprintf( cmnd,"%s %e",TmSclCmnd,v);
  command( cmnd);
  setTimePerDiv(v);
}
int drvDS1x::_wfPreamble( char* p,int* ln,int* nb,
			double* yinc,double* yorg,int* yref){
/*-----------------------------------------------------------------------------
 * Unpacks the waveform preamble string.  Returns the length of the preamble
 * as a function value.
 *---------------------------------------------------------------------------*/
  int i=0,npts=0,fmt=0,type,cnt,xref; float xinc,xorg; char* ps;
  strncpy( _wfpre,p,WFPRE); _wfpre[WFPRE-1]=0;
  ps=strtok(p,",");
  while(ps){
//printf( "i=%d, %s\n",i,ps);
    switch(i){
      case 0:	_wfprei[0]=fmt=atoi(ps); break;
      case 1:	_wfprei[1]=type=atoi(ps); break;
      case 2:	_wfprei[2]=npts=atoi(ps); break;
      case 3:	_wfprei[3]=cnt=atoi(ps); break;
      case 4:	_wfpref[0]=xinc=atof(ps); break;
      case 5:	_wfpref[1]=xorg=atof(ps); break;
      case 6:	_wfprei[4]=xref=atoi(ps); break;
      case 7:	_wfpref[2]=(*yinc)=atof(ps); break;
      case 8:	_wfpref[3]=(*yorg)=atof(ps); break;
      case 9:	_wfprei[5]=(*yref)=atoi(ps); break;
      default:	break;
    }
    i++; ps=strtok(0,",");
  }
  if(i!=10){
    printf( "_wfPreamble: i=%d, failed to unpack preamble\n",i); return(0);
  }
  *ln=npts; *nb=fmt;
  return(i);
}
void drvDS1x::getWaveform( int ch){
/*-----------------------------------------------------------------------------
 * Requests waveform data for channel ch (0..3).  This gets waveform preamble
 * and waveform data.  It gets called from the base class via the virtual
 * function mechanism.
 *---------------------------------------------------------------------------*/
  static const char* iam="getWaveform"; static int ctst=0; int ctstmx=20;
  asynStatus stat=asynSuccess; int i,chon,len,n=0,nb,nbyte,yref;
  double vdiv,yinc,yorg; word* pwr=_wfraw; char str[32]; char* pc=_rbuf;
  byte* pb; word* pw; word wtmp; float* pwf=_wfbuf; 

  getIntegerParam( ch,_boChOn,&chon);
  if(chon){
    sprintf( str,WfPreCmnd,ch+1);
    stat=writeRd( str,_rbuf,DBUF_LEN);
    if(stat!=asynSuccess) return;
    i=_wfPreamble( _rbuf,&len,&nbyte,&yinc,&yorg,&yref);
    if(i<=0) return;
    stat=writeRd( ixWfTrace,ch+1,_rbuf,DBUF_LEN);
    if(stat!=asynSuccess) return;
    if(pc[0]!='#'){
      errlogPrintf( "%s::%s first char not '#' in data\n",dname,iam);
      return;
    }
    str[0]=pc[1]; str[1]=0;
    n=atoi(str);
    if((n<1)||(n>9)){
      errlogPrintf( "%s::%s bad second char in data %s\n",dname,iam,str);
      return;
    }
    strncpy( str,&pc[2],n); str[n]=0;
    nb=atoi(str);
    if(nb!=len){
      return;
      errlogPrintf( "%s::%s bad length: nb=%d, len=%d\n",dname,iam,nb,len);
    }
    pb=(byte*)(&_rbuf[n+2]);
    pw=(word*)pb;
    n=WF_LEN;
    vdiv=yinc*25;	// because Rigol gives us yscale in V/25
    n=len<n?len:n;
    n=n<0?0:n;
    for( i=0; i<n; i++,pb++,pw++){
      if(nbyte==1){
	wtmp=(200-(*pb));
      }
      else wtmp=(*pw);
      *pwr=wtmp;
      *pwf=wtmp;
      pwf++; pwr++;
    }
//  _printWF( nbyte,len,n,_rbuf);
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
  else{
    pwf=_wfbuf; n=WF_LEN;
    for(i=0; i<WF_LEN; i++,pwf++) *pwf=1000.0;
  }
  doCallbacksFloat32Array( _wfbuf,n,_wfTrace,ch);
  if((!ch)&&(!((ctst++)%ctstmx))) getString( TrigStCmnd,_siTrSta);
}
void drvDS1x::_printWF( int nbyte,int len,int n,char* pbuf){
/*-----------------------------------------------------------------------------
 * prints the first 100 data points of a waveform in p.
 *---------------------------------------------------------------------------*/
  char str[32]; int nb=MIN(31,MAX(0,n));
  strncpy( str,pbuf,nb); str[nb]=0;
  printf("%s, nbyte=%d, len=%d\n",str,nbyte,len);
  _printWFData();
}
void drvDS1x::_printWFData(){
/*-----------------------------------------------------------------------------
 * prints the first 100 data points of a waveform in p.
 *---------------------------------------------------------------------------*/
  word* pb=_wfraw; int i,npl=20,npts=100;
  printf( "%s\n",_wfpre);
  printf( "%d %d %d %d %f %f %d %f %f %d\n",_wfprei[0],_wfprei[1],_wfprei[2],
	_wfprei[3],_wfpref[0],_wfpref[1],_wfprei[4],_wfpref[2],_wfpref[3],
	_wfprei[5]);
  for( i=0; i<npts; i++,pb++){
    printf(" %d",*pb);
    if(!((i+1)%npl)) printf("\n");
  }
  printf("\n");
}
void drvDS1x::timeDelayStr( int m,int uix){
/*-----------------------------------------------------------------------------
 * This routine is a re-implementation of a virtual in base class.  Here we
 * construct a string and push it to the db record.
 *---------------------------------------------------------------------------*/
  char str[32]; uint ix=(uint)uix;
  if(ix>=SIZE(timDivU)) return;
  sprintf( str,"%d %s",m,timDivU[ix]);
  setStringParam( _siTimDly,str);
}
void drvDS1x::getTrigLevl(){
/*-----------------------------------------------------------------------------
 * Setup slider for the trigger level value.
 *---------------------------------------------------------------------------*/
  int ch,sv,svq,en; double levl,y,scl; asynStatus stat=asynSuccess;
  if((stat=getIntegerParam( 0,_mbboTrSou,&ch))!=asynSuccess) return;
  if(ch<0||ch>=NCHAN) return;
  if((stat=getIntegerParam( ch,_boChOn,&en))!=asynSuccess) return;
  if(!en) return;
  if((stat=getDoubleParam( 0,_aoTrLev,&levl))!=asynSuccess) return;
  if((stat=getDoubleParam( ch,_aoChPos,&y))!=asynSuccess) return;
  if((stat=getDoubleParam( ch,_aoChScl,&scl))!=asynSuccess) return;
  if(0.02>scl) scl=0.02;
  sv=(y+levl/scl)*100.0;
  scl=MAX(1.0,scl);
  svq=128+32*(levl+y)/scl;
  stat=setIntegerParam( 0,_loTrLev,sv);
  callParamCallbacks();
}
void drvDS1x::setTrigLevl( int v){
/*-----------------------------------------------------------------------------
 * trigger level request from a slider.  v is slider value.
 *---------------------------------------------------------------------------*/
  int ch; double levl,y,scl; char cmnd[32]; const char* pcmd;
  getIntegerParam( 0,_mbboTrSou,&ch);
  if(ch<0||ch>=MAX_ADDR) return;
  pcmd=TrigLevCmnd;
  getDoubleParam( ch,_aoChPos,&y);
  getDoubleParam( ch,_aoChScl,&scl);
  levl=(v/100.0-y)*scl;
  sprintf( cmnd,"%s %f",pcmd,levl);
  command( cmnd);
  setDoubleParam( 0,_aoTrLev,levl);
}
void drvDS1x::saveConfig(){
/*-----------------------------------------------------------------------------
 * A prearranged list of scope parameters is saved in a disk file.  The list
 * consists of set command and value pairs separated with either a semicolon or
 * a new line.  This list is used to restore the scope to a state which was
 * saved in the file.  See restoreConfig() method for details.
 *---------------------------------------------------------------------------*/
  static const char* iam="saveConfig"; int i; FILE* fs; char* pcmnd; int st;

  fs=fopen( _fname,"w");
  if(!fs){
    errlogPrintf( "%s::%s:failed to open %s\n",dname,iam,_fname);
    return;
  }
  for( i=1; i<=NCHAN; i++){
    if(!(pcmnd=_getChanCmnds(i))) break;
    st=fputs( pcmnd,fs);
    if(st==EOF){
      errlogPrintf( "%s::%s:fputs failed; i=%d\n",dname,iam,i);
      fclose(fs);
      return;
    }
  }
  if(!pcmnd){
    errlogPrintf( "%s::%s: failed chan %d query\n",dname,iam,i);
    fclose(fs);
    return;
  }
  if(!(pcmnd=_getTrigCmnds())){
    errlogPrintf( "%s::%s: failed in _getTrigCmnds\n",dname,iam);
    fclose(fs);
    return;
  }
  st=fputs( pcmnd,fs);
  if(st==EOF){
    errlogPrintf( "%s::%s:fputs failed (_getTrigCmnds)\n",dname,iam);
    fclose(fs);
    return;
  }
  if(!(pcmnd=_getTimeCmnds())){
    errlogPrintf( "%s::%s: failed in _getTimeCmnds\n",dname,iam);
    fclose(fs);
    return;
  }
  st=fputs( pcmnd,fs);
  if(st==EOF){
    errlogPrintf( "%s::%s:fputs failed (_getTimeCmnds)\n",dname,iam);
    fclose(fs);
    return;
  }
  if(!(pcmnd=_getAcquCmnds())){
    errlogPrintf( "%s::%s: failed in _getAcquCmnds\n",dname,iam);
    fclose(fs);
    return;
  }
  st=fputs( pcmnd,fs);
  if(st==EOF){
    errlogPrintf( "%s::%s:fputs failed (_getAcquCmnds)\n",dname,iam);
    fclose(fs);
    return;
  }
  fclose(fs);
}
char* drvDS1x::_getChanCmnds( int ch){
/*-----------------------------------------------------------------------------
 * Constructs a string of channel commands for save restore.
 *---------------------------------------------------------------------------*/
  char str[32]; char* p; char* pcmnd=0;
  _wbuf[0]=0;
  sprintf( str,ChOnCmnd,ch);
  if((p=_getOneCmnd( str))){
    strcat( _wbuf,p); strcat( _wbuf,"; ");
    sprintf( str,ChSclCmnd,ch);
    if((p=_getOneCmnd( str))){
      strcat( _wbuf,p); strcat( _wbuf,"; ");
      sprintf( str,ChCplCmnd,ch);
      if((p=_getOneCmnd( str))){
	strcat( _wbuf,p); strcat( _wbuf,"; ");
	sprintf( str,ChPosCmnd,ch);
	if((p=_getOneCmnd( str))){
	  strcat( _wbuf,p); strcat( _wbuf,"\n");
	  pcmnd=_wbuf;
	}
      }
    }
  }
  return(pcmnd);
}
char* drvDS1x::_getAcquCmnds(){
/*-----------------------------------------------------------------------------
 * Constructs a string of acquire commands for save restore.
 *---------------------------------------------------------------------------*/
  char* p; char* pcmnd=0;
  _wbuf[0]=0;
  if((p=_getOneCmnd( AcqAvrgCmnd))){
    strcat( _wbuf,p); strcat( _wbuf,"; ");
    if((p=_getOneCmnd( AcqTypeCmnd))){
      strcat( _wbuf,p); strcat( _wbuf,"\n");
      pcmnd=_wbuf;
    }
  }
  return(pcmnd);
}
char* drvDS1x::_getTimeCmnds(){
/*-----------------------------------------------------------------------------
 * Constructs a string of time base commands for save restore.
 *---------------------------------------------------------------------------*/
  char* p; char* pcmnd=0;
  _wbuf[0]=0;
  if((p=_getOneCmnd( TmModeCmnd))){
    strcat( _wbuf,p); strcat( _wbuf,"; ");
    if((p=_getOneCmnd( TmOfstCmnd))){
      strcat( _wbuf,p); strcat( _wbuf,"; ");
      if((p=_getOneCmnd( TmSclCmnd))){
	strcat( _wbuf,p); strcat( _wbuf,"\n");
	pcmnd=_wbuf;
      }
    }
  }
  return(pcmnd);
}
char* drvDS1x::_getTrigCmnds(){
/*-----------------------------------------------------------------------------
 * Constructs a string of trigger commands for save restore.
 *---------------------------------------------------------------------------*/
  char* p; char* pcmnd=0;
  _wbuf[0]=0;
  if((p=_getOneCmnd( TrigCplCmnd))){
    strcat( _wbuf,p); strcat( _wbuf,"; ");
    if((p=_getOneCmnd( TrigModeCmnd))){
      strcat( _wbuf,p); strcat( _wbuf,"; ");
      if((p=_getOneCmnd( TrigSweCmnd))){
        strcat( _wbuf,p); strcat( _wbuf,"; ");
        if((p=_getOneCmnd( TrigLevCmnd))){
          strcat( _wbuf,p); strcat( _wbuf,"; ");
          if((p=_getOneCmnd( TrigSloCmnd))){
            strcat( _wbuf,p); strcat( _wbuf,"; ");
            if((p=_getOneCmnd( TrigSouCmnd))){
              strcat( _wbuf,p); strcat( _wbuf,"\n");
              pcmnd=_wbuf;
            }
          }
        }
      }
    }
  }
  return(pcmnd);
}
char* drvDS1x::_getOneCmnd( const char* cmnd){
/*-----------------------------------------------------------------------------
 * Constructs a query command, writes the query to the scope.  Constructs a
 * set command using the reply to the query.  Returns a pointer to the set
 * command as a function value.
 *---------------------------------------------------------------------------*/
  static char str[32]; char* p;
  sprintf( str,"%s?",cmnd);
  if(command( str,_rbuf,DBUF_LEN)!=asynSuccess){
    errlogPrintf( "%s::_getOneCmnd: command %s failed\n",dname,str);
    return(0);
  }
  sprintf( str,"%s",cmnd);
  strcat( str," ");
  strncat( str,_rbuf,16); str[30]=0;
  p=strchr( str,'\n');
  if(p) *p=0;
  return(str);
}
void drvDS1x::restoreConfig(){
/*-----------------------------------------------------------------------------
 * Reads a set of setup commands from a disk file, file path and name is in
 * _filename, and sends these commands to the scope, resulting in a desired
 * state, which was previously saved in the file.  The list of commands read
 * from the file consists of groups of commands separated with commas.  Within
 * each group commands are separated with a ';' character.  A group is
 * terminated with a new line character.  A whole group is
 * sent to the scope, one at a time.
 *---------------------------------------------------------------------------*/
  static const char* iam="_restSetup";
  asynStatus stat=asynError; FILE* fs; char* p; int n=0;
  fs=fopen( _fname,"r");
  if(!fs){
    errlogPrintf( "%s::%s: failed to open %s\n",dname,iam,_fname);
    return;
  }
  stat=asynSuccess;
  while(1){
    p=fgets( _rbuf,DBUF_LEN,fs);
    if(!p) break;
    p=strchr( _rbuf,'\n');
    if(p) *p=0;
    stat=command(_rbuf);
    if(stat!=asynSuccess) break;
    n++;
  }
  if((!p)&&(n!=7)) errlogPrintf( "%s::%s: failed in fgets; n=%d\n",dname,iam,n);
  if(stat!=asynSuccess)
    errlogPrintf( "%s::%s: failed in command; n=%d\n",dname,iam,n);
  if(n!=7) errlogPrintf( "%s::%s: wrong number of groups: %d\n",dname,iam,n);
  if((!p)||(n!=7)) stat=asynError;
  fclose(fs);
//  _update1();
//  _updateCh(0);
//  _updateCh(1);
//  _updateCh(2);
//  _updateCh(3);
}
asynStatus drvDS1x::getCmnds( int ix,int addr){
/*-----------------------------------------------------------------------------
 * This virtual function reimplements the one in the base class.
 * This routine is called from the pollerThread in the base class when it
 * receives a message on the message queue.
 * ix is the index into parameter library (or the reason)
 * addr is channel or address
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int jx=ix-_firstix;
  switch( jx){
    case ixBoTmMode:	getBinary( TmModeCmnd,ix,tmMode,SIZE(tmMode)); break;
    case ixMbboTrMode:	getBinary( TrigModeCmnd,ix,trgMode,3); break;
    case ixMbboTrSou:	getBinary( TrigSouCmnd,ix,trigSou,SIZE(trigSou)); break;
    case ixMbboTrSlo:	getBinary( TrigSloCmnd,ix,trigSlo,SIZE(trigSlo)); break;
    case ixMbboTrCpl:	getBinary( TrigCplCmnd,ix,trigCpl,SIZE(trigCpl)); break;
    case ixMbboTrSwe:	getBinary( TrigSweCmnd,ix,trgSwee,SIZE(trgSwee)); break;
    case ixMbboAcqTp:	getBinary( AcqTypeCmnd,ix,acqType,SIZE(acqType)); break;
    case ixBoAcqMode:	getBinary( AcqModeCmnd,ix,acqMode,SIZE(acqMode)); break;
    case ixMbboWfFmt:	getBinary( WfFormCmnd,ix,wfForm,SIZE(wfForm)); break;
    case ixAoTmDlyScl:	getFloat( TmDlySclCmnd,ix); break;
    case ixAoTmDlyOfs:	getFloat( TmDlyOfsCmnd,ix); break;
    case ixSiTrSta:	getString( TrigStCmnd,ix); break;
    case ixSiSRate:	getString( AcqSRateCmnd,ix); break;
    default:            stat=drvScope::getCmnds(ix,addr); break;
  }
  callParamCallbacks( addr);
  return(stat);
}
asynStatus drvDS1x::putIntCmnds( int ix,int addr,int v){
/*-----------------------------------------------------------------------------
 * This is a reimplementation of a virtual function in the base class.
 * It is called from the pollerThread routine in the base dlass when it
 * receives a message on the message queue.
 * ix is the index into parameter library (or the reason)
 * addr is channel or address
 * v is a possible integer set value.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess;
  int jx=ix-_firstix,uix;
  switch( jx){
    case ixMbboTrMode:	setBinary(v,TrigModeCmnd,trgMode,SIZE(trgMode));
			setIntegerParam( ix,v); break;
    case ixMbboTrCpl:	setBinary(v,TrigCplCmnd,trigCpl,SIZE(trigCpl));
			setIntegerParam( ix,v); break;
    case ixMbboTrSou:	setBinary(v,TrigSouCmnd,trigSou,SIZE(trigSou));
			break;
    case ixMbboTrSlo:	setBinary(v,TrigSloCmnd,trigSlo,SIZE(trigSlo));
			break;
    case ixMbboTrSwe:	setBinary(v,TrigSweCmnd,trgSwee,SIZE(trgSwee));
			break;
    case ixLoTimDivV:	setIntegerParam( _loTimDivV,v);
			getIntegerParam( _mbboTimDivU,&uix);
			_setTimePerDiv( uix);
			break;
    case ixMbboTimDivU:	_setTimePerDiv( v); break;
    case ixBoAutoScl:	command( AutoCmnd); break;
    case ixMbboAcqTp:	setBinary( v,AcqTypeCmnd,acqType,SIZE(acqType));
			break;
    case ixBoAcqMode:	setBinary( v,AcqModeCmnd,acqMode,SIZE(acqMode));
			break;
    case ixMbboWfFmt:	setBinary( v,WfFormCmnd,wfForm,SIZE(wfForm));
			break;
    case ixBoTmMode:	setBinary(v,TmModeCmnd,tmMode,SIZE(tmMode));
			break;
    default:		stat=drvScope::putIntCmnds( ix,addr,v); break;
  }
  callParamCallbacks(addr);
  return(stat);
}
asynStatus drvDS1x::putFltCmnds( int ix,int addr,float v){
/*-----------------------------------------------------------------------------
 * This routine is a reimplementation of a virtual function in base class.
 * It is called from the pollerThread routine in base class when it receives
 * a message on the message queue.
 * ix is the index into parameter library (or the reason)
 * addr is channel or address
 * v is a possible integer set value.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int jx=ix-_firstix; char cmnd[32];
  switch( jx){
    case ixAoTimDiv:	sprintf( cmnd,"%s %f",TmSclCmnd,v);
			command( cmnd);
			setDoubleParam( _aoTimDiv,v);
			setTimePerDiv(v); break;
    case ixAoTmOfst:	sprintf( cmnd,"%s %f",TmOfstCmnd,v);
			command( cmnd);
			drvScope::timeDelayStr(v); break;
    case ixAoTmDlyOfs:	sprintf( cmnd,"%s %f",TmDlyOfsCmnd,v);
			command( cmnd); break;
    case ixAoTmDlyScl:	sprintf( cmnd,"%s %f",TmDlySclCmnd,v);
			command( cmnd); break;
    default:            stat=drvScope::putFltCmnds( ix,addr,v); break;
  }
  callParamCallbacks(addr);
  return(stat);
}
asynStatus drvDS1x::writeInt32( asynUser* pau,epicsInt32 v){
/*-----------------------------------------------------------------------------
 * This method overrides the virtual method in asynPortDriver.  Here we service
 * all write requests comming from EPICS records.
 * Parameters:
 *  pau         (in) structure containing addr and reason.
 *  v           (in) this is the command index, which together with
 *              pau->reason define the command to be sent.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int ix,jx,addr;
  stat=getAddress(pau,&addr); if(stat!=asynSuccess) return(stat);
  ix=pau->reason;
  jx=ix-_firstix;
  switch( jx){
    case ixBoDump:	printf("%s::writeInt32:ixBoDump\n",dname);
			if(!v) break;
			_printWFData(); break;
    default:		stat=drvScope::writeInt32( pau,v); break;
  }
  callParamCallbacks(addr);
  return(stat);
}
asynStatus drvDS1x::writeFloat64( asynUser* pau,epicsFloat64 v){
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
    default:		stat=drvScope::writeFloat64( pau,fv); break;
  }
  return(stat);
}
drvDS1x::drvDS1x(const char* port, const char* udp):
			drvScope(port, udp) {
/*------------------------------------------------------------------------------
 * Constructor for the drvDS1x class. Calls constructor for the asynPortDriver
 * base class. Where
 *   portName The name of the asyn port driver to be created.
 *   udpPort is the actual device port name.
 *   np is the total number of items in the parameter library.
 *---------------------------------------------------------------------------*/
  createParam( mbboTrModeStr,	asynParamInt32,		&_mbboTrMode);
  createParam( mbboTrSouStr,	asynParamInt32,		&_mbboTrSou);
  createParam( mbboTrCplStr,	asynParamInt32,		&_mbboTrCpl);
  createParam( mbboTrSloStr,	asynParamInt32,		&_mbboTrSlo);
  createParam( mbboTrSweStr,	asynParamInt32,		&_mbboTrSwe);

  createParam( siTrStaStr,	asynParamOctet,		&_siTrSta);
  createParam( loTimDivVStr,	asynParamInt32,		&_loTimDivV);
  createParam( mbboTimDivUStr,	asynParamInt32,		&_mbboTimDivU);
  createParam( boTmModeStr,	asynParamInt32,		&_boTmMode);
  createParam( aoTimDivStr,	asynParamFloat64,	&_aoTimDiv);

  createParam( aoTmOfstStr,	asynParamFloat64,	&_aoTmOfst);
  createParam( aoTmDlyOfsStr,	asynParamFloat64,	&_aoTmDlyOfs);
  createParam( aoTmDlySclStr,	asynParamFloat64,	&_aoTmDlyScl);
  createParam( siSRateStr,	asynParamOctet,		&_siSRate);
  createParam( boAutoSclStr,	asynParamInt32,		&_boAutoScl);

  createParam( loAcqAveStr,	asynParamInt32,		&_loAcqAve);
  createParam( mbboAcqTpStr,	asynParamInt32,		&_mbboAcqTp);
  createParam( boAcqModeStr,	asynParamInt32,		&_boAcqMode);
  createParam( mbboWfFmtStr,	asynParamInt32,		&_mbboWfFmt);
  createParam( boDumpStr,	asynParamInt32,		&_boDump);

  _firstix=_mbboTrMode;

  setStringParam( _siName,dname);
  message( "Constructor drvDS1x: success");
  callParamCallbacks();
}


// Configuration routines.  Called directly, or from the iocsh function below
extern "C" {

int drvDS1xConfigure(const char* port, const char* udp) {
/*-----------------------------------------------------------------------------
 * EPICS iocsh callable function to call constructor for the drvDS1x class.
 *  port The name of the asyn port driver to be created.
 *  udp is the IO port.
 *---------------------------------------------------------------------------*/
  _this = new drvDS1x(port, udp);
  return(asynSuccess);
}

/* EPICS iocsh shell commands */

static const iocshArg initArg0 = { "port",iocshArgString};
static const iocshArg initArg1 = { "udp",iocshArgString};
static const iocshArg * const initArgs[] = {&initArg0,&initArg1};
static const iocshFuncDef initFuncDef = {"drvDS1xConfigure",2,initArgs};
static void initCallFunc(const iocshArgBuf *args){
  drvDS1xConfigure(args[0].sval, args[1].sval);
}
void drvDS1xRegister(void){
  iocshRegister(&initFuncDef,initCallFunc);
  initHookRegister(&inithooks);
}
epicsExportRegistrar(drvDS1xRegister);
}
