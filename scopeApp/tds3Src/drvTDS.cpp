/* drvTDS.cpp
 * Asyn driver to control Tektronix TDS 3000 series scopes.  This is a
 * subclass of drvScope base class.
 * Started on 05/22/2015, zms.
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

const char* ChOnCmnd       = "SEL:CH%d";
const char* ChPosCmnd      = "CH%d:POS";
const char* ChImpCmnd      = "CH%d:IMP";
const char* ChCplCmnd      = "CH%d:COUP";
const char* ChSclCmnd      = "CH%d:SCA";
const char* WfDatCmnd      = "DAT:SOU CH%d; :WAVF?";
const char* WfNptCmnd      = "HOR:RECORDL";
const char* WfWidCmnd      = "DAT:WID";
const char* WfStrtCmnd     = "DAT:STAR";
const char* WfStopCmnd     = "DAT:STOP";
const char* WfFormCmnd     = "DAT:ENC";
const char* TmDlyOfsCmnd   = "HOR:DEL:TIM";
const char* TmDlyEnCmnd    = "HOR:DEL:STATE";
const char* TmSclCmnd      = "HOR:MAI:SCA";
const char* TrigPosCmnd    = "HOR:TRIG:POS";
const char* TrigLevCmnd    = "TRIG:A:LEV";
const char* TrigHoldCmnd   = "TRIG:A:HOL";
const char* TrigModeCmnd   = "TRIG:A:MOD";
const char* TrigSouCmnd    = "TRIG:A:EDG:SOU";
const char* TrigSloCmnd    = "TRIG:A:EDG:SLO";
const char* AcqStateCmnd   = "ACQ:STATE";
const char* TrigStaCmnd    = "TRIG:STATE?";
const char* RunCmnd        = "ACQ:STATE RUN";
const char* StopCmnd       = "ACQ:STATE STOP";
const char* EseCmnd        = "*ESE";
const char* ClsCmnd        = "*CLS";
const char* EsrCmnd        = "*ESR?";
const char* ECodeCmnd      = "EVENT?";
const char* EvqCmnd        = "EVQ?";
const char* OpcCmnd        = "*OPC";
const char* StbCmnd        = "*STB?";
const char* ResetCmnd      = "*RST";
const char* RecallCmnd     = "*RCL %d";
const char* SaveCmnd       = "*SAV %d";
const char* IdnCmnd        = "*IDN?";
const char* IPAddrCmnd     = "ETHER:IPADD?";
const char* WfSouCmnd      = "DATA:SOU";
const char* InitCmnd       = "*CLS; :DAT:ENC RIB; :HOR:RECORDL 500; :HEAD OFF; :VERB ON; :DAT:STAR 1; :DAT:STOP 500; HOR:DEL:STATE 1";
const char* HeaderCmnd     = "HEAD?";
const char* GetConfCmnd    = "*LRN?";
const char* ErrMsgCmnd     = "EVM?";
const char* MeasValCmnd    = "MEASU:MEAS%d:VAL?";
const char* MeasUnitsCmnd  = "MEASU:MEAS%d:UNI?";

// cmnds is a list of commands understood by the instrument that we implement.
// The order is important and must agree with the order of enumerated names
// defined in drvScope.h header file, where the first item is ixBoChOn and
// which corresponds here with the command "SEL:CH%d".
static const char* cmnds[]={
  ChOnCmnd,     ChPosCmnd,    ChImpCmnd,   ChCplCmnd,  ChSclCmnd,
  WfDatCmnd,    WfNptCmnd,    WfStrtCmnd,  WfStopCmnd, WfFormCmnd,
  TmDlyOfsCmnd, TmDlyEnCmnd,  TmSclCmnd,   TrigPosCmnd,TrigLevCmnd,
  TrigHoldCmnd, RunCmnd,      StopCmnd,    EseCmnd,    ClsCmnd,
  EsrCmnd,      OpcCmnd,      StbCmnd,     ResetCmnd,  IdnCmnd,
  IPAddrCmnd,   InitCmnd,     GetConfCmnd, ErrMsgCmnd, MeasValCmnd,
  MeasUnitsCmnd};

// some of the commands listed above return or require specific keywords.  What
// follows are lists of these keywords for some of the commands.
static const char* chanImp[]={"FIFTY","MEG"};
static const char* chanCpl[]={"DC","AC","GND"};
static const char* chanScl[]={"1.0E-3","2.0E-3","5.0E-3","1.0E-2","2.0E-2",
	"5.0E-2","1.0E-1","2.0E-1","5.0E-1","1.0E0","2.0E0","5.0E0","1.0E1"};
static const char* timDivV[]={"1","2","4","10","20","40","100","200","400"};
static const char* timDivU[]={"ns","us","ms","s"};
static const char* trgMode[]={"NORMAL","AUTO"};
static const char* trigSou[]={"CH1","CH2","CH3","CH4","LINE","VERTICAL",
				"EXT10","EXT"};
static const char* trigSlo[]={"FALL","RISE"};
static const char* acqStat[]={"RUNSTOP","SEQUESCE"};
static const char* trigSta[]={"AUTO","ARMED","READY","SAVE","TRIGGER"};
static const char* dataFmt[]={"ASCII","RIBINARY",
			"RPBINARY","SRIBINARY","SRPBINARY"};

// here we construct a list of lists.  Again the order is important.  The total
// number of items in this list must agree with the number of commands above.
// Zero is entered for the command, which does not use list of keywords.
static const char** listIx[]={
	0,0,chanImp,chanCpl,chanScl,
	0,0,0,0,dataFmt,
	0,0,0,0,0,
	0,0,0,0,0,
	0,0,0,0,0,
	0,0,0,0};

// finally, we construct a list of number of keywords in each list.  Again,
// order is important.
static int itemSz[]={
	0,0,2,3,13, 0,0,0,0,5,  0,0,0,0,0,  0,0,0,0,0,
	0,0,0,0,0,  0,0,0,0};

static const int hsc[]={   1,   2,   4,  10};
static const int hsx[]={ 225, 200, 150,   0};
static const int hnp[]={  51, 101, 201, 500};


static drvTDS*	_this;
static const char *dname="drvTDS";

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
    case initHookAtEnd: _this->postInit(); break;
    default:            break;
  }
}

void drvTDS::postInit(){
/*-----------------------------------------------------------------------------
 * After IOC init.
 *---------------------------------------------------------------------------*/
    printf("$$$$$$$$$$$$$$ drvTDS::postInit\n");
  putInMessgQ( enQuery,_mbboTrSou,0,0);
  putInMessgQ( enQuery,_boTrSlo,0,0);
  putInMessgQ( enQuery,_mbbiTrSta,0,0);
  putInMessgQ( enQuery,_boTrMode,0,0);
  putInMessgQ( enQuery,_siSource,0,0);
  putInMessgQ( enQuery,_siHead,0,0);
  putInMessgQ( enQuery,_loWfWid,0,0);
  putInMessgQ( enQuery,_liEvQ,0,0);
  putInMessgQ( enQuery,_biAcqStat,0,0);
  drvScope::afterInit();
}

void drvTDS::updateUser(){
/*-----------------------------------------------------------------------------
 * This is a re-implementation of a virtual function in the base class.
 *---------------------------------------------------------------------------*/
    printf("**********drvTDS::updateUser()**************\n");
  getBinary( TrigSouCmnd,_mbboTrSou,trigSou,SIZE(trigSou));
  getBinary( TrigSloCmnd,_boTrSlo,trigSlo,SIZE(trigSlo));
  getBinary( TrigStaCmnd,_mbbiTrSta,trigSta,SIZE(trigSta));
  getBinary( TrigModeCmnd,_boTrMode,trgMode,SIZE(trgMode));
}

void drvTDS::getHSParams( double hs,int* x0,int* np){
  __getHSParams( hs,x0,np);
}

asynStatus drvTDS::trigState(){
/*-----------------------------------------------------------------------------
 * Gets trigger state, which gets posted.
 *---------------------------------------------------------------------------*/
  asynStatus stat;
  stat=getBinary( TrigStaCmnd,_mbbiTrSta,trigSta,SIZE(trigSta));
  return(stat);
}

int drvTDS::isTriggered(){
/*-----------------------------------------------------------------------------
 * Returns true if scope is in a triggered state, returns false otherwise.
 *---------------------------------------------------------------------------*/
  asynStatus stat=trigState(); int trst;
  if(stat!=asynSuccess){
    printf( "%s::isTriggered: failed in trigState\n",dname);
    return(0);
  }
  getIntegerParam( 0,_mbbiTrSta,&trst);
  if(trst==4) return(1);
  return(0);
}

const char** drvTDS::getCmndList( int cix,uint* ni){
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

const char* drvTDS::getCommand( int cix){
/*-----------------------------------------------------------------------------
 * Overrides the virtual function in the base class.  This function returns
 * a pointer to a command string for the index cix into the list commands.
 * Returns a null pointer if index is out of range.
 *---------------------------------------------------------------------------*/
  if((cix<0)||(cix>=SIZE(cmnds))) return(NULL);
  if(!strlen(cmnds[cix])) return(NULL);
  return(cmnds[cix]);
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
printf( "_wfPreamble: nbyt=%d,nbit=%d,enc=%s,bfmt=%s\n",nbyt,nbit,enc,bfmt);
printf( "_wfPreamble: bord=%s,len=%d,ids=%s,ptfm=%s\n",bord,len,ids,ptfm);
printf( "_wfPreamble: xinc=%g,ptof=%d,xzr=%g,xunt=%s\n",xinc,ptof,xzr,xunt);
printf( "_wfPreamble: ymult=%g,yzr=%g,yof=%g,yunt=%s\n",ymult,yzr,yof,yunt);
printf( "_wfPreamble: j=%d,i=%d,wd=%d\n",j,i,wd);
  *ln=len; *nb=nbyt; *ym=ymult; *yz=yzr; *yo=yof;
  return(i);
}

void drvTDS::getWaveform( int ch){
/*-----------------------------------------------------------------------------
 * Requests waveform data for channel ch (0..3).  This gets waveform preamble
 * and waveform data.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int i,j,chon,len,n,nbyte,x0,np=0;
  double hs,pos,vdiv,ymult,yzr,yof; float* pwr=_wfraw;
  char* pb; short* pw; float ftmp; float* pwf=_wfbuf;

  getIntegerParam( ch,_boChOn,&chon);
  if (chon) {
    stat=writeRd( ixWfTrace,ch+1,_rbuf,DBUF_LEN);
    if(stat != asynSuccess) {
        printf("getWaveform: ch=%d, stat=%d, rbuf=%s\n", ch, stat, _rbuf);
        return;
    }
    getDoubleParam( ch, _aoChPos,&pos);
    getDoubleParam( ch,_aoChScl,&vdiv);
    getDoubleParam( 0,_aiTimDiv,&hs);
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
  } else {
    printf("getWaveform: ch=%d not on\n");
    for (i=0; i<WF_LEN; i++,pwf++) {
        *pwf = 1000.0;
    }
  }
  
    doCallbacksFloat32Array( _wfbuf,WF_LEN,_wfTrace,ch);
}

void drvTDS::getMeasurements() {
/*-----------------------------------------------------------------------------
 *---------------------------------------------------------------------------*/
    for (int i=0; i<_num_meas; i++) {
        _getMeasData(i, _aiMeas1+i);
    }
}

void drvTDS::_getMeasData(int meas_num, int param){
/*-----------------------------------------------------------------------------
 *---------------------------------------------------------------------------*/
    asynStatus stat = asynSuccess;
    char str[64];
    double val;

    // Measurement number starts at 1
    meas_num += 1;
    
    if (meas_num > _num_meas) {
        return;
    }
   
    // Get measurement value 
    sprintf(str, MeasValCmnd, meas_num);
    stat = writeRd(str, _rbuf, DBUF_LEN);
    if(stat != asynSuccess) {
        printf("drvTDS::getMeasData: stat=%d, cmd=%s, rbuf=%s\n", stat, str, _rbuf);
        return;
    }
    val = atof(_rbuf);
    setDoubleParam(param, val);

    // Get measurement units
    sprintf(str, MeasUnitsCmnd, meas_num);
    stat = writeRd(str, _rbuf, DBUF_LEN);
    if(stat != asynSuccess) {
        printf("drvTDS::getMeasData: stat=%d, cmd=%s, rbuf=%s\n", stat, str, _rbuf);
        return;
    }
    setStringParam(param, _rbuf);

    callParamCallbacks();
}

void drvTDS::getChanScl( int ch){
/*-----------------------------------------------------------------------------
 * Re-implemntation of a virtual function in base class.
 *---------------------------------------------------------------------------*/
  getBinaryCh( ChSclCmnd,ch,_mbboChScl,chanScl,SIZE(chanScl));
}

void drvTDS::timeDelayStr( int m,int uix){
/*-----------------------------------------------------------------------------
 * This routine is a re-implementation of a virtual in base class.  Here we
 * construct a string and push it to the db record.
 *---------------------------------------------------------------------------*/
  char str[32];
  if((uix<0)||(uix>=SIZE(timDivU))) return;
  sprintf( str,"%d %s",m,timDivU[uix]);
  setStringParam( _siTimDly,str);
}

void drvTDS::getTrigLevl(){
/*-----------------------------------------------------------------------------
 * Setup slider for the trigger level value.
 *---------------------------------------------------------------------------*/
  int ch,sv,en; double levl,y,scl;
  getIntegerParam( 0,_mbboTrSou,&ch);
  if(ch<0||ch>=MAX_ADDR) return;
  getIntegerParam( ch,_boChOn,&en);
  if(!en) return;
  getDoubleParam( 0,_aoTrLev,&levl);
  getDoubleParam( ch,_aoChPos,&y);
  getDoubleParam( ch,_aoChScl,&scl);
  if(-0.000001<scl&&scl<0.000001) scl=1.0;
  sv=(y+levl/scl)*100;
  setIntegerParam( 0,_loTrLev,sv);
}

void drvTDS::setTrigLevl( int v){
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

void drvTDS::setTimePerDiv( double v){
/*-----------------------------------------------------------------------------
 * v is an analog time/div value.  This is parsed and two integer values are
 * calculated, one is "mantissa" like 1,2,4,10,20,40,100,200,400, while the
 * other is an index to the unit (ns,us,ms,s).  Both are set in the parameter
 * library for the corresponding mbbi/mbbo records.
 * Changed 1.0e-6 to 0.9e-6, etc, because if(v<1.0e-6) was returning true when
 * v=1e-6, therefore calculating wrong mantisa and exponent.
 *---------------------------------------------------------------------------*/
  int mix,uix,m,np=0; char str[16];
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
  uint ni=SIZE(timDivU); int x0;
  const char** list=timDivU;
  if(list){
    sprintf( str,"%d %s/div",m,list[uix]);
    setIntegerParam( _mbboTimDivV,mix);
  }
  setIntegerParam( _mbboTimDivU,uix);
  setStringParam( _siTimDiv,str);
  __getHSParams( v,&x0,&np);		// EDM needs to know how many x point
  setIntegerParam( _liXNpts,np);	// of data to dislay.
}

void drvTDS::_setTimePerDiv( uint vix,uint uix){
/*-----------------------------------------------------------------------------
 * Either of the two time per division mbbo records changed.  Indeces to the
 * value (vix) and unit (uix) are used to calculate the new analog value of
 * horizontal time per division.
 *---------------------------------------------------------------------------*/
  int m=0; uint niV=SIZE(timDivV),niU=SIZE(timDivU);
  double v=0; char cmnd[32]; char str[16];
  const char** listV=timDivV; const char** listU=timDivU;
  if(vix<0||vix>=niV||uix<0||uix>=niU) return;
  m=atoi(listV[vix]);
  switch(uix){
    case 0:     v=m*1.0e-9; break;
    case 1:     v=m*1.0e-6; break;
    case 2:     v=m*1.0e-3; break;
    case 3:     v=m; break;
  }
  sprintf( str,"%s %s/div",listV[vix],listU[uix]);
  setStringParam( _siTimDiv,str);
  setDoubleParam( _aiTimDiv,v);
  sprintf( cmnd,"%s %e",TmSclCmnd,v);
  command( cmnd);
  setTimePerDiv(v);
}

asynStatus drvTDS::getCmnds( int ix,int addr){
/*-----------------------------------------------------------------------------
 * This virtual function reimplements the one in the base class.
 * This routine is called from the pollerThread in the base class when it
 * receives a message on the message queue.
 * ix is the index into parameter library (or the reason)
 * addr is channel or address
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int jx=ix-_firstix,v; char cmnd[32];
    printf("drvTDS::getCmnds(): jx=%d\n", jx);
//  _opc();
  switch( jx){
    case ixBoTrMode:    getBinary(TrigModeCmnd, ix, trgMode, 2); break;
    case ixMbboTrSou:   getBinary(TrigSouCmnd, ix, trigSou, SIZE(trigSou)); break;
    case ixBoTrSlo:     getBinary(TrigSloCmnd, ix, trigSlo, 2); break;
    case ixMbbiTrSta:   getBinary(TrigStaCmnd, ix, trigSta, 5); break;
    case ixMbboChScl:   getBinaryCh(ChSclCmnd, addr+1, ix, chanScl, SIZE(chanScl)); break;
    case ixSiSource:    stat = getString(WfSouCmnd, ix); break;
    case ixLoWfWid:     stat = getInt(WfWidCmnd, ix); break;
    case ixLiEvQ:       stat = getInt(EvqCmnd, ix); break;
    case ixBiAcqStat:   stat = getInt(AcqStateCmnd, ix); break;
    case ixSiHead:      stat = getString(HeaderCmnd, ix); break;
    default:            stat = drvScope::getCmnds(ix, addr); break;
  } 
  callParamCallbacks( addr);
  return(stat);
}

asynStatus drvTDS::putIntCmnds( int ix,int addr,int v){
/*-----------------------------------------------------------------------------
 * This is a reimplementation of a virtual function in the base class.
 * It is called from the pollerThread routine in the base dlass when it
 * receives a message on the message queue.
 * ix is the index into parameter library (or the reason)
 * addr is channel or address
 * v is a possible integer set value.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; char cmnd[32];
  int jx=ix-_firstix;
  switch( jx){
    case ixMbboChScl:   setIntegerParam( addr,ix,v);
                        setBinaryCh( v,addr,ChSclCmnd,chanScl,SIZE(chanScl));
                        getFloatCh( ChSclCmnd,addr+1,_aoChScl);
                        break;
    case ixMbboTimDivV: stat=getIntegerParam( _mbboTimDivU,&jx);
                        _setTimePerDiv( v,jx);
                        break;
    case ixMbboTimDivU: getIntegerParam( _mbboTimDivV,&jx);
                        _setTimePerDiv( jx,v);
                        break;
    case ixLoWfWid:	setInt( jx,WfDatCmnd,v,ix); break;
    case ixBoTrMode:    setBinary(v,TrigModeCmnd,trgMode,SIZE(trgMode));
                        setIntegerParam( ix,v);
                        break;
    case ixMbboTrSou:   setBinary(v,TrigSouCmnd,trigSou,SIZE(trigSou));
                        break;
    case ixBoTrSlo:     setBinary(v,TrigSloCmnd,trigSlo,SIZE(trigSlo));
                        break;
    case ixLoRecall:	sprintf( cmnd,RecallCmnd,v);
			command( cmnd);
			update();
			break;
    case ixLoStore:	sprintf( cmnd,SaveCmnd,v);
                        command( cmnd); break;
    default:		stat=drvScope::putIntCmnds( ix,addr,v); break;
  }
  callParamCallbacks(addr);
  return(stat);
}

asynStatus drvTDS::putFltCmnds( int ix,int addr,float v){
/*-----------------------------------------------------------------------------
 * This routine is a reimplementation of a virtual function in base class.
 * It is called from the pollerThread routine in base class when it receives
 * a message on the message queue.
 * ix is the index into parameter library (or the reason)
 * addr is channel or address
 * v is a possible integer set value.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int jx=ix-_firstix;
  switch( jx){
    default:		stat=drvScope::putFltCmnds( ix,addr,v); break;
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
  asynStatus stat=asynSuccess; int ix=pau->reason,jx,addr;

  stat=getAddress(pau,&addr); if(stat!=asynSuccess) return(stat);
  jx=ix-_firstix;
  switch( jx){
    default:		stat=drvScope::writeInt32( pau,v); break;
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
  asynStatus stat=asynSuccess; int ix,addr;

  stat=getAddress(pau,&addr); if(stat!=asynSuccess) return(stat);
  ix=pau->reason-_firstix;
  switch( ix){
    default:		stat=drvScope::writeFloat64( pau,v); break;
  }
  return(stat);
}

drvTDS::drvTDS(const char* port, const char* udp):
			drvScope(port, udp),
             _num_meas(4) {
/*------------------------------------------------------------------------------
 * Constructor for the drvTDS class. Calls the base class constructor, which
 * in tern calls the asynPortDriver constructor.
 * base class. Where
 *   portName The name of the asyn port driver to be created.
 *   udpPort is the actual device port name.
 *   np is the total number of items for the parameter library.
 *---------------------------------------------------------------------------*/
  createParam( loWfWidStr,	asynParamInt32,         &_loWfWid);
  createParam( boTrModeStr,     asynParamInt32,         &_boTrMode);
  createParam( mbboTrSouStr,    asynParamInt32,         &_mbboTrSou);
  createParam( boTrSlopeStr,    asynParamInt32,         &_boTrSlo);
  createParam( mbbiTrStaStr,    asynParamInt32,         &_mbbiTrSta);

  createParam( mbboChSclStr,    asynParamInt32,         &_mbboChScl);
  createParam( mbboTimDivVStr,  asynParamInt32,         &_mbboTimDivV);
  createParam( mbboTimDivUStr,  asynParamInt32,         &_mbboTimDivU);
  createParam( biAcqStatStr,    asynParamInt32,         &_biAcqStat);
  createParam( liEvQStr,        asynParamInt32,         &_liEvQ);

  createParam( liEvQtyStr,      asynParamInt32,         &_liEvQty);
  createParam( loRecallStr,     asynParamInt32,         &_loRecall);
  createParam( loStoreStr,      asynParamInt32,         &_loStore);
  createParam( siSourceStr,	asynParamOctet,         &_siSource);
  createParam( siHeadStr,       asynParamOctet,         &_siHead);

  createParam(aiMeas1Str,       asynParamFloat64,       &_aiMeas1);
  createParam(aiMeas2Str,       asynParamFloat64,       &_aiMeas2);
  createParam(aiMeas3Str,       asynParamFloat64,       &_aiMeas3);
  createParam(aiMeas4Str,       asynParamFloat64,       &_aiMeas4);
  createParam(siMeas1UnitsStr,  asynParamOctet,         &_siMeas1Units);
  createParam(siMeas2UnitsStr,  asynParamOctet,         &_siMeas2Units);
  createParam(siMeas3UnitsStr,  asynParamOctet,         &_siMeas3Units);
  createParam(siMeas4UnitsStr,  asynParamOctet,         &_siMeas4Units);

  _firstix=_loWfWid;

  setStringParam( _siName,dname);
  setIntegerParam( _loStore,1);
  setIntegerParam( _loRecall,1);
   message( "Constructor drvTDS success");
  callParamCallbacks( 0);
}


// Configuration routines.  Called directly, or from the iocsh function below
extern "C" {

int drvTDSConfigure(const char* port, const char* udp) {
/*-----------------------------------------------------------------------------
 * EPICS iocsh callable function to call constructor for the drvTDS class.
 *  port The name of the asyn port driver to be created.
 *  udp is the IO port.
 *---------------------------------------------------------------------------*/
  _this = new drvTDS(port, udp);
  return asynSuccess;
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

