#ifndef DRVMDO_H
#define DRVMDO_H

/* drvMDO.h
 * Asyn driver to control Tektronix MDO34 series scopes.
 * asynPortDriver --> drvScope --> drvTek --> drvMDO
 *---------------------------------------------------------------------------*/

#include "drvTek.h"


class drvMDO: public drvTek {
public:
    drvMDO(const char* port, const char* udp);
    virtual ~drvMDO(){}
  
protected:
    virtual int _parseWfPreamble(const char* buf, int*, int*, double*, double*, double*);

private:
    void _initializeParams();
};

#endif

