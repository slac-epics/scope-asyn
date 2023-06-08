#ifndef DRVTDS_H
#define DRVTDS_H

/* drvTDS.h
 * Asyn driver to control Tektronix TDS 3000 series scopes.
 * asynPortDriver --> drvScope --> drvTek --> drvTDS
 *---------------------------------------------------------------------------*/

#include "drvTek.h"


class drvTDS: public drvTek {
public:
    drvTDS(const char* port, const char* udp);
    virtual ~drvTDS(){}
  
protected:
    virtual int _parseWfPreamble(const char* buf, int*, int*, double*, double*, double*);

private:
    void _initializeParams();
};

#endif

