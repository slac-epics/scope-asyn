TOP=../..

include $(TOP)/configure/CONFIG

USR_CXXFLAGS_Linux = -std=c++0x

#LIBRARY_IOC += scope
LIBRARY_IOC += tds3x
LIBRARY_IOC += mdo3x
LIBRARY_IOC += ds1x
LIBRARY_IOC += ds6x

DBD += drvTDS.dbd
DBD += drvMDO.dbd
DBD += drvDS1x.dbd
DBD += drvDS6x.dbd

#scope_SRCS += drvScope.cpp

tds3x_SRCS += drvScope.cpp
tds3x_SRCS += drvTek.cpp
tds3x_SRCS += drvTDS.cpp

mdo3x_SRCS += drvScope.cpp
mdo3x_SRCS += drvTek.cpp
mdo3x_SRCS += drvMDO.cpp

ds1x_SRCS += drvScope.cpp
ds1x_SRCS  += drvDS1x.cpp

ds6x_SRCS += drvScope.cpp
ds6x_SRCS  += drvDS6x.cpp

LIB_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

