TOP = ..
include $(TOP)/configure/CONFIG
DIRS += baseSrc
DIRS += tds3Src
DIRS += mdo3Src
DIRS += ds6xSrc
DIRS += ds1xSrc
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard *db*))
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard *Db*))
include $(TOP)/configure/RULES_DIRS
