TOP = ..
include $(TOP)/configure/CONFIG
DIRS += baseSrc
DIRS += tds3Src
DIRS += ds6xSrc
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard *db*))
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard *Db*))
include $(TOP)/configure/RULES_DIRS
