TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS BELOW HERE
PROD_NAME = c400

# Build the IOC application
PROD_IOC = c400

DBD += c400.dbd

$(PROD_NAME)_DBD += c400drv.dbd
$(PROD_NAME)_DBD += drvAsynIPPort.dbd

$(PROD_NAME)_DBD += NDFileHDF5.dbd
PROD_LIBS += hdf5
PROD_LIBS += hdf5_hl

$(PROD_NAME)_SRCS += c400drv.cpp
$(PROD_NAME)_SRCS += c400_registerRecordDeviceDriver.cpp

# Build the main IOC entry point where needed
$(PROD_NAME)_SRCS_DEFAULT += c400Main.cpp
$(PROD_NAME)_SRCS_vxWorks += -nil-

$(PROD_NAME)_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(ADCORE)/ADApp/commonDriverMakefile

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD EXTRA GNUMAKE RULES BELOW HERE

