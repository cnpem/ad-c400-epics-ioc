TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS BELOW HERE

# Build the IOC application
PROD_IOC = c400

DBD += c400.dbd

c400_DBD += base.dbd
c400_DBD += asyn.dbd
c400_DBD += drvAsynIPPort.dbd
c400_DBD += c400drv.dbd

# Compile and add the code to the support library
c400Support_SRCS += c400drv.cpp

# System libraries
c400_LIBS	    += asyn

c400_SRCS += c400drv.cpp
c400_SRCS += c400_registerRecordDeviceDriver.cpp

# Build the main IOC entry point where needed
c400_SRCS_DEFAULT += c400Main.cpp
c400_SRCS_vxWorks += -nil-

# Needed for recsync
# Add support from base/src/vxWorks if needed
#luciole_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

c400_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD EXTRA GNUMAKE RULES BELOW HERE

