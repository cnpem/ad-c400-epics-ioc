#!../../bin/linux-x86_64/c400

#- You may have to change c400 to something else
#- everywhere it appears in this file

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/c400.dbd"
c400_registerRecordDeviceDriver pdbbase
drvAsynIPPortConfigure("PS", "10.31.54.44:4003")
c400CreateDriver("testAPD", "PS")

# Turn on asyn trace
#asynSetTraceMask("testAPD",0,3)
#asynSetTraceIOMask("testAPD",0,1)

## Load record instances
dbLoadRecords("db/c400.db","P=EMA:, R=B:c40001:,PORT=testAPD,ADDR=0,TIMEOUT=5")
# asyn record for troubleshooting
dbLoadRecords("$(ASYN)/db/asynRecord.db","P=EMA:B:,R=asyn_1,PORT=PS,ADDR=0,OMAX=256,IMAX=256")

cd "${TOP}/iocBoot/${IOC}"
iocInit

