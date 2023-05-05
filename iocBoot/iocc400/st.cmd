#!bin/linux-x86_64/c400

#- You may have to change c400 to something else
#- everywhere it appears in this file

< iocBoot/iocc400/envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/c400.dbd"
c400_registerRecordDeviceDriver pdbbase
#10.31.54.44:4003 EMA
#10.32.74.32:4002 PNR A
#10.32.74.32:4003 PNR B
drvAsynIPPortConfigure("PS", "10.32.74.32:4003")
c400CreateDriver("testAPD", "PS")

## Load record instances
dbLoadRecords("db/c400.db","P=EMA:, R=A:,PORT=testAPD,ADDR=0,TIMEOUT=10")
# asyn record for troubleshooting
# dbLoadRecords("$(ASYN)/db/asynRecord.db","P=EMA:A:,R=asyn_1,PORT=PS,ADDR=0,OMAX=256,IMAX=256")

cd "${TOP}/iocBoot/${IOC}"
iocInit

