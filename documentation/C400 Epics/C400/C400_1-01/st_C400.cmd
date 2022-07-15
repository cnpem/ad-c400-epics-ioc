#!../../bin/linux-x86/stream
# DSoFt Solutions Ltd v1.0 30/03/2010

## You may have to change stream to something else
## everywhere it appears in this file

< envPaths

cd ${TOP}

## Register all support components
dbLoadDatabase "dbd/stream.dbd"
stream_registerRecordDeviceDriver pdbbase

## Load record instances
#dbLoadRecords("db/xxx.db","user=steveHost")

# Initialise test
epicsEnvSet "STREAM_PROTOCOL_PATH", "."
###drvAsynIPPortConfigure "terminal", "localhost:40000"
drvAsynSerialPortConfigure "COM1", "/dev/ttyS0"
asynOctetSetInputEos "COM1",0,"\r\n"
asynOctetSetOutputEos "COM1",0,"\r\n"
asynSetOption ("COM1", 0, "baud", "115200")
asynSetOption ("COM1", 0, "bits", "8")
asynSetOption ("COM1", 0, "parity", "none")
asynSetOption ("COM1", 0, "stop", "1")
asynSetOption ("COM1", 0, "clocal", "Y")
asynSetOption ("COM1", 0, "crtscts", "N")
cd ${TOP}/iocBoot/${IOC}
dbLoadTemplate "C400.substitutions"

cd ${TOP}/iocBoot/${IOC}
iocInit

## Start any sequence programs
#seq sncxxx,"user=steveHost"
