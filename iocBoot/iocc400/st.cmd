#!/home/hugo/SOL/IOCs/c400/bin/linux-x86_64/c400

#- You may have to change c400 to something else
#- everywhere it appears in this file

< /home/hugo/SOL/IOCs/c400/iocBoot/iocc400/envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/c400.dbd"
c400_registerRecordDeviceDriver pdbbase


drvAsynIPPortConfigure("PS", "10.31.54.44:4003")
c400CreateDriver("testAPD", "PS")

## Load record instances
dbLoadRecords("db/c400.db","P=EMA:, R=A:,PORT=testAPD,ADDR=0,TIMEOUT=10")


cd "${TOP}/iocBoot/${IOC}"
iocInit

dbpf EMA:A:set_current 5s