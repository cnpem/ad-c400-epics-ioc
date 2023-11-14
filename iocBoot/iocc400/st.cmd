#!../../bin/linux-x86_64/c400

#- You may have to change c400 to something else
#- everywhere it appears in this file

< envPaths

# The max array bytes
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "100000000")

# Prefix P
epicsEnvSet("P","EMA:B:C40001:")
# The port name for the detector
epicsEnvSet("PORT","c400")
# The queue size for all plugins
epicsEnvSet("QSIZE","400") #Number buffer response c400
# The maximum image width;
epicsEnvSet("XSIZE","1")
# The maximum image height;
epicsEnvSet("YSIZE","4")
# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/c400.dbd"
c400_registerRecordDeviceDriver pdbbase

drvAsynIPPortConfigure("IP", "10.31.54.29:4003")

# Load record instances
c400CreateDriver("$(PORT)", "IP", 0, 0)
dbLoadRecords("db/c400.db","P=$(P), R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Create an Image plugin
NDStdArraysConfigure("Image1", $(QSIZE), 0, "$(PORT)", 0, 0)
dbLoadRecords("NDStdArrays.template", "P=$(P),R=image1:,PORT=Image1,NDARRAY_PORT=$(PORT),ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=4")

# Create an HDF5 file saving plugin
NDFileHDF5Configure("FileHDF1", $(QSIZE), 0, "$(PORT)", 0)
dbLoadRecords("NDFileHDF5.template","P=$(P),R=HDF1:,PORT=FileHDF1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")

# Create ROI plugins
NDROIConfigure("ROI1", $(QSIZE), 0, "$(PORT)", 0, 0, 0)
dbLoadRecords("NDROI.template","P=$(P),R=ROI1:,PORT=ROI1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")

# Create ROIStat plugins - Configure four channel
NDROIStatConfigure("ROISTAT1", $(QSIZE), 0, "$(PORT)", 0, 8, 0, 0)
dbLoadRecords("NDROIStat.template",   "P=$(P),R=ROIStat1:  ,PORT=ROISTAT1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NCHANS=4")
dbLoadRecords("NDROIStatN.template",  "P=$(P),R=ROIStat1:1:,PORT=ROISTAT1,ADDR=0,TIMEOUT=1,NCHANS=4")
dbLoadRecords("NDROIStatN.template",  "P=$(P),R=ROIStat1:2:,PORT=ROISTAT1,ADDR=1,TIMEOUT=1,NCHANS=4")
dbLoadRecords("NDROIStatN.template",  "P=$(P),R=ROIStat1:3:,PORT=ROISTAT1,ADDR=2,TIMEOUT=1,NCHANS=4")
dbLoadRecords("NDROIStatN.template",  "P=$(P),R=ROIStat1:4:,PORT=ROISTAT1,ADDR=3,TIMEOUT=1,NCHANS=4")

# asyn record for troubleshooting
#dbLoadRecords("$(ASYN)/db/asynRecord.db","P=$(P),R=asyn_1,PORT=$(PORT),ADDR=0,OMAX=256,IMAX=256")

# Turn on asyn trace
#asynSetTraceMask("$(PORT)",0,0x21)
#asynSetTraceIOMask("$(PORT)",0,1)

cd "${TOP}/iocBoot/${IOC}"
iocInit

# Configure Callbacks
dbpf $(P)cam1:ArrayCallbacks 1
dbpf $(P)image1:EnableCallbacks 1
dbpf $(P)HDF1:EnableCallbacks 1
dbpf $(P)ROIStat1:EnableCallbacks 1

# Configure Drive
dbpf $(P)cam1:DataType 4

# Configure image1
dbpf $(P)image1:DataType 4

# Configure ROIStat1:1 - Channel 1
dbpf $(P)ROIStat1:1:Use 1
dbpf $(P)ROIStat1:1:MinX 0
dbpf $(P)ROIStat1:1:MinY 0
dbpf $(P)ROIStat1:1:SizeX 1
dbpf $(P)ROIStat1:1:SizeY 1
#dbgf $(P)ROIStat1:1:Net_RBV

# Configure ROIStat1:2 - Channel 2
dbpf $(P)ROIStat1:2:Use 1
dbpf $(P)ROIStat1:2:MinX 0
dbpf $(P)ROIStat1:2:MinY 1
dbpf $(P)ROIStat1:2:SizeX 1
dbpf $(P)ROIStat1:2:SizeY 1
#dbgf $(P)ROIStat1:2:Net_RBV

# Configure ROIStat1:3 - Channel 3
dbpf $(P)ROIStat1:3:Use 1
dbpf $(P)ROIStat1:3:MinX 0
dbpf $(P)ROIStat1:3:MinY 2
dbpf $(P)ROIStat1:3:SizeX 1
dbpf $(P)ROIStat1:3:SizeY 1
#dbgf $(P)ROIStat1:3:Net_RBV

# Configure ROIStat1:4 - Channel 4
dbpf $(P)ROIStat1:4:Use 1
dbpf $(P)ROIStat1:4:MinX 0
dbpf $(P)ROIStat1:4:MinY 3
dbpf $(P)ROIStat1:4:SizeX 1
dbpf $(P)ROIStat1:4:SizeY 1
#dbgf $(P)ROIStat1:4:Net_RBV

# Configure HDF5
dbpf $(P)HDF1:DataType 4
dbpf $(P)HDF1:FileName "test"
dbpf $(P)HDF1:FilePath "/var/"
dbpf $(P)HDF1:FileTemplate "%s%s%d.hdf5"
dbpf $(P)HDF1:AutoIncrement 1
dbpf $(P)HDF1:LazyOpen 1
dbpf $(P)HDF1:FileWriteMode "Stream"

#dbpf $(P)cam1:AcquireTime 0.1
#dbpf $(P)cam1:ImageMode 2
#dbpf $(P)cam1:NumImages 600
#dbpf $(P)HDF1:NumCapture 600
#dbpf $(P)HDF1:Capture 1
#dbpf $(P)cam1:Acquire 1



