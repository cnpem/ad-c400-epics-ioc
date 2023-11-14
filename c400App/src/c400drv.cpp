/*
    - Based on the IOC asynDrive c400 by Hugo Henrique Valim.

    Developer: Guilherme Rodrigues de Lima
    Email: guilherme.lima@lnls.br
    Company: CNPEM/Sirius - Brazil
    Date: 05/10/2023
*/

#include "c400drv.h"
using namespace std;

void update_counts(void *drvPvt);
static const char *driverName = "c400driver";

c400drv::c400drv(const char *portName, const char *ip, int maxBuffers, size_t maxMemory, int priority, int stackSize)
   : ADDriver(portName,
                    1, /* maxAddr */
                    0,
                    maxBuffers,
                    maxMemory,
                    asynEnumMask | asynFloat64ArrayMask,
                    asynEnumMask | asynFloat64ArrayMask, /* No interfaces beyond those set in ADDriver.cpp */
                    ASYN_CANBLOCK, /* asynFlags.  This driver does not block and it is not multi-device, so flag is 0 */
                    1, /* Autoconnect */
                    priority,
                    stackSize) // thread priority and stack size (0=default)
{
    asynStatus status;
    const char *functionName = "c400drv";

    createParam(P_DACString1, asynParamFloat64, &P_DAC1);
    createParam(P_DACString2, asynParamFloat64, &P_DAC2);
    createParam(P_DACString3, asynParamFloat64, &P_DAC3);
    createParam(P_DACString4, asynParamFloat64, &P_DAC4);
    createParam(P_DEADString, asynParamFloat64, &P_DEAD);
    createParam(P_DHIString1, asynParamFloat64, &P_DHI1);
    createParam(P_DHIString2, asynParamFloat64, &P_DHI2);
    createParam(P_DHIString3, asynParamFloat64, &P_DHI3);
    createParam(P_DHIString4, asynParamFloat64, &P_DHI4);
    createParam(P_DLOString1, asynParamFloat64, &P_DLO1);
    createParam(P_DLOString2, asynParamFloat64, &P_DLO2);
    createParam(P_DLOString3, asynParamFloat64, &P_DLO3);
    createParam(P_DLOString4, asynParamFloat64, &P_DLO4);
    createParam(P_HIVO_VOLTSString1, asynParamFloat64, &P_HIVO_VOLTS1);
    createParam(P_HIVO_VOLTSString2, asynParamFloat64, &P_HIVO_VOLTS2);
    createParam(P_HIVO_VOLTSString3, asynParamFloat64, &P_HIVO_VOLTS3);
    createParam(P_HIVO_VOLTSString4, asynParamFloat64, &P_HIVO_VOLTS4);
    createParam(P_HIVO_ENABLEString1, asynParamInt32, &P_HIVO_ENABLE1);
    createParam(P_HIVO_ENABLEString2, asynParamInt32, &P_HIVO_ENABLE2);
    createParam(P_HIVO_ENABLEString3, asynParamInt32, &P_HIVO_ENABLE3);
    createParam(P_HIVO_ENABLEString4, asynParamInt32, &P_HIVO_ENABLE4);
    createParam(P_POLARITYString1, asynParamInt32, &P_POLARITY1);
    createParam(P_POLARITYString2, asynParamInt32, &P_POLARITY2);
    createParam(P_POLARITYString3, asynParamInt32, &P_POLARITY3);
    createParam(P_POLARITYString4, asynParamInt32, &P_POLARITY4);
    createParam(P_PULSER_PeriodString, asynParamFloat64, &P_PULSER_Period);
    createParam(P_PULSER_WidthString, asynParamFloat64, &P_PULSER_Width);
    createParam(P_BURSTString, asynParamFloat64, &P_BURST);
    createParam(P_ENCODERString, asynParamFloat64, &P_ENCODER);
    createParam(P_TRIGGER_POLARITYString, asynParamInt32, &P_TRIGGER_POLARITY);
    createParam(P_TRIGGER_STARTString, asynParamInt32, &P_TRIGGER_START);
    createParam(P_TRIGGER_STOPString, asynParamInt32, &P_TRIGGER_STOP);
    createParam(P_TRIGGER_PAUSEString, asynParamInt32, &P_TRIGGER_PAUSE);
    createParam(P_SYSTEM_IPMODEString, asynParamInt32, &P_SYSTEM_IPMODE);

    status = pasynOctetSyncIO->connect(ip, 0, &pasynUserEcho, NULL);
    pasynOctetSyncIO->setInputEos(pasynUserEcho,  "\r\n", strlen("\r\n"));
    pasynOctetSyncIO->setOutputEos(pasynUserEcho, "\n",   strlen("\n"));

    if (status) {
        setStringParam(ADStatusMessage, "Not connected to c400");
        asynPrint(pasynUserEcho, ASYN_TRACE_ERROR,
                "%s:%s Error: Not connected to c400: status=%d\n",
                driverName, functionName, status);
    }

    //Abort any counting while IOC is booting
    status = send_to_equipment(C400_MSG_ABORT_SET);

    // Set default parameters (1 point and 4 channels)
    dims[0] = 1;
    dims[1] = 4;
    setIntegerParam(NDArraySizeX, 1);
    setIntegerParam(NDArraySizeY, 4);
    setStringParam(ADStatusMessage, "Connected to c400");

    status = (asynStatus) callParamCallbacks();

    status = (asynStatus)(epicsThreadCreate("c400countTask",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)::update_counts,
                          this) == NULL);
    if (status) {
        setStringParam(ADStatusMessage, "epicsThreadCreate failure");
        asynPrint(pasynUserEcho, ASYN_TRACE_ERROR,
                "%s:%s Error: epicsThreadCreate failure: status=%d\n",
                driverName, functionName, status);
        abort();
    }
}

//------------ asynPortDriver extended method ------------

void update_counts(void *drvPvt)
{
    c400drv *pPvt = (c400drv *)drvPvt;

    pPvt->update_counts();
}

void c400drv::update_counts(){
    asynStatus status = asynSuccess;
    const char* functionName = "update_counts";
    int acquire;
    int imageMode;
    int bufferSize;

    while (true){
        getIntegerParam(ADAcquire, &acquire);
        getIntegerParam(ADImageMode, &imageMode);
        getIntegerParam(ADNumImages, &bufferSize);

        if ((acquire && imageMode == 0) || (acquire && imageMode == 3)) {
            status = get_counts();
        }
        else if (acquire && imageMode == 1) {
            if (bufferSize > 0) {
                status = get_counts();
            }
            else {
                setIntegerParam (ADAcquire, 0);
                callParamCallbacks();
                asynPrint(pasynUserEcho, ASYN_TRACE_ERROR,
                "%s:%s: ImageMode requires a number of images\n", 
                driverName, functionName);
            }
        }
        else if (acquire && imageMode == 2){
            if (bufferSize > 0 && bufferSize < buffer_array_size) {
                status = get_buffer();
            }
            else {
                setIntegerParam (ADAcquire, 0);
                callParamCallbacks();
                asynPrint(pasynUserEcho, ASYN_TRACE_ERROR,
                "%s:%s: ImageMode requires a number of images\n", 
                driverName, functionName);
            }
        }
    }
}

asynStatus c400drv::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName;
    const char* functionName = "writeInt32";
    double result;
    int res;
    int is_counting;
    int bufferSize;

    /* Set the parameter in the parameter library. */
    status = setIntegerParam(function, value);

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    if (function == ADAcquire){
        setIntegerParam (ADAcquire, value);
    }
    else if (function == ADImageMode){
        if (value == 2) {
            int bufferSize;
            getIntegerParam(ADNumImages, &bufferSize);
            status = set_to_equipment(C400_MSG_BUFFER_SET, C400_MSG_BUFFER_ASK, 0, bufferSize, result);
        }
        else {
            status = set_to_equipment(C400_MSG_BUFFER_SET, C400_MSG_BUFFER_ASK, 0, 0, result);
        }
        setIntegerParam(ADImageMode, value);
    }

    else if (function == ADNumImages){
        int imageMode;
        getIntegerParam(ADImageMode, &imageMode);
        if (imageMode == 2){
            if (value < buffer_array_size) {
                status = set_to_equipment(C400_MSG_BUFFER_SET, C400_MSG_BUFFER_ASK, 0, value, result);
            }
            else {
                asynPrint(pasynUserEcho, ASYN_TRACE_ERROR,
                "%s:%s: The number of images cannot exceed the equipment buffer (maximum: 65536)\n", 
                driverName, functionName);
            }
        }
        else {
            status = set_to_equipment(C400_MSG_BUFFER_SET, C400_MSG_BUFFER_ASK, 0, 0, result);
        }
        setIntegerParam(ADNumImages, value);
    }

    else if (function == P_HIVO_ENABLE1) {
        status = set_4_channels_to_equipment(C400_MSG_HIVO_ENABLE_SET, C400_MSG_HIVO_ENABLE_ASK, 
                                P_HIVO_ENABLE1, P_HIVO_ENABLE2, P_HIVO_ENABLE3, P_HIVO_ENABLE4, 0, 1, result);
        setIntegerParam (P_HIVO_ENABLE1,      result);
    }
    else if (function == P_HIVO_ENABLE2){
        status = set_4_channels_to_equipment(C400_MSG_HIVO_ENABLE_SET, C400_MSG_HIVO_ENABLE_ASK, 
                                P_HIVO_ENABLE1, P_HIVO_ENABLE2, P_HIVO_ENABLE3, P_HIVO_ENABLE4, 0, 2, result);
        setIntegerParam (P_HIVO_ENABLE2,      result);
    }
    else if (function == P_HIVO_ENABLE3){
        status = set_4_channels_to_equipment(C400_MSG_HIVO_ENABLE_SET, C400_MSG_HIVO_ENABLE_ASK, 
                                P_HIVO_ENABLE1, P_HIVO_ENABLE2, P_HIVO_ENABLE3, P_HIVO_ENABLE4, 0, 3, result);
        setIntegerParam (P_HIVO_ENABLE3,      result);
    }
    else if (function == P_HIVO_ENABLE4){
        status = set_4_channels_to_equipment(C400_MSG_HIVO_ENABLE_SET, C400_MSG_HIVO_ENABLE_ASK, 
                                P_HIVO_ENABLE1, P_HIVO_ENABLE2, P_HIVO_ENABLE3, P_HIVO_ENABLE4, 0, 4, result);
        setIntegerParam (P_HIVO_ENABLE4,      result);
    }
    else if (function == P_POLARITY1) {
        status = set_4_channels_to_equipment(C400_MSG_POLARITY_SET, C400_MSG_POLARITY_ASK, 
                                P_POLARITY1, P_POLARITY2, P_POLARITY3, P_POLARITY4, 0, 1, result);
        setIntegerParam (P_POLARITY1,      result);
    }
    else if (function == P_POLARITY2){
        status = set_4_channels_to_equipment(C400_MSG_POLARITY_SET, C400_MSG_POLARITY_ASK, 
                                P_POLARITY1, P_POLARITY2, P_POLARITY3, P_POLARITY4, 0, 2, result);
        setIntegerParam (P_POLARITY2,      result);
    }
    else if (function == P_POLARITY3){
        status = set_4_channels_to_equipment(C400_MSG_POLARITY_SET, C400_MSG_POLARITY_ASK, 
                                P_POLARITY1, P_POLARITY2, P_POLARITY3, P_POLARITY4, 0, 3, result);
        setIntegerParam (P_POLARITY3,      result);
    }
    else if (function == P_POLARITY4){
        status = set_4_channels_to_equipment(C400_MSG_POLARITY_SET, C400_MSG_POLARITY_ASK, 
                                P_POLARITY1, P_POLARITY2, P_POLARITY3, P_POLARITY4, 0, 4, result);
        setIntegerParam (P_POLARITY4,      result);
    }
    else if (function == ADTriggerMode){
        status = set_to_equipment(C400_MSG_TRIGGER_MODE_SET, C400_MSG_TRIGGER_MODE_ASK, 0, value, result);
        setIntegerParam (ADTriggerMode,      result);
    }
    else if (function == P_SYSTEM_IPMODE){
        status = set_to_equipment(C400_MSG_SYSTEM_IPMODE_SET, C400_MSG_SYSTEM_IPMODE_ASK, 0, value, result);
        setIntegerParam (P_SYSTEM_IPMODE,      result);
    }
    else if (function == P_TRIGGER_POLARITY){
        status = set_to_equipment(C400_MSG_TRIGGER_POLARITY_SET, C400_MSG_TRIGGER_POLARITY_ASK, 0, value, result);
        setIntegerParam (P_TRIGGER_POLARITY,      result);
    }
    else if (function == P_TRIGGER_START){
        status = set_to_equipment(C400_MSG_TRIGGER_START_SET, C400_MSG_TRIGGER_START_ASK, 0, value, result);
        setIntegerParam (P_TRIGGER_START,      result);
    }
    else if (function == P_TRIGGER_STOP){
        status = set_to_equipment(C400_MSG_TRIGGER_STOP_SET, C400_MSG_TRIGGER_STOP_ASK, 0, value, result);
        setIntegerParam (P_TRIGGER_STOP,      result);
    }
    else if (function == P_TRIGGER_PAUSE){
        status = set_to_equipment(C400_MSG_TRIGGER_PAUSE_SET, C400_MSG_TRIGGER_PAUSE_ASK, 0, value, result);
        setIntegerParam (P_TRIGGER_PAUSE,      result);
    }

    /* Do callbacks so higher layers see any changes */

    status = (asynStatus) callParamCallbacks();

    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s:%s Error: status=%d function=%d, value=%d\n",
                driverName, functionName, status, function, value);
    }
    else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, name=%s, value=%d\n",
              driverName, functionName, function, paramName, value);
    }
    return status;
}

asynStatus c400drv::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName;
    const char* functionName = "writeFloat64";
    double result;

    /* Set the parameter in the parameter library. */
    status = setDoubleParam(function, value);

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    if (function == P_DAC1) {
        status = set_to_equipment(C400_MSG_DAC_SET, C400_MSG_DAC_ASK, 1, value, result);
        setDoubleParam (P_DAC1,      result);
    }
    else if (function == P_DAC2){
        status = set_to_equipment(C400_MSG_DAC_SET, C400_MSG_DAC_ASK, 2, value, result);
        setDoubleParam (P_DAC2,      result);
    }
    else if (function == P_DAC3){
        status = set_to_equipment(C400_MSG_DAC_SET, C400_MSG_DAC_ASK, 3, value, result);
        setDoubleParam (P_DAC3,      result);
    }
    else if (function == P_DAC4){
        status = set_to_equipment(C400_MSG_DAC_SET, C400_MSG_DAC_ASK, 4, value, result);
        setDoubleParam (P_DAC4,      result);
    }
    else if (function == P_DEAD){
        status = set_to_equipment(C400_MSG_DEAD_SET, C400_MSG_DEAD_ASK, 0, value, result);
        setDoubleParam (P_DEAD,      value);
    }
    else if (function == P_BURST){
        status = set_to_equipment(C400_MSG_BURST_SET, C400_MSG_BURST_ASK, 0, value, result);
        setDoubleParam (P_BURST,      result);
    }
    else if (function == ADAcquireTime){
        status = set_to_equipment(C400_MSG_PERIOD_SET, C400_MSG_PERIOD_ASK, 0, value, result);
        setDoubleParam (ADAcquireTime,      result);
    }
    else if (function == P_DHI1) {
        status = set_4_channels_to_equipment(C400_MSG_DHI_SET, C400_MSG_DHI_ASK, 
                                P_DHI1, P_DHI2, P_DHI3, P_DHI4, 1, 1, result);
        setDoubleParam (P_DHI1,      result);
    }
    else if (function == P_DHI2){
        status = set_4_channels_to_equipment(C400_MSG_DHI_SET, C400_MSG_DHI_ASK, 
                                P_DHI1, P_DHI2, P_DHI3, P_DHI4, 1, 2, result);
        setDoubleParam (P_DHI2,      result);
    }
    else if (function == P_DHI3){
        status = set_4_channels_to_equipment(C400_MSG_DHI_SET, C400_MSG_DHI_ASK, 
                                P_DHI1, P_DHI2, P_DHI3, P_DHI4, 1, 3, result);
        setDoubleParam (P_DHI3,      result);
    }
    else if (function == P_DHI4){
        status = set_4_channels_to_equipment(C400_MSG_DHI_SET, C400_MSG_DHI_ASK, 
                                P_DHI1, P_DHI2, P_DHI3, P_DHI4, 1, 4, result);
        setDoubleParam (P_DHI4,      result);
    }
    else if (function == P_DLO1) {
        status = set_4_channels_to_equipment(C400_MSG_DLO_SET, C400_MSG_DLO_ASK, 
                                P_DLO1, P_DLO2, P_DLO3, P_DLO4, 1, 1, result);
        setDoubleParam (P_DLO1,      result);
    }
    else if (function == P_DLO2){
        status = set_4_channels_to_equipment(C400_MSG_DLO_SET, C400_MSG_DLO_ASK, 
                                P_DLO1, P_DLO2, P_DLO3, P_DLO4, 1, 2, result);
        setDoubleParam (P_DLO2,      result);
    }
    else if (function == P_DLO3){
        status = set_4_channels_to_equipment(C400_MSG_DLO_SET, C400_MSG_DLO_ASK, 
                                P_DLO1, P_DLO2, P_DLO3, P_DLO4, 1, 3, result);
        setDoubleParam (P_DLO3,      result);
    }
    else if (function == P_DLO4){
        status = set_4_channels_to_equipment(C400_MSG_DLO_SET, C400_MSG_DLO_ASK, 
                                P_DLO1, P_DLO2, P_DLO3, P_DLO4, 1, 4, result);
        setDoubleParam (P_DLO4,      result);
    }
    else if (function == P_HIVO_VOLTS1) {
        status = set_4_channels_to_equipment(C400_MSG_HIVO_VOLTS_SET, C400_MSG_HIVO_VOLTS_ASK, 
                                P_HIVO_VOLTS1, P_HIVO_VOLTS2, P_HIVO_VOLTS3, P_HIVO_VOLTS4, 1, 1, result);
        setDoubleParam (P_HIVO_VOLTS1,      result);
    }
    else if (function == P_HIVO_VOLTS2){
        status = set_4_channels_to_equipment(C400_MSG_HIVO_VOLTS_SET, C400_MSG_HIVO_VOLTS_ASK, 
                                P_HIVO_VOLTS1, P_HIVO_VOLTS2, P_HIVO_VOLTS3, P_HIVO_VOLTS4, 1, 2, result);
        setDoubleParam (P_HIVO_VOLTS2,      result);
    }
    else if (function == P_HIVO_VOLTS3){
        status = set_4_channels_to_equipment(C400_MSG_HIVO_VOLTS_SET, C400_MSG_HIVO_VOLTS_ASK, 
                                P_HIVO_VOLTS1, P_HIVO_VOLTS2, P_HIVO_VOLTS3, P_HIVO_VOLTS4, 1, 3, result);
        setDoubleParam (P_HIVO_VOLTS3,      result);
    }
    else if (function == P_HIVO_VOLTS4){
        status = set_4_channels_to_equipment(C400_MSG_HIVO_VOLTS_SET, C400_MSG_HIVO_VOLTS_ASK, 
                                P_HIVO_VOLTS1, P_HIVO_VOLTS2, P_HIVO_VOLTS3, P_HIVO_VOLTS4, 1, 4, result);
        setDoubleParam (P_HIVO_VOLTS4,      result);
    }

    else if (function == P_PULSER_Period) {
        status = set_2_vals_to_equipment(C400_MSG_PULSER_SET, C400_MSG_PULSER_ASK, 
                                P_PULSER_Period, P_PULSER_Width, 1, 1, result);
        setDoubleParam (P_PULSER_Period,      result);
    }
    else if (function == P_PULSER_Width) {
        status = set_2_vals_to_equipment(C400_MSG_PULSER_SET, C400_MSG_PULSER_ASK, 
                                P_PULSER_Period, P_PULSER_Width, 1, 2, result);
        setDoubleParam (P_PULSER_Width,      result);
    }

    status = (asynStatus) callParamCallbacks();

    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s:%s Error: status=%d function=%d, value=%f\n",
                driverName, functionName, status, function, value);
    }
    else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, name=%s, value=%f\n",
              driverName, functionName, function, paramName, value);
    }
    return status;
}

asynStatus c400drv::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName;
    const char* functionName = "readInt32";
    double res;
    
    getParamName(function, &paramName);

    if (function == P_HIVO_ENABLE1) {
        status = get_to_equipment(C400_MSG_HIVO_ENABLE_ASK, 1, res);
        setIntegerParam (P_HIVO_ENABLE1,          res);
    }
    else if (function == P_HIVO_ENABLE2) {
        status = get_to_equipment(C400_MSG_HIVO_ENABLE_ASK, 2, res);
        setIntegerParam (P_HIVO_ENABLE2,          res);
    }
    else if (function == P_HIVO_ENABLE3) {
        status = get_to_equipment(C400_MSG_HIVO_ENABLE_ASK, 3, res);
        setIntegerParam (P_HIVO_ENABLE3,          res);
    }
    else if (function == P_HIVO_ENABLE4) {
        status = get_to_equipment(C400_MSG_HIVO_ENABLE_ASK, 4, res);
        setIntegerParam (P_HIVO_ENABLE4,          res);
    }
    else if (function == P_POLARITY1) {
        status = get_to_equipment(C400_MSG_POLARITY_ASK, 1, res);
        setIntegerParam (P_POLARITY1,          res);
    }
    else if (function == P_POLARITY2) {
        status = get_to_equipment(C400_MSG_POLARITY_ASK, 2, res);
        setIntegerParam (P_POLARITY2,          res);
    }
    else if (function == P_POLARITY3) {
        status = get_to_equipment(C400_MSG_POLARITY_ASK, 3, res);
        setIntegerParam (P_POLARITY3,          res);
    }
    else if (function == P_POLARITY4) {
        status = get_to_equipment(C400_MSG_POLARITY_ASK, 4, res);
        setIntegerParam (P_POLARITY4,          res);
    }
    else if (function == P_SYSTEM_IPMODE){
        status = get_to_equipment(C400_MSG_SYSTEM_IPMODE_ASK, 1, res);
        setIntegerParam (P_SYSTEM_IPMODE,      res);
    }
    else if (function == ADNumImages){
        status = get_to_equipment(C400_MSG_BUFFER_ASK, 1, res);
        setIntegerParam (ADNumImages,          res);
    }
    else if (function == ADTriggerMode){
        status = get_to_equipment(C400_MSG_TRIGGER_MODE_ASK, 1, res);
        setIntegerParam (ADTriggerMode,      res);
    }

    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s:%s Error: status=%d function=%d, value=%d\n",
                driverName, functionName, status, function, value);
    }
    else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, name=%s, value=%d\n",
              driverName, functionName, function, paramName, value);
    }
    return status;
}

asynStatus c400drv::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName;
    const char* functionName = "readFloat64";
    double res;

    getParamName(function, &paramName);

    if (function == P_DAC1) {
        status = get_to_equipment(C400_MSG_DAC_ASK, 1, res);
        setDoubleParam (P_DAC1,          res);
    }
    else if (function == P_DAC2){
        status = get_to_equipment(C400_MSG_DAC_ASK, 2, res);
        setDoubleParam (P_DAC2,          res);
    }
    else if (function == P_DAC3){
        status = get_to_equipment(C400_MSG_DAC_ASK, 3, res);
        setDoubleParam (P_DAC3,          res);
    }
    else if (function == P_DAC4){
        status = get_to_equipment(C400_MSG_DAC_ASK, 4, res);
        setDoubleParam (P_DAC4,          res);
    }
    else if (function == P_DEAD){
        status = get_to_equipment(C400_MSG_DEAD_ASK, 1, res);
        setDoubleParam (P_DEAD,          res);
    }
    else if (function == P_DHI1) {
        status = get_to_equipment(C400_MSG_DHI_ASK, 1, res);
        setDoubleParam (P_DHI1,          res);
    }
    else if (function == P_DHI2){
        status = get_to_equipment(C400_MSG_DHI_ASK, 2, res);
        setDoubleParam (P_DHI2,          res);
    }
    else if (function == P_DHI3){
        status = get_to_equipment(C400_MSG_DHI_ASK, 3, res);
        setDoubleParam (P_DHI3,          res);
    }
    else if (function == P_DHI4){
        status = get_to_equipment(C400_MSG_DHI_ASK, 4, res);
        setDoubleParam (P_DHI4,          res);
    }
    else if (function == P_DLO1) {
        status = get_to_equipment(C400_MSG_DLO_ASK, 1, res);
        setDoubleParam (P_DLO1,          res);
    }
    else if (function == P_DLO2){
        status = get_to_equipment(C400_MSG_DLO_ASK, 2, res);
        setDoubleParam (P_DLO2,          res);
    }
    else if (function == P_DLO3){
        status = get_to_equipment(C400_MSG_DLO_ASK, 3, res);
        setDoubleParam (P_DLO3,          res);
    }
    else if (function == P_DLO4){
        status = get_to_equipment(C400_MSG_DLO_ASK, 4, res);
        setDoubleParam (P_DLO4,          res);
    }
    else if (function == P_HIVO_VOLTS1) {
        status = get_to_equipment(C400_MSG_HIVO_VOLTS_ASK, 1, res);
        setDoubleParam (P_HIVO_VOLTS1,          res);
    }
    else if (function == P_HIVO_VOLTS2){
        status = get_to_equipment(C400_MSG_HIVO_VOLTS_ASK, 2, res);
        setDoubleParam (P_HIVO_VOLTS2,          res);
    }
    else if (function == P_HIVO_VOLTS3){
        status = get_to_equipment(C400_MSG_HIVO_VOLTS_ASK, 3, res);
        setDoubleParam (P_HIVO_VOLTS3,          res);
    }
    else if (function == P_HIVO_VOLTS4){
        status = get_to_equipment(C400_MSG_HIVO_VOLTS_ASK, 4, res);
        setDoubleParam (P_HIVO_VOLTS4,          res);
    }
    else if (function == ADAcquireTime){
        status = get_to_equipment(C400_MSG_PERIOD_ASK, 1, res);
        setDoubleParam (ADAcquireTime,          res);
    }
    else if (function == P_PULSER_Period) {
        status = get_to_equipment(C400_MSG_PULSER_ASK, 1, res);
        setDoubleParam (P_PULSER_Period,          res);
    }
    else if (function == P_PULSER_Width) {
        status = get_to_equipment(C400_MSG_PULSER_ASK, 2, res);
        setDoubleParam (P_PULSER_Width,          res);
    }
    else if (function == P_BURST){
        status = get_to_equipment(C400_MSG_BURST_ASK, 1, res);
        setDoubleParam (P_BURST,          res);
    }

    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s:%s Error: status=%d function=%d, value=%f\n",
                driverName, functionName, status, function, value);
    }
    else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, name=%s, value=%f\n",
              driverName, functionName, function, paramName, value);
        }
    return status;
}

/** Called when asyn clients call pasynEnum->read().
  * The base class implementation simply prints an error message.
  * @param pasynUser - pasynUser structure that encodes the reason and address.
  * @param strings - Array of string pointers.
  * @param values - Array of values
  * @param severities - Array of severities
  * @param nElements - Size of value array
  * @param nIn - Number of elements actually returned */
asynStatus c400drv::readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], size_t nElements, size_t *nIn)
    {
        const int function = pasynUser->reason;
        const char* functionName = "readEnum";
        asynStatus status = asynSuccess;
        size_t index;

        if (function == ADTriggerMode) {
            for (index = 0; ((index < (size_t)trigger_mode.size()) && (index < nElements)); index++){
                if (strings[index]){
                    free(strings[index]);
                }
                strings[index] = epicsStrDup(trigger_mode[index].c_str());
                values[index] = index;
                severities[index] = 0;
            }
            *nIn = index;
        }
        else if (function == ADImageMode) {
            for (index = 0; ((index < (size_t)image_mode.size()) && (index < nElements)); index++){
                if (strings[index]){
                    free(strings[index]);
                }
                strings[index] = epicsStrDup(image_mode[index].c_str());
                values[index] = index;
                severities[index] = 0;
            }
            *nIn = index;
        }
        else {
            *nIn = 0;
            status = asynError;
        }
        return status;
    }

asynStatus c400drv::send_to_equipment(const char *writeBuffer, string &response)
{
    asynStatus status = asynSuccess;
    const char *functionName = "send_to_equipment";
    size_t nRead, nActual;
    int eomReason;
    char readBuffer[10000];

    status = pasynOctetSyncIO->writeRead(pasynUserEcho, writeBuffer, strlen(writeBuffer), readBuffer,
                                     sizeof(readBuffer), TIMEOUT, &nActual, &nRead, &eomReason);

    response = string(readBuffer);

    if (status) {
        asynPrint(pasynUserEcho, ASYN_TRACE_ERROR,
                "%s:%s Error: status=%d, response=%s\n",
                driverName, functionName, status, response);
    }
    else {
        asynPrint(pasynUserEcho, ASYN_TRACEIO_DRIVER,
              "%s:%s: status=%d, response=%s\n",
              driverName, functionName, status, response);
    }

    //cout << "Status: " << status << endl;
    //cout << "Response: " << readBuffer << endl;

    return status;

}

asynStatus c400drv::send_to_equipment(const char *writeBuffer)
{
    asynStatus status = asynSuccess;
    const char *functionName = "send_to_equipment";
    size_t nRead, nActual;
    int eomReason;   
    char readBuffer[10000];

    status = pasynOctetSyncIO->writeRead(pasynUserEcho, writeBuffer, strlen(writeBuffer), readBuffer,
                                     sizeof(readBuffer), TIMEOUT, &nActual, &nRead, &eomReason);

    if (status) {
        asynPrint(pasynUserEcho, ASYN_TRACE_ERROR,
                "%s:%s Error: status=%d, response=%s\n",
                driverName, functionName, status, string(readBuffer));
    }
    else {
        asynPrint(pasynUserEcho, ASYN_TRACEIO_DRIVER,
              "%s:%s: status=%d, response=%s\n",
              driverName, functionName, status, string(readBuffer));
    }

    //cout << "Status: " << status << endl;
    //cout << "Response: " << readBuffer << endl;

    return status;

}

asynStatus c400drv::receive_to_equipment(string &response)
{
    asynStatus status = asynSuccess;
    const char *functionName = "receive_to_equipment";
    size_t nRead;
    int eomReason;
    char readBuffer[10000];

    status = pasynOctetSyncIO->read(pasynUserEcho, readBuffer,
                                     sizeof(readBuffer), TIMEOUT, &nRead, &eomReason);

    response = string(readBuffer);

    if (status > 1) {
        asynPrint(pasynUserEcho, ASYN_TRACE_ERROR,
                "%s:%s Error: status=%d, response=%s\n",
                driverName, functionName, status, string(readBuffer));
    }
    else {
        asynPrint(pasynUserEcho, ASYN_TRACEIO_DRIVER,
              "%s:%s: status=%d, response=%s\n",
              driverName, functionName, status, string(readBuffer));
    }

    //cout << "Status: " << status << endl;
    //cout << "Response Receiver: " << readBuffer << endl;

    return status;

}

asynStatus c400drv::get_to_equipment(const char *command_ask, int n_element, double &response)
{   
    asynStatus status = asynSuccess;
    string delimiter = ",";
    int pos = 0;
    int index = 0;
    string result_array[15];
    string token;
    string val;

    status = send_to_equipment(command_ask, val);

    if (status==asynSuccess) {

        token = val.substr(0, val.find("\n")+1);
        val = val.substr(token.length(), val.length() - token.length());
        
        result_array[0] = token;
        int array_idx = 1;
        while (array_idx < 12) {
            pos = val.find(delimiter);
            token = val.substr(0, pos);
            result_array[array_idx] = token;
            val.erase(0, pos + delimiter.length());
            if (pos == -1){
                result_array[array_idx] = val; //Append the last val
                break;
            }
            array_idx++;
        }

        if (command_ask==C400_MSG_TRIGGER_START_ASK or command_ask==C400_MSG_TRIGGER_STOP_ASK or command_ask==C400_MSG_TRIGGER_PAUSE_ASK){
            index = distance(trigger_source.begin(),find(trigger_source.begin(), trigger_source.end(), result_array[n_element]));
            response = index;
        }
        else if (command_ask == C400_MSG_TRIGGER_MODE_ASK) {
            index = distance(trigger_mode.begin(),find(trigger_mode.begin(), trigger_mode.end(), result_array[n_element]));
            response = index;
        }
        else if (command_ask == C400_MSG_SYSTEM_IPMODE_ASK) {
            index = distance(system_ipmode.begin(),find(system_ipmode.begin(), system_ipmode.end(), result_array[n_element]));
            response = index;
        }
        else if (command_ask==C400_MSG_POLARITY_ASK){
            index = distance(polarity.begin(),find(polarity.begin(), polarity.end(), result_array[n_element]));
            response = index;
        }
        else {
            response = stof(result_array[n_element]);
        }
        
        return status;
    }
    else {
        return status;
    }
}

asynStatus c400drv::set_to_equipment(const char *command_set, const char *command_ask, int channel, double value, double &response)
{
    // If 0 is passed to channel, them no specific channel is set
    asynStatus status = asynSuccess;
    double res;
    string str_channel;
    string str_val;
    string cmd_msg_send;

    str_channel = to_string(channel);
    str_val = to_string(value);

    if (channel > 0){

        cmd_msg_send =  command_set + str_channel + " " + str_val;
        status = send_to_equipment(cmd_msg_send.c_str());
        status = get_to_equipment(command_ask, channel, response);
    }
    else{
        if (command_set==C400_MSG_TRIGGER_START_SET or command_set==C400_MSG_TRIGGER_STOP_SET or command_set==C400_MSG_TRIGGER_PAUSE_SET){
            str_val = trigger_source[(int)value];
        }
        else if (command_set == C400_MSG_TRIGGER_MODE_SET) {
            str_val = trigger_mode[(int)value];
        }
        else if (command_set == C400_MSG_SYSTEM_IPMODE_SET) {
            str_val = system_ipmode[(int)value];
        }

        cmd_msg_send =  command_set + str_val;
        status = send_to_equipment(cmd_msg_send.c_str());
        status = get_to_equipment(command_ask, 1, response);
    }
    return status;
}

asynStatus c400drv::set_4_channels_to_equipment(const char *command_set, const char *command_ask, int param1, int param2, 
                               int param3, int param4, int is_float, int channel, double &response)
{
        asynStatus status = asynSuccess;
        double val_ch1;
        double val_ch2;
        double val_ch3;
        double val_ch4;
        int val_ch1_int;
        int val_ch2_int;
        int val_ch3_int;
        int val_ch4_int;
        string str_val_ch1;
        string str_val_ch2;
        string str_val_ch3;
        string str_val_ch4;
        string cmd_msg_send;

        if (is_float) {
            getDoubleParam(param1, &val_ch1);
            getDoubleParam(param2, &val_ch2);
            getDoubleParam(param3, &val_ch3);
            getDoubleParam(param4, &val_ch4);

        }
        else {
            getIntegerParam(param1, &val_ch1_int);
            getIntegerParam(param2, &val_ch2_int);
            getIntegerParam(param3, &val_ch3_int);
            getIntegerParam(param4, &val_ch4_int);
            val_ch1 = (float)val_ch1_int;
            val_ch2 = (float)val_ch2_int;
            val_ch3 = (float)val_ch3_int;
            val_ch4 = (float)val_ch4_int;
        }

        if (command_set==C400_MSG_POLARITY_SET){
            str_val_ch1 = polarity[(int)val_ch1];
            str_val_ch2 = polarity[(int)val_ch2];
            str_val_ch3 = polarity[(int)val_ch3];
            str_val_ch4 = polarity[(int)val_ch4];
        }
        else {
            str_val_ch1 = to_string(val_ch1);
            str_val_ch2 = to_string(val_ch2);
            str_val_ch3 = to_string(val_ch3);
            str_val_ch4 = to_string(val_ch4);
        }

        cmd_msg_send = command_set + str_val_ch1 + " " + str_val_ch2 + " " + str_val_ch3 + " " + str_val_ch4 + " ";
        status = send_to_equipment(cmd_msg_send.c_str());
        status = get_to_equipment(command_ask, channel, response);

        return status;
}

asynStatus c400drv::set_2_vals_to_equipment(const char *command_set, const char *command_ask, int param1, int param2, int is_float, int channel, double &response)
{
        asynStatus status = asynSuccess;
        double res;
        double val_ch1;
        double val_ch2;
        int val_ch1_int;
        int val_ch2_int;
        string str_val_ch1;
        string str_val_ch2;
        string cmd_msg_send;
        string cmd_msg_read;

        // Check if the value to feed is a float or integer and handles it
        if (is_float){
            getDoubleParam(param1, &val_ch1);
            getDoubleParam(param2, &val_ch2);
        }

        else{
            getIntegerParam(param1, &val_ch1_int);
            getIntegerParam(param2, &val_ch2_int);
            val_ch1 = (float)val_ch1_int;
            val_ch2 = (float)val_ch2_int;
        }

        str_val_ch1 = to_string(val_ch1);
        str_val_ch2 = to_string(val_ch2);

        cmd_msg_send = command_set + str_val_ch1 + " " + str_val_ch2;
        status = send_to_equipment(cmd_msg_send.c_str());
        status = get_to_equipment(command_ask, channel, res);
        return status;
}

asynStatus c400drv::get_counts() {
    asynStatus status = asynSuccess;

    old_triggercounts = 0;
    accumulate_triggercounts = 0;

    Sample sample;

    string response;
    string old_response = "";
    double encoder;
    int acquire;
    int imageMode;
    int numImages = 0;
    int numImagesCounter = 0;
    int imageCounter = 0;

    getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
    getIntegerParam(ADImageMode, &imageMode);
    getIntegerParam(ADNumImages, &numImages);
    getIntegerParam(ADAcquire, &acquire);
    
    setStringParam(ADStatusMessage, "Acquiring data");
    setIntegerParam(NDArrayCounter, 0);
    setIntegerParam(ADNumImagesCounter, 0);

    // Get data type
    getIntegerParam(NDDataType, (int *) &dataType);

    // Start acquisition
    status = send_to_equipment(C400_MSG_ACQUIRE_SET);
    status = send_to_equipment(C400_MSG_COUNTS_ASK, old_response); //Required to remove the first empty acquisition.
    
    this->lock();
    while(acquire) {

        status = send_to_equipment(C400_MSG_COUNTS_ASK, response);

        if(response != old_response && status == 0) {
            old_response = response;
            //cout << "count: " << response << endl;
            
            // Allocate NDArray memory
            pImage = this->pNDArrayPool->alloc(2, dims, dataType, 0, NULL);

            parse_counts(&sample, response);

            memcpy(pImage->pData, sample.array_count, pImage->dataSize);

            pImage->timeStamp = sample.timestamp;
            pImage->uniqueId = sample.trigger_counts;

            // Set a bit of areadetector image/frame statistics...
            getIntegerParam(NDArrayCounter, &imageCounter);
            getIntegerParam(ADNumImagesCounter, &numImagesCounter);
            imageCounter++;
            numImagesCounter++;
            setIntegerParam(NDArrayCounter, imageCounter);
            setIntegerParam(ADNumImagesCounter, numImagesCounter);

            // Get any attributes that have been defined for this driver
            this->getAttributes(pImage->pAttributeList);
            if (arrayCallbacks){
                // Must release the lock here, or we can get into a deadlock, because we can
                // block on the plugin lock, and the plugin can be calling us
                this->unlock();
                doCallbacksGenericPointer(pImage, NDArrayData, 0);
                this->lock();
            }
            // Free the image buffer
            pImage->release();
            status = get_to_equipment(C400_MSG_ENCODER_ASK, 1, encoder);
            setDoubleParam(P_ENCODER, encoder);

            //check the image mode
            if (imageMode == 0) {
                setIntegerParam(ADAcquire, 0);
            }
            else if (imageMode == 1 && numImagesCounter >= numImages) {
                setIntegerParam(ADAcquire, 0);
            }
        }
        callParamCallbacks();
        getIntegerParam(ADAcquire, &acquire);
    }
    this->unlock();
    status = send_to_equipment(C400_MSG_ABORT_SET);
    setIntegerParam(ADAcquire, 0);
    setStringParam(ADStatusMessage, "Acquisition stopped");
    callParamCallbacks();
    return status;
}

asynStatus c400drv::get_buffer(){
    asynStatus status = asynSuccess;

    old_triggercounts = 0;
    accumulate_triggercounts = 0;

    string first_5_char;
    string fetch_5_char = "FETch";
    string error_5_char = "-401,";

    Sample sample;

    string response; // Placeholder for the retrieved equipmente message
    string line;

    int acquire;
    int acquire_buffer = 1;
    int numImages;
    int numImagesCounter = 0;
    int imageCounter     = 0;
    
    getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
    getIntegerParam(ADAcquire, &acquire);
    getIntegerParam(ADNumImages, &numImages);

    setStringParam(ADStatusMessage, "Acquiring data");
    setIntegerParam(NDArrayCounter, 0);
    setIntegerParam(ADNumImagesCounter, 0);

    string get_full_buffer_str;
    get_full_buffer_str = C400_MSG_COUNTS_ASK + to_string(numImages);

    // Get data type
    getIntegerParam(NDDataType, (int *) &dataType);

    // Start acquisition
    status = send_to_equipment(C400_MSG_ACQUIRE_SET);

    this->lock();
    while (acquire_buffer && acquire){

        //status = pasynOctetSyncIO->flush(pasynUserEcho);
        status = send_to_equipment(get_full_buffer_str.c_str(), response);

        istringstream iss(response);
        getline(iss, line);
        getline(iss, line);

        while (line != "" && acquire && status == 0)  {

            first_5_char = line.substr(0, 5);

            if (strcmp(first_5_char.c_str(), fetch_5_char.c_str()) == 0){
                continue;
            }
            else if (strcmp(first_5_char.c_str(),  error_5_char.c_str()) == 0){
                break;
            }
            else {
                //cout << "count: " << line << endl;

                // Allocate NDArray memory
                pImage = pNDArrayPool->alloc(2, dims, dataType, 0, NULL);

                parse_counts(&sample, line);

                memcpy(pImage->pData, sample.array_count, pImage->dataSize);

                pImage->timeStamp = sample.timestamp;
                pImage->uniqueId = sample.trigger_counts;

                // Set a bit of areadetector image/frame statistics...
                getIntegerParam(NDArrayCounter, &imageCounter);
                getIntegerParam(ADNumImagesCounter, &numImagesCounter);
                imageCounter++;
                numImagesCounter++;
                setIntegerParam(NDArrayCounter, imageCounter);
                setIntegerParam(ADNumImagesCounter, numImagesCounter);

                // Get any attributes that have been defined for this driver
                this->getAttributes(pImage->pAttributeList);
                if (arrayCallbacks){
                    // Must release the lock here, or we can get into a deadlock, because we can
                    // block on the plugin lock, and the plugin can be calling us
                    this->unlock();
                    doCallbacksGenericPointer(pImage, NDArrayData, 0);
                    this->lock();
                }
                //Free the image buffer
                pImage->release();
            }
            callParamCallbacks();
            if (numImagesCounter < numImages){
                status = receive_to_equipment(line);
                if (status) {
                    break;
                }
            }
            else if (numImagesCounter >= numImages) {
                acquire_buffer = 0;
                setIntegerParam(ADAcquire, 0);
                callParamCallbacks();
                break;
            }

        }
        callParamCallbacks();
        getIntegerParam(ADAcquire, &acquire);
    }
    this->unlock();
    status = send_to_equipment(C400_MSG_ABORT_SET);
    setIntegerParam(ADAcquire, 0);
    setStringParam(ADStatusMessage, "Acquisition stopped");
    callParamCallbacks();
    return status;
}

asynStatus c400drv::parse_counts(Sample *sample, string val)
{
    asynStatus status = asynSuccess;
    const char* functionName = "parse_counts";

    string delimiter = ",";
    size_t pos = 0;
    string token;
    string last_char;

    vector<string> result;

    // If necessary, remove the first line ("\n")
    token = val.substr(0, val.find("\n")+1);
    val = val.substr(token.length(), val.length() - token.length());

    // Parse (split) a string using string delimiter
    while ((pos = val.find(delimiter)) != string::npos) {
        token = val.substr(0, pos);
        last_char = token[token.length() - 1];
        if (last_char == "S" or last_char == "V"){
            token = token.substr(0, token.length()-2);
        }
        result.push_back(token);
        val.erase(0, pos + delimiter.length());
    }
    result.push_back(val); //last string

    // Add value in struct Sample
    try {
        sample->integration = stod(result.at(0));
        sample->array_count[0] = stoi(result.at(1));
        sample->array_count[1] = stoi(result.at(2));
        sample->array_count[2] = stoi(result.at(3));
        sample->array_count[3] = stoi(result.at(4));
        sample->timestamp = stod(result.at(5));
        
        //trigger counts of the c400 is an 8-bit register, 
        //therefore, logic was implemented for counts above 255.
        if (stoi(result.at(6)) < old_triggercounts) {
            accumulate_triggercounts = accumulate_triggercounts + 256;
        }
        old_triggercounts = stoi(result.at(6));
        sample->trigger_counts = accumulate_triggercounts + stoi(result.at(6)) + 1;

        sample->low_level[0] = stod(result.at(7));
        sample->low_level[1] = stod(result.at(8));
        sample->low_level[2] = stod(result.at(9));
        sample->low_level[3] = stod(result.at(10));
        sample->overflow = stoi(result.at(11));
    }
    catch(const exception& e) {
        asynPrint(pasynUserEcho, ASYN_TRACE_ERROR,
                "%s:%s Warning: Incomplete equipment response - Exception: %s\n",
                driverName, functionName, e.what());
        status = asynError;
    }

    return status;

}

//--------------------------------------------------------

extern "C" int c400CreateDriver(const char *portName, const char *ip, int maxBuffers, size_t maxMemory, int priority, int stackSize){
    new c400drv(portName, ip, maxBuffers, maxMemory, priority, stackSize);
    return(asynSuccess);
}

static const iocshArg portNameArg = { "Port name", iocshArgString};
static const iocshArg ipArg = { "IP", iocshArgString};
static const iocshArg maxBufferArg = { "Max Buffer", iocshArgString};
static const iocshArg maxMemoryArg = { "Max Memory", iocshArgString};
static const iocshArg priorityArg = {"priority", iocshArgInt};
static const iocshArg stackSizeArg = {"stackSize", iocshArgInt};

static const iocshArg * const createDriverArgs[] = {&portNameArg, &ipArg, &maxBufferArg, &maxMemoryArg, &priorityArg, &stackSizeArg};
static const iocshFuncDef createDriverFuncDef = {"c400CreateDriver", 6, createDriverArgs};

static void createDriverCallFunc(const iocshArgBuf *args){
    c400CreateDriver(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}

void c400drvRegister(void){
    iocshRegister(&createDriverFuncDef,createDriverCallFunc);
}

extern "C" {
    epicsExportRegistrar(c400drvRegister);
}

