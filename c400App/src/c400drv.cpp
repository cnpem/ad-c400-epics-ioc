#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <sstream>
#include <vector>
#include <algorithm>

using namespace std;

//EPICS's includes
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <iocsh.h>

//AsynPortDriver's includes
// #include <asynPortDriver.h>
#include "asynOctetSyncIO.h"

//c400's includes
#include "c400drv.h"
#include <epicsExport.h>

void update_counts(void *drvPvt);
static const char *driverName = "c400driver";

c400drv::c400drv(const char *portName, char *ip)
   : asynPortDriver(portName,
                    1, /* maxAddr */
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynEnumMask | asynDrvUserMask, /* Interface mask */
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynEnumMask,  /* Interrupt mask */
                    ASYN_CANBLOCK, /* asynFlags.  This driver does not block and it is not multi-device, so flag is 0 */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0) /* Default stack size*/
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
    createParam(P_PERIODString, asynParamFloat64, &P_PERIOD);
    createParam(P_POLARITYString1, asynParamInt32, &P_POLARITY1);
    createParam(P_POLARITYString2, asynParamInt32, &P_POLARITY2);
    createParam(P_POLARITYString3, asynParamInt32, &P_POLARITY3);
    createParam(P_POLARITYString4, asynParamInt32, &P_POLARITY4);
    createParam(P_PULSER_PeriodString, asynParamFloat64, &P_PULSER_Period);
    createParam(P_PULSER_WidthString, asynParamFloat64, &P_PULSER_Width);
    createParam(P_ACQUIREString, asynParamInt32, &P_ACQUIRE);
    createParam(P_BUFFERString, asynParamInt32, &P_BUFFER);
    createParam(P_BURSTString, asynParamFloat64, &P_BURST);
    createParam(P_COUNT1String, asynParamFloat64, &P_COUNT1);
    createParam(P_COUNT2String, asynParamFloat64, &P_COUNT2);
    createParam(P_COUNT3String, asynParamFloat64, &P_COUNT3);
    createParam(P_COUNT4String, asynParamFloat64, &P_COUNT4);
    createParam(P_TRIGCOUNTSString, asynParamFloat64, &P_TRIGCOUNTS);
    createParam(P_ENCODERString, asynParamFloat64, &P_ENCODER);
    createParam(P_TRIGGER_MODEString, asynParamInt32, &P_TRIGGER_MODE);
    createParam(P_TRIGGER_POLARITYString, asynParamInt32, &P_TRIGGER_POLARITY);
    createParam(P_TRIGGER_STARTString, asynParamInt32, &P_TRIGGER_START);
    createParam(P_TRIGGER_STOPString, asynParamInt32, &P_TRIGGER_STOP);
    createParam(P_TRIGGER_PAUSEString, asynParamInt32, &P_TRIGGER_PAUSE);
    createParam(P_SYSTEM_IPMODEString, asynParamInt32, &P_SYSTEM_IPMODE);
    createParam(P_READ_BUFFER1String, asynParamFloat64Array,  &P_READ_BUFFER1);
    createParam(P_READ_BUFFER2String, asynParamFloat64Array,  &P_READ_BUFFER2);
    createParam(P_READ_BUFFER3String, asynParamFloat64Array,  &P_READ_BUFFER3);
    createParam(P_READ_BUFFER4String, asynParamFloat64Array,  &P_READ_BUFFER4);
    createParam(P_READ_BUFFER_TRIGCOUNTSString, asynParamFloat64Array,  &P_READ_BUFFER_TRIGCOUNTS);

    pasynOctetSyncIO->connect(ip, 0, &pasynUserEcho, NULL);
    pasynOctetSyncIO->setInputEos(pasynUserEcho,  "\r\n", strlen("\r\n"));
    pasynOctetSyncIO->setOutputEos(pasynUserEcho, "\n",   strlen("\n"));

    //Set default TRIGGER MODE as internal
    //double result;
    //status = set_to_equipment(C400_MSG_TRIGGER_MODE_SET, C400_MSG_TRIGGER_MODE_ASK, 0, 1, result);
    //setIntegerParam (P_TRIGGER_MODE,      result);

    //Abort any counting while IOC is booting
    send_to_equipment(C400_MSG_ABORT_SET);

    status = (asynStatus) callParamCallbacks();
    status = (asynStatus)(epicsThreadCreate("c400countTask",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)::update_counts,
                          this) == NULL);
    if (status) {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
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
    int is_counting;
    int buffer_size;
    double result;

    while (true){
        getIntegerParam(P_ACQUIRE, &is_counting);
        getIntegerParam(P_BUFFER, &buffer_size);

        if (is_counting and buffer_size == 0){
            status = get_counts();
            status = get_to_equipment(C400_MSG_ENCODER_ASK, 1, result);
            setDoubleParam (P_ENCODER, result);
        }
        else if (is_counting and buffer_size < buffer_array_size and buffer_size != 0){
            status = get_buffer();
        }
        callParamCallbacks();
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
    int buffer_size;
    /* Set the parameter in the parameter library. */
    status = (asynStatus) setIntegerParam(function, value);

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    if (function == P_ACQUIRE and value == 0){
        send_to_equipment(C400_MSG_ABORT_SET);
        setIntegerParam (P_ACQUIRE,      0);
    }
    else if (function == P_ACQUIRE and value == 1){
        old_triggercounts = 0;
        accumulate_triggercounts = 0;
        send_to_equipment(C400_MSG_ACQUIRE_SET);
        setIntegerParam (P_ACQUIRE,      1);
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
    else if (function == P_ACQUIRE){
        setIntegerParam (P_ACQUIRE,      value);
        status = (asynStatus) callParamCallbacks();
        if (value==1)
            send_to_equipment(C400_MSG_ACQUIRE_SET);
        else if (value==0)
            send_to_equipment(C400_MSG_ABORT_SET);
        
    }
    else if (function == P_TRIGGER_MODE){
        status = set_to_equipment(C400_MSG_TRIGGER_MODE_SET, C400_MSG_TRIGGER_MODE_ASK, 0, value, result);
        setIntegerParam (P_TRIGGER_MODE,      result);
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
    else if (function == P_BUFFER){
        status = set_to_equipment(C400_MSG_BUFFER_SET, C400_MSG_BUFFER_ASK, 0, value, result);
        setIntegerParam (P_BUFFER,      result);
    }

    /* Do callbacks so higher layers see any changes */

    status = (asynStatus) callParamCallbacks();

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                  "%s:%s: status=%d, function=%d, name=%s, value=%d",
                  driverName, functionName, status, function, paramName, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, name=%s, value=%d\n",
              driverName, functionName, function, paramName, value);
    return status;
}


asynStatus c400drv::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    epicsInt32 run;
    const char *paramName;
    const char* functionName = "writeFloat64";
    double result;

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
    else if (function == P_PERIOD){
        status = set_to_equipment(C400_MSG_PERIOD_SET, C400_MSG_PERIOD_ASK, 0, value, result);
        setDoubleParam (P_PERIOD,      result);
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
    status = asynPortDriver::readInt32(pasynUser, value);

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
    else if (function == P_BUFFER){
        status = get_to_equipment(C400_MSG_BUFFER_ASK, 1, res);
        setIntegerParam (P_BUFFER,          res);
    }
    else {
        setIntegerParam (function,          res);
    }

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s",
                driverName, functionName, status, function, paramName);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, name=%s\n",
                driverName, functionName, function, paramName);
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
    status = asynPortDriver::readFloat64(pasynUser, value);

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
    else if (function == P_PERIOD){
        status = get_to_equipment(C400_MSG_PERIOD_ASK, 1, res);
        setDoubleParam (P_PERIOD,          res);
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

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s",
                driverName, functionName, status, function, paramName);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, name=%s\n",
                driverName, functionName, function, paramName);

    return status;
}

asynStatus c400drv::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                         size_t nElements, size_t *nIn)
{
    int function = pasynUser->reason;
    size_t ncopy;
    epicsInt32 itemp;
    asynStatus status = asynSuccess;
    epicsTimeStamp timeStamp;
    const char *functionName = "readFloat64Array";

    if (function == P_READ_BUFFER1) {
        memcpy(value, pData_ch1, sizeof(epicsFloat64));
        
    }
    else if (function == P_READ_BUFFER2) {
        memcpy(value, pData_ch2, sizeof(epicsFloat64));
        
    }
    else if (function == P_READ_BUFFER3) {
        memcpy(value, pData_ch3, sizeof(epicsFloat64));
        
    }
    else if (function == P_READ_BUFFER4) {
        memcpy(value, pData_ch4, sizeof(epicsFloat64));
        
    }

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                  "%s:%s: status=%d, function=%d",
                  driverName, functionName, status, function);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d\n",
              driverName, functionName, function);


    return status;
}

asynStatus c400drv::send_to_equipment(const char *writeBuffer, string &response)
{
    asynStatus status = asynSuccess;
    const char *functionName = "send_to_equipment";
    size_t nRead, nActual;
    string null_str = "";
    int eomReason;
    double readValue;
    char readBuffer[10000];

    status = pasynOctetSyncIO->writeRead(pasynUserEcho, writeBuffer, strlen(writeBuffer), readBuffer,
                                     sizeof(readBuffer), TIMEOUT, &nActual, &nRead, &eomReason);

    response = string(readBuffer);

    if (status) {
        epicsSnprintf(pasynUserEcho->errorMessage, pasynUserEcho->errorMessageSize,
                  "%s:%s: status=%d, response=%s",
                  driverName, functionName, status, string(readBuffer));
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
    string null_str = "";
    int eomReason;
    double readValue;    
    char readBuffer[10000];

    status = pasynOctetSyncIO->writeRead(pasynUserEcho, writeBuffer, strlen(writeBuffer), readBuffer,
                                     sizeof(readBuffer), TIMEOUT, &nActual, &nRead, &eomReason);

    if (status) {
        epicsSnprintf(pasynUserEcho->errorMessage, pasynUserEcho->errorMessageSize,
                  "%s:%s: status=%d, response=%s",
                  driverName, functionName, status, string(readBuffer));
    }

    //cout << "Status: " << status << endl;
    //cout << "Response: " << readBuffer << endl;

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

    double result_array[15];
    string response;

    // string cmd_msg_read;
    double ch1_val;
    double ch2_val;
    double ch3_val;
    double ch4_val;
    double trigger_counts_val;

    status = send_to_equipment(C400_MSG_COUNTS_ASK, response);
    parse_counts(result_array, response);

    ch1_val             = result_array[1];
    ch2_val             = result_array[2];
    ch3_val             = result_array[3];
    ch4_val             = result_array[4];
    trigger_counts_val  = result_array[6];
    
    setDoubleParam (P_COUNT1, ch1_val);
    setDoubleParam (P_COUNT2, ch2_val);
    setDoubleParam (P_COUNT3, ch3_val);
    setDoubleParam (P_COUNT4, ch4_val);
    setDoubleParam (P_TRIGCOUNTS, trigger_counts_val);

    return status;
}

asynStatus c400drv::get_buffer(){
    asynStatus status = asynSuccess;
    int end_loop_flag = 1;
    int p_data_array_pos = 0; // Position in the waveform array to be updated in the loop
    double temp_array[20]; // array to hold the data that is going to be placed in the PV array 
    string data_not_collected_code = "-401,";
    string base_msg = "FETch:COUNts? ";
    string val; // Placeholder for the retrieved equipmente message
    int buffer_size;
    string get_full_buffer_str;
    getIntegerParam (P_BUFFER,      &buffer_size);
    get_full_buffer_str = base_msg + to_string(buffer_size);

    pData_ch1 = (epicsFloat64 *)calloc(buffer_size, sizeof(epicsFloat64));
    pData_ch2 = (epicsFloat64 *)calloc(buffer_size, sizeof(epicsFloat64));
    pData_ch3 = (epicsFloat64 *)calloc(buffer_size, sizeof(epicsFloat64));
    pData_ch4 = (epicsFloat64 *)calloc(buffer_size, sizeof(epicsFloat64));
    pData_trigcounts = (epicsFloat64 *)calloc(buffer_size, sizeof(epicsFloat64));

    for (int i = 0; i < buffer_size; ++i) {  // Reset the array
        pData_ch1[i] = 0;
        pData_ch2[i] = 0;
        pData_ch3[i] = 0;
        pData_ch4[i] = 0;
        pData_trigcounts[i] = 0;
    }

    // Need to parse several lines, each one ending with "\r\n"
    // The equipment also return a blank line at the end, wich means that the last line is \r\n\r\n
    // So we can use this to tell that this is the last line and get all previous ones as well
    // the problem is that asyn matches any \r\n and stop listening to the equipment
    // Soultion: Use the middle pattern that appears in the string \r\n\r\n, which is \n\r
    // Return to the standard input eos afterwards 

    while (end_loop_flag){

        pasynOctetSyncIO->setInputEos(pasynUserEcho, "\n\r", strlen("\n\r"));
        status = send_to_equipment(get_full_buffer_str.c_str(), val);
        pasynOctetSyncIO->setInputEos(pasynUserEcho, "\r\n", strlen("\r\n"));

        istringstream iss(val);
        for (string line; getline(iss, line); ){
            const char* first_5_char = line.substr(0, 5).c_str();

            if (strcmp(first_5_char, "FETch") == 0){
                continue;
            }
            else if (strcmp(first_5_char,  data_not_collected_code.c_str()) == 0){
                if (p_data_array_pos == buffer_size){
                    end_loop_flag = 0;
                    break;
                }
                continue;
            }
            else {
                parse_counts(temp_array, line);

                pData_ch1[p_data_array_pos]         = temp_array[1];
                pData_ch2[p_data_array_pos]         = temp_array[2];
                pData_ch3[p_data_array_pos]         = temp_array[3];
                pData_ch4[p_data_array_pos]         = temp_array[4];
                pData_trigcounts[p_data_array_pos]  = temp_array[6];

                p_data_array_pos ++;
            }
        }
    }

    doCallbacksFloat64Array(pData_ch1,          buffer_size,  P_READ_BUFFER1,           0);
    doCallbacksFloat64Array(pData_ch2,          buffer_size,  P_READ_BUFFER2,           0);
    doCallbacksFloat64Array(pData_ch3,          buffer_size,  P_READ_BUFFER3,           0);
    doCallbacksFloat64Array(pData_ch4,          buffer_size,  P_READ_BUFFER4,           0);
    doCallbacksFloat64Array(pData_trigcounts,   buffer_size,  P_READ_BUFFER_TRIGCOUNTS, 0);

    setIntegerParam (P_ACQUIRE, 0);

    return status;
}

void c400drv::parse_counts(double *result_array, string val)
{
    string delimiter = ",";
    size_t pos = 0;
    string token;
    string last_char;

    token = val.substr(0, val.find("\n")+1);
    val = val.substr(token.length(), val.length() - token.length());

    // result_array[0] = token;
    int array_idx = 0;
    while ((pos = val.find(delimiter)) != string::npos) {
        token = val.substr(0, pos);
        last_char = token[token.length() - 1];
        if (last_char == "S" or last_char == "V"){
            token = token.substr(0, token.length() - 2);
        }
        try
        {
            if (array_idx == 6) {
                if (stod(token) < old_triggercounts) {
                   accumulate_triggercounts = accumulate_triggercounts + 256;
                }
                old_triggercounts = stod(token);
                result_array[array_idx] = accumulate_triggercounts + stod(token) + 1;
            }
            else {
                result_array[array_idx] = stod(token);
            }
        }
        catch(const exception& e)
        {
            cerr << e.what() << '\n';
        }
        
        
        val.erase(0, pos + delimiter.length());
        
        array_idx++;
    }
    try
    {
        result_array[array_idx] = stod(val); //Append the last val
    }
    catch(const exception& e)
    {
        cerr << e.what() << '\n';
    }
    

}

//--------------------------------------------------------

extern "C" int c400CreateDriver(const char *portName, char *ip){
    new c400drv(portName, ip);
    return(asynSuccess);
}

static const iocshArg portNameArg = { "Port name", iocshArgString};
static const iocshArg ipArg = { "IP", iocshArgString};
static const iocshArg * const createDriverArgs[] = {&portNameArg, &ipArg};
static const iocshFuncDef createDriverFuncDef = {"c400CreateDriver", 2, createDriverArgs};

static void createDriverCallFunc(const iocshArgBuf *args){
    c400CreateDriver(args[0].sval, args[1].sval);
}

void c400drvRegister(void){
    iocshRegister(&createDriverFuncDef,createDriverCallFunc);
}

extern "C" {
    epicsExportRegistrar(c400drvRegister);
}

