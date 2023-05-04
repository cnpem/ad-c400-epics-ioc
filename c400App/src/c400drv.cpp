#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <sstream>

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

#define TIMEOUT 2.0 // Scintillator response timeout
#define C400_MSG_DAC_ASK "CONFigure:DAC?"
#define C400_MSG_DAC_SET "CONFigure:DAC "
#define C400_MSG_DEAD_ASK "CONFigure:DEADtime?"
#define C400_MSG_DEAD_SET "CONFigure:DEADtime "
#define C400_MSG_DHI_ASK "CONFigure:DHI?"
#define C400_MSG_DHI_SET "CONFigure:DHI "
#define C400_MSG_DLO_ASK "CONFigure:DLO?"
#define C400_MSG_DLO_SET "CONFigure:DLO "
#define C400_MSG_HIVO_VOLTS_ASK "CONFigure:HIVOltage:VOLts?"
#define C400_MSG_HIVO_VOLTS_SET "CONFigure:HIVOltage:VOLts "
#define C400_MSG_HIVO_ENABLE_ASK "CONFigure:HIVOltage:ENable?"
#define C400_MSG_HIVO_ENABLE_SET "CONFigure:HIVOltage:ENable "
#define C400_MSG_PERIOD_ASK "CONFigure:PERiod?"
#define C400_MSG_PERIOD_SET "CONFigure:PERiod "
#define C400_MSG_POLARITY_ASK "CONFigure:POLarity"
#define C400_MSG_POLARITY_SET "CONFigure:POLarity "
#define C400_MSG_PULSER_ASK "CONFigure:PULser?"
#define C400_MSG_PULSER_SET "CONFigure:PULser "
#define C400_MSG_ACQUIRE_SET "INITiate"
#define C400_MSG_ABORT_SET "ABORt"
#define C400_MSG_BUFFER_ASK "TRIGger:BUFfer?"
#define C400_MSG_BUFFER_SET "TRIGger:BUFfer "
#define C400_MSG_BURST_ASK "TRIGger:BURst?"
#define C400_MSG_BURST_SET "TRIGger:BURst "
#define C400_MSG_COUNTS_ASK "FETch:COUNts?"
#define C400_MSG_TRIGGER_MODE_ASK "TRIGger:MODE?"
#define C400_MSG_TRIGGER_MODE_SET "TRIGger:MODE "
#define C400_MSG_TRIGGER_POLARITY_ASK "TRIGger:POLarity?"
#define C400_MSG_TRIGGER_POLARITY_SET "TRIGger:POLarity "
#define C400_MSG_TRIGGER_START_ASK "TRIGger:SOURce:STARt?"
#define C400_MSG_TRIGGER_START_SET "TRIGger:SOURce:STARt "
#define C400_MSG_TRIGGER_STOP_ASK "TRIGger:SOURce:STOP?"
#define C400_MSG_TRIGGER_STOP_SET "TRIGger:SOURce:STOP "
#define C400_MSG_TRIGGER_PAUSE_ASK "TRIGger:SOURce:PAUSE?"
#define C400_MSG_TRIGGER_PAUSE_SET "TRIGger:SOURce:PAUSE "
#define C400_MSG_SYSTEM_IPMODE_ASK "SYSTem:COMMunication:IPMODE?"
#define C400_MSG_SYSTEM_IPMODE_SET "SYSTem:COMMunication:IPMODE "

#define buffer_array_size 65536*8

void update_counts(void *drvPvt);
static const char *driverName = "c400driver";
static std::string trigger_mode_mbbo[]={"CUSTom", "INTernal", "EXTERNAL_START", "EXTERNAL_START_STOP",
                                        "EXTERNAL_START_HOLD", "EXTERNAL_WINDOWED", "DISCRIMINATOR_SWEEP"};
static std::string system_ipmode_mbbo[]={"DHCP", "Static"};

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
    int maxPoints = buffer_array_size;
    pData_ = (epicsFloat64 *)calloc(maxPoints, sizeof(epicsFloat64));

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
    createParam(P_TRIGGER_MODEString, asynParamInt32, &P_TRIGGER_MODE);
    createParam(P_TRIGGER_POLARITYString, asynParamInt32, &P_TRIGGER_POLARITY);
    createParam(P_TRIGGER_STARTString, asynParamInt32, &P_TRIGGER_START);
    createParam(P_TRIGGER_STOPString, asynParamInt32, &P_TRIGGER_STOP);
    createParam(P_TRIGGER_PAUSEString, asynParamInt32, &P_TRIGGER_PAUSE);
    createParam(P_SYSTEM_IPMODEString, asynParamInt32, &P_SYSTEM_IPMODE);
    createParam(P_UPDATE_BUFFERString, asynParamInt32,  &P_UPDATE_BUFFER);
    createParam(P_READ_BUFFERString, asynParamFloat64Array,  &P_READ_BUFFER);

    pasynOctetSyncIO->connect(ip, 0, &pasynUserEcho, NULL);
    pasynOctetSyncIO->setInputEos(pasynUserEcho, "\r\n", strlen("\r\n"));
    pasynOctetSyncIO->setOutputEos(pasynUserEcho, "\n", strlen("\n"));

    //Set default TRIGGER MODE as internal
    set_mbbo(C400_MSG_TRIGGER_MODE_SET, trigger_mode_mbbo, 1); 
    setIntegerParam (P_TRIGGER_MODE,      1);

    //Abort any counting while IOC is booting
    send_to_equipment(C400_MSG_ABORT_SET);

    //Set counter to 0 as default
    setDoubleParam (P_COUNT1,          0);
    setDoubleParam (P_COUNT2,          0);
    setDoubleParam (P_COUNT3,          0);
    setDoubleParam (P_COUNT4,          0);

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
    int is_counting;
    double sleep_for;
    int buffer_size;

    while (true){
        getIntegerParam(P_ACQUIRE, &is_counting);
        getIntegerParam(P_BUFFER, &buffer_size);
        getDoubleParam(P_PERIOD, &sleep_for);
        if (is_counting and buffer_size == 0){
            get_n_set_4_channels(C400_MSG_COUNTS_ASK, P_COUNT1, P_COUNT2, 
                                P_COUNT3, P_COUNT4, 2,3,4,5);
            callParamCallbacks();
        }
        sleep(sleep_for);
    }
}

asynStatus c400drv::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName;
    const char* functionName = "writeInt32";
    double result;
    int is_counting;
    int was_counting = 0;
    int buffer_size;
    /* Set the parameter in the parameter library. */
    status = (asynStatus) setIntegerParam(function, value);

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    getIntegerParam(P_BUFFER, &buffer_size);
    getIntegerParam(P_ACQUIRE, &is_counting);
    if (is_counting and function != P_ACQUIRE and buffer_size == 0){
        send_to_equipment(C400_MSG_ABORT_SET);
        setIntegerParam (P_ACQUIRE,      0);
        was_counting = 1;
        status = (asynStatus) callParamCallbacks();
    }


    if (function == P_HIVO_ENABLE1) {
        result = set_4_channels_int(C400_MSG_HIVO_ENABLE_SET, C400_MSG_HIVO_ENABLE_ASK, 
                                P_HIVO_ENABLE1, P_HIVO_ENABLE2, P_HIVO_ENABLE3, P_HIVO_ENABLE4, 1, value);
        setIntegerParam (P_HIVO_ENABLE1,      result);
    }
    else if (function == P_HIVO_ENABLE2){
        result = set_4_channels_int(C400_MSG_HIVO_ENABLE_SET, C400_MSG_HIVO_ENABLE_ASK, 
                                P_HIVO_ENABLE1, P_HIVO_ENABLE2, P_HIVO_ENABLE3, P_HIVO_ENABLE4, 2, value);
        setIntegerParam (P_HIVO_ENABLE2,      result);
    }
    else if (function == P_HIVO_ENABLE3){
        result = set_4_channels_int(C400_MSG_HIVO_ENABLE_SET, C400_MSG_HIVO_ENABLE_ASK, 
                                P_HIVO_ENABLE1, P_HIVO_ENABLE2, P_HIVO_ENABLE3, P_HIVO_ENABLE4, 3, value);
        setIntegerParam (P_HIVO_ENABLE3,      result);
    }
    else if (function == P_HIVO_ENABLE4){
        result = set_4_channels_int(C400_MSG_HIVO_ENABLE_SET, C400_MSG_HIVO_ENABLE_ASK, 
                                P_HIVO_ENABLE1, P_HIVO_ENABLE2, P_HIVO_ENABLE3, P_HIVO_ENABLE4, 4, value);
        setIntegerParam (P_HIVO_ENABLE4,      result);
    }
    else if (function == P_POLARITY1) {
        result = set_4_channels_int(C400_MSG_POLARITY_SET, C400_MSG_POLARITY_ASK, 
                                P_POLARITY1, P_POLARITY2, P_POLARITY3, P_POLARITY4, 1, value, 1);
        setIntegerParam (P_POLARITY1,      result);
    }
    else if (function == P_POLARITY2){
        result = set_4_channels_int(C400_MSG_POLARITY_SET, C400_MSG_POLARITY_ASK, 
                                P_POLARITY1, P_POLARITY2, P_POLARITY3, P_POLARITY4, 2, value, 1);
        setIntegerParam (P_POLARITY2,      result);
    }
    else if (function == P_POLARITY3){
        result = set_4_channels_int(C400_MSG_POLARITY_SET, C400_MSG_POLARITY_ASK, 
                                P_POLARITY1, P_POLARITY2, P_POLARITY3, P_POLARITY4, 3, value, 1);
        setIntegerParam (P_POLARITY3,      result);
    }
    else if (function == P_POLARITY4){
        result = set_4_channels_int(C400_MSG_POLARITY_SET, C400_MSG_POLARITY_ASK, 
                                P_POLARITY1, P_POLARITY2, P_POLARITY3, P_POLARITY4, 4, value, 1);
        setIntegerParam (P_POLARITY4,      result);
    }
    else if (function == P_ACQUIRE){
        if (value==1)
            send_to_equipment(C400_MSG_ACQUIRE_SET);
        else if (value==0)
            send_to_equipment(C400_MSG_ABORT_SET);
        setIntegerParam (P_ACQUIRE,      value);
    }
    else if (function == P_TRIGGER_MODE){
        set_mbbo(C400_MSG_TRIGGER_MODE_SET, trigger_mode_mbbo, value);
        setIntegerParam (P_TRIGGER_MODE,      value);
    }
    else if (function == P_SYSTEM_IPMODE){
        set_mbbo(C400_MSG_SYSTEM_IPMODE_SET, system_ipmode_mbbo, value);
        setIntegerParam (P_SYSTEM_IPMODE,      value);
    }
    else if (function == P_TRIGGER_POLARITY){
        result = set_direct(C400_MSG_TRIGGER_POLARITY_SET, C400_MSG_TRIGGER_POLARITY_ASK, 0, value);
        setIntegerParam (P_TRIGGER_POLARITY,      result);
    }
    else if (function == P_TRIGGER_START){
        result = set_direct(C400_MSG_TRIGGER_START_SET, C400_MSG_TRIGGER_START_ASK, 0, value);
        setIntegerParam (P_TRIGGER_START,      result);
    }
    else if (function == P_TRIGGER_STOP){
        result = set_direct(C400_MSG_TRIGGER_STOP_SET, C400_MSG_TRIGGER_STOP_ASK, 0, value);
        setIntegerParam (P_TRIGGER_STOP,      result);
    }
    else if (function == P_TRIGGER_PAUSE){
        result = set_direct(C400_MSG_TRIGGER_PAUSE_SET, C400_MSG_TRIGGER_PAUSE_ASK, 0, value);
        setIntegerParam (P_TRIGGER_PAUSE,      result);
    }
    else if (function == P_BUFFER){
        result = set_direct(C400_MSG_BUFFER_SET, C400_MSG_BUFFER_ASK, 0, value);
        setIntegerParam (P_BUFFER,      value);
    }
    else if (function == P_UPDATE_BUFFER){
        update_buffer(13);
        setIntegerParam (P_UPDATE_BUFFER,      value);
    }


    /* Do callbacks so higher layers see any changes */

    if (was_counting){
        send_to_equipment(C400_MSG_ACQUIRE_SET);
        setIntegerParam (P_ACQUIRE,      1);
    }

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
    std::cout << "Write" << std::endl;
    const char *paramName;
    const char* functionName = "writeFloat64";
    double result;
    int is_counting;
    int was_counting = 0;

    getIntegerParam(P_ACQUIRE, &is_counting);
    if (is_counting and function != P_ACQUIRE){
        send_to_equipment(C400_MSG_ABORT_SET);
        setIntegerParam (P_ACQUIRE,      0);
        was_counting = 1;
        status = (asynStatus) callParamCallbacks();
    }
    if (function == P_DAC1) {
        result = set_direct(C400_MSG_DAC_SET, C400_MSG_DAC_ASK, 1, value);
        setDoubleParam (P_DAC1,      result);
    }
    else if (function == P_DAC2){
        result = set_direct(C400_MSG_DAC_SET, C400_MSG_DAC_ASK, 2, value);
        setDoubleParam (P_DAC2,      result);
    }
    else if (function == P_DAC3){
        result = set_direct(C400_MSG_DAC_SET, C400_MSG_DAC_ASK, 3, value);
        setDoubleParam (P_DAC3,      result);
    }
    else if (function == P_DAC4){
        result = set_direct(C400_MSG_DAC_SET, C400_MSG_DAC_ASK, 4, value);
        setDoubleParam (P_DAC4,      result);
    }
    else if (function == P_DEAD){
        result = set_direct(C400_MSG_DEAD_SET, C400_MSG_DEAD_ASK, 0, value);
        setDoubleParam (P_DEAD,      value);
    }
    else if (function == P_DHI1) {
        result = set_4_channels(C400_MSG_DHI_SET, C400_MSG_DHI_ASK, 
                                P_DHI1, P_DHI2, P_DHI3, P_DHI4, 1, value);
        setDoubleParam (P_DHI1,      result);
    }
    else if (function == P_DHI2){
        result = set_4_channels(C400_MSG_DHI_SET, C400_MSG_DHI_ASK, 
                                P_DHI1, P_DHI2, P_DHI3, P_DHI4, 2, value);
        setDoubleParam (P_DHI2,      result);
    }
    else if (function == P_DHI3){
        result = set_4_channels(C400_MSG_DHI_SET, C400_MSG_DHI_ASK, 
                                P_DHI1, P_DHI2, P_DHI3, P_DHI4, 3, value);
        setDoubleParam (P_DHI3,      result);
    }
    else if (function == P_DHI4){
        result = set_4_channels(C400_MSG_DHI_SET, C400_MSG_DHI_ASK, 
                                P_DHI1, P_DHI2, P_DHI3, P_DHI4, 4, value);
        setDoubleParam (P_DHI4,      result);
    }
    else if (function == P_DLO1) {
        result = set_4_channels(C400_MSG_DLO_SET, C400_MSG_DLO_ASK, 
                                P_DLO1, P_DLO2, P_DLO3, P_DLO4, 1, value);
        setDoubleParam (P_DLO1,      result);
    }
    else if (function == P_DLO2){
        result = set_4_channels(C400_MSG_DLO_SET, C400_MSG_DLO_ASK, 
                                P_DLO1, P_DLO2, P_DLO3, P_DLO4, 2, value);
        setDoubleParam (P_DLO2,      result);
    }
    else if (function == P_DLO3){
        result = set_4_channels(C400_MSG_DLO_SET, C400_MSG_DLO_ASK, 
                                P_DLO1, P_DLO2, P_DLO3, P_DLO4, 3, value);
        setDoubleParam (P_DLO3,      result);
    }
    else if (function == P_DLO4){
        result = set_4_channels(C400_MSG_DLO_SET, C400_MSG_DLO_ASK, 
                                P_DLO1, P_DLO2, P_DLO3, P_DLO4, 4, value);
        setDoubleParam (P_DLO4,      result);
    }
    else if (function == P_HIVO_VOLTS1) {
        result = set_4_channels(C400_MSG_HIVO_VOLTS_SET, C400_MSG_HIVO_VOLTS_ASK, 
                                P_HIVO_VOLTS1, P_HIVO_VOLTS2, P_HIVO_VOLTS3, P_HIVO_VOLTS4, 1, value);
        setDoubleParam (P_HIVO_VOLTS1,      result);
    }
    else if (function == P_HIVO_VOLTS2){
        result = set_4_channels(C400_MSG_HIVO_VOLTS_SET, C400_MSG_HIVO_VOLTS_ASK, 
                                P_HIVO_VOLTS1, P_HIVO_VOLTS2, P_HIVO_VOLTS3, P_HIVO_VOLTS4, 2, value);
        setDoubleParam (P_HIVO_VOLTS2,      result);
    }
    else if (function == P_HIVO_VOLTS3){
        result = set_4_channels(C400_MSG_HIVO_VOLTS_SET, C400_MSG_HIVO_VOLTS_ASK, 
                                P_HIVO_VOLTS1, P_HIVO_VOLTS2, P_HIVO_VOLTS3, P_HIVO_VOLTS4, 3, value);
        setDoubleParam (P_HIVO_VOLTS3,      result);
    }
    else if (function == P_HIVO_VOLTS4){
        result = set_4_channels(C400_MSG_HIVO_VOLTS_SET, C400_MSG_HIVO_VOLTS_ASK, 
                                P_HIVO_VOLTS1, P_HIVO_VOLTS2, P_HIVO_VOLTS3, P_HIVO_VOLTS4, 4, value);
        setDoubleParam (P_HIVO_VOLTS4,      result);
    }
    else if (function == P_PERIOD){
        result = set_direct(C400_MSG_PERIOD_SET, C400_MSG_PERIOD_ASK, 0, value);
        setDoubleParam (P_PERIOD,      value);
    }
    else if (function == P_PULSER_Period) {
        result = set_2_vals(C400_MSG_PULSER_SET, C400_MSG_PULSER_ASK, 
                                P_PULSER_Period, P_PULSER_Width, 1, value);
        setDoubleParam (P_PULSER_Period,      result);
    }
    else if (function == P_PULSER_Width) {
        result = set_2_vals(C400_MSG_PULSER_SET, C400_MSG_PULSER_ASK, 
                                P_PULSER_Period, P_PULSER_Width, 2, value);
        setDoubleParam (P_PULSER_Width,      result);
    }
    else if (function == P_BURST){
        result = set_direct(C400_MSG_BURST_SET, C400_MSG_BURST_ASK, 0, value);
        setDoubleParam (P_BURST,      value);
    }

    if (was_counting){
        send_to_equipment(C400_MSG_ACQUIRE_SET);
        setIntegerParam (P_ACQUIRE,      1);
    }


    status = (asynStatus) callParamCallbacks();

    return status;
}

asynStatus c400drv::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr = 0;
    const char *paramName;
    const char* functionName = "readInt32";
    double res;
    std::string cmd_msg;
    std::cout << "Read" << std::endl;
    /* Get channel for possible use */
    status = getAddress(pasynUser, &addr);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s",
                driverName, functionName, status, function, paramName);
        return status;
    }
    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    status = asynPortDriver::readInt32(pasynUser, value);

    if (function == P_HIVO_ENABLE1) {
        res = get_parsed_response(send_to_equipment(C400_MSG_HIVO_ENABLE_ASK), 1);
        setIntegerParam (P_HIVO_ENABLE1,          res);
    }
    else if (function == P_HIVO_ENABLE2) {
        res = get_parsed_response(send_to_equipment(C400_MSG_HIVO_ENABLE_ASK), 2);
        setIntegerParam (P_HIVO_ENABLE2,          res);
    }
    else if (function == P_HIVO_ENABLE3) {
        res = get_parsed_response(send_to_equipment(C400_MSG_HIVO_ENABLE_ASK), 3);
        setIntegerParam (P_HIVO_ENABLE3,          res);
    }
    else if (function == P_HIVO_ENABLE4) {
        res = get_parsed_response(send_to_equipment(C400_MSG_HIVO_ENABLE_ASK), 4);
        setIntegerParam (P_HIVO_ENABLE4,          res);
    }
    else if (function == P_POLARITY1) {
        res = get_parsed_response(send_to_equipment(C400_MSG_POLARITY_ASK), 1);
        setIntegerParam (P_POLARITY1,          res);
    }
    else if (function == P_POLARITY2) {
        res = get_parsed_response(send_to_equipment(C400_MSG_POLARITY_ASK), 2);
        setIntegerParam (P_POLARITY2,          res);
    }
    else if (function == P_POLARITY3) {
        res = get_parsed_response(send_to_equipment(C400_MSG_POLARITY_ASK), 3);
        setIntegerParam (P_POLARITY3,          res);
    }
    else if (function == P_POLARITY4) {
        res = get_parsed_response(send_to_equipment(C400_MSG_POLARITY_ASK), 4);
        setIntegerParam (P_POLARITY4,          res);
    }
    else if (function == P_BUFFER){
        res = get_parsed_response(send_to_equipment(C400_MSG_BUFFER_ASK), 1);
        setIntegerParam (P_BUFFER,          res);
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
    int addr = 0;
    const char *paramName;
    const char* functionName = "readFloat64";
    double res;
    std::string cmd_msg;
    std::cout << "Read" << std::endl;
    /* Get channel for possible use */
    status = getAddress(pasynUser, &addr);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s",
                driverName, functionName, status, function, paramName);
        return status;
    }
    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    status = asynPortDriver::readFloat64(pasynUser, value);

    if (function == P_DAC1) {
        res = get_parsed_response(send_to_equipment(C400_MSG_DAC_ASK), 1);
        setDoubleParam (P_DAC1,          res);
    }
    else if (function == P_DAC2){
        res = get_parsed_response(send_to_equipment(C400_MSG_DAC_ASK), 2);
        setDoubleParam (P_DAC2,          res);
    }
    else if (function == P_DAC3){
        res = get_parsed_response(send_to_equipment(C400_MSG_DAC_ASK), 3);
        setDoubleParam (P_DAC3,          res);
    }
    else if (function == P_DAC4){
        res = get_parsed_response(send_to_equipment(C400_MSG_DAC_ASK), 4);
        setDoubleParam (P_DAC4,          res);
    }
    else if (function == P_DEAD){
        res = get_parsed_response(send_to_equipment(C400_MSG_DEAD_ASK), 1);
        setDoubleParam (P_DEAD,          res);
    }
    else if (function == P_DHI1) {
        res = get_parsed_response(send_to_equipment(C400_MSG_DHI_ASK), 1);
        setDoubleParam (P_DHI1,          res);
    }
    else if (function == P_DHI2){
        res = get_parsed_response(send_to_equipment(C400_MSG_DHI_ASK), 2);
        setDoubleParam (P_DHI2,          res);
    }
    else if (function == P_DHI3){
        res = get_parsed_response(send_to_equipment(C400_MSG_DHI_ASK), 3);
        setDoubleParam (P_DHI3,          res);
    }
    else if (function == P_DHI4){
        res = get_parsed_response(send_to_equipment(C400_MSG_DHI_ASK), 4);
        setDoubleParam (P_DHI4,          res);
    }
    else if (function == P_DLO1) {
        res = get_parsed_response(send_to_equipment(C400_MSG_DLO_ASK), 1);
        setDoubleParam (P_DLO1,          res);
    }
    else if (function == P_DLO2){
        res = get_parsed_response(send_to_equipment(C400_MSG_DLO_ASK), 2);
        setDoubleParam (P_DLO2,          res);
    }
    else if (function == P_DLO3){
        res = get_parsed_response(send_to_equipment(C400_MSG_DLO_ASK), 3);
        setDoubleParam (P_DLO3,          res);
    }

    else if (function == P_DLO4){
        res = get_parsed_response(send_to_equipment(C400_MSG_DLO_ASK), 4);
        setDoubleParam (P_DLO4,          res);
    }
    else if (function == P_HIVO_VOLTS1) {
        res = get_parsed_response(send_to_equipment(C400_MSG_HIVO_VOLTS_ASK), 1);
        setDoubleParam (P_HIVO_VOLTS1,          res);
    }
    else if (function == P_HIVO_VOLTS2){
        res = get_parsed_response(send_to_equipment(C400_MSG_HIVO_VOLTS_ASK), 2);
        setDoubleParam (P_HIVO_VOLTS2,          res);
    }
    else if (function == P_HIVO_VOLTS3){
        res = get_parsed_response(send_to_equipment(C400_MSG_HIVO_VOLTS_ASK), 3);
        setDoubleParam (P_HIVO_VOLTS3,          res);
    }
    else if (function == P_HIVO_VOLTS4){
        res = get_parsed_response(send_to_equipment(C400_MSG_HIVO_VOLTS_ASK), 4);
        setDoubleParam (P_HIVO_VOLTS4,          res);
    }
    else if (function == P_PERIOD){
        res = get_parsed_response(send_to_equipment(C400_MSG_PERIOD_ASK), 1);
        setDoubleParam (P_PERIOD,          res);
    }
    else if (function == P_PULSER_Period) {
        res = get_parsed_response(send_to_equipment(C400_MSG_PULSER_ASK), 1);
        setDoubleParam (P_PULSER_Period,          res);
    }
    else if (function == P_PULSER_Width) {
        res = get_parsed_response(send_to_equipment(C400_MSG_PULSER_ASK), 2);
        setDoubleParam (P_PULSER_Width,          res);
    }
    else if (function == P_BURST){
        res = get_parsed_response(send_to_equipment(C400_MSG_BURST_ASK), 1);
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

    // std::cout << "im inside readFloatArray" << std::endl;

    if (function == P_READ_BUFFER) {
        // memcpy(value, pData_, ncopy*sizeof(epicsFloat64));
        // *nIn = ncopy;
        ncopy = 100;
        memcpy(value, pData_, ncopy*sizeof(epicsFloat64));
        
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

std::string c400drv::send_to_equipment(const char *writeBuffer)
{
    asynStatus status = asynSuccess;
    size_t nRead, nActual;
    std::string null_str = "";
    int eomReason;
    double readValue;    
    char readBuffer[1000];

    status = pasynOctetSyncIO->writeRead(pasynUserEcho, writeBuffer, strlen(writeBuffer), readBuffer,
                                     sizeof(readBuffer), TIMEOUT, &nActual, &nRead, &eomReason);

    
    std::cout << "status: " << status << std::endl;
    std::cout << "Buffer: " << readBuffer << std::endl;
    // if (status == 1){
    //     return null_str;
    // }
    return std::string (readBuffer);

}
float c400drv::get_parsed_response(std::string val, int n_element, std::string delimiter)
{
    int pos = 0;
    std::string result_array[15];
    std::string token;
    std::cout << "Entering msg: " << val << std::endl;
    token = val.substr(0, val.find("\n")+1);
    val = val.substr(token.length(), val.length() - token.length());
    float res;
    
    result_array[0] = token;
    int array_idx = 1;
    while (array_idx < 12) {
        pos = val.find(delimiter);
        token = val.substr(0, pos);
        result_array[array_idx] = token;
        val.erase(0, pos + delimiter.length());
        std::cout << "pos: " << pos << std::endl;
        if (pos == -1){
            result_array[array_idx] = val; //Append the last val
            break;
        }
        array_idx++;
    }
    
    if (result_array[n_element] == "P"){
        std::cout << "Got: " << "P" << std::endl;
        return 1;
    }
    else if (result_array[n_element] == "N"){
        std::cout << "Got: " << "N" << std::endl;
        return 0;
    }

    try{
        std::cout << "Value to convert: " << result_array[n_element] << std::endl;
        return std::stof(result_array[n_element]);
    }
    catch (...){
        std::cout << "Error converting to float" << std::endl;
        return 0;
    }
}


double c400drv::set_direct(const char *command_set, const char *command_ask, int channel, double val)
{
    // If 0 is passed to channel, them no specific channel is set
    double res;
    std::string res_str;
    std::string str_channel;
    std::string str_val;
    std::string token;
    std::string cmd_msg_send;
    std::string cmd_msg_read;

    str_channel = std::__cxx11::to_string(channel);
    str_val = std::__cxx11::to_string(val);

    if (channel){
        cmd_msg_send =  command_set + str_channel + " " + str_val;
        send_to_equipment(cmd_msg_send.c_str());
        cmd_msg_read = command_ask;
        res = get_parsed_response(send_to_equipment(cmd_msg_read.c_str()), channel);
        return res;
    }

    else{
        if (command_set==C400_MSG_TRIGGER_START_SET or command_set==C400_MSG_TRIGGER_STOP_SET or command_set==C400_MSG_TRIGGER_PAUSE_SET){
            if (int(val)==0)
                str_val = "INTernal";
            else if (int(val)==1)
                str_val = "BNC";
        }
        cmd_msg_send =  command_set + str_val;
        send_to_equipment(cmd_msg_send.c_str());
        res_str = send_to_equipment(command_ask);
        token = res_str.substr(0, res_str.find("\n")+1);
        res_str = res_str.substr(token.length(), res_str.length() - token.length());
        std::cout << res_str << std::endl;
        
        if (res_str=="INTernal")
            return 0;
        else if(res_str=="BNC")
            return 1;

        try{

            return std::stof(res_str);
        }
        catch (...){
            std::cout << "Error converting from str to float" << std::endl;
            return 0;
        }
        
    }
}

double c400drv::set_4_channels(const char *command_set, const char *command_ask, int param1, int param2, 
                               int param3, int param4, int channel, double val)
{
        asynStatus status = asynSuccess;
        double res;
        double val_ch1;
        double val_ch2;
        double val_ch3;
        double val_ch4;
        std::string str_val_ch1;
        std::string str_val_ch2;
        std::string str_val_ch3;
        std::string str_val_ch4;
        std::string cmd_msg_send;
        std::string cmd_msg_read;

        // Check if the value to feed is a float or integer and handles it

        getDoubleParam(param1, &val_ch1);
        getDoubleParam(param2, &val_ch2);
        getDoubleParam(param3, &val_ch3);
        getDoubleParam(param4, &val_ch4);
        
        std::cout << "val1: " << val_ch1 << std::endl;
        std::cout << "val2: " << val_ch2 << std::endl;
        std::cout << "val3: " << val_ch3 << std::endl;
        std::cout << "val4: " << val_ch4 << std::endl;

        switch (channel)
        {
            case 1:
            str_val_ch1 = std::__cxx11::to_string(val);
            str_val_ch2 = std::__cxx11::to_string(val_ch2);
            str_val_ch3 = std::__cxx11::to_string(val_ch3);
            str_val_ch4 = std::__cxx11::to_string(val_ch4);
            break;

            case 2:
            str_val_ch1 = std::__cxx11::to_string(val_ch1);
            str_val_ch2 = std::__cxx11::to_string(val);
            str_val_ch3 = std::__cxx11::to_string(val_ch3);
            str_val_ch4 = std::__cxx11::to_string(val_ch4);
            break;
            
            case 3:
            str_val_ch1 = std::__cxx11::to_string(val_ch1);
            str_val_ch2 = std::__cxx11::to_string(val_ch2);
            str_val_ch3 = std::__cxx11::to_string(val);
            str_val_ch4 = std::__cxx11::to_string(val_ch4);
            break;

            case 4:
            str_val_ch1 = std::__cxx11::to_string(val_ch1);
            str_val_ch2 = std::__cxx11::to_string(val_ch2);
            str_val_ch3 = std::__cxx11::to_string(val_ch3);
            str_val_ch4 = std::__cxx11::to_string(val);
            break;
        }

        cmd_msg_send = command_set + str_val_ch1 + " " + str_val_ch2 + " "\
                                        + str_val_ch3 + " " + str_val_ch4 + " ";
        send_to_equipment(cmd_msg_send.c_str());
        cmd_msg_read = command_ask;


        res = get_parsed_response(send_to_equipment(cmd_msg_read.c_str()), channel);

        std::cout << "my res is: " << res << std::endl;
        return res;
}

double c400drv::set_4_channels_int(const char *command_set, const char *command_ask, int param1, int param2, 
                               int param3, int param4, int channel, double val, int to_string)
{
        asynStatus status = asynSuccess;
        double res;
        int val_ch1;
        int val_ch2;
        int val_ch3;
        int val_ch4;
        std::string str_val_ch1;
        std::string str_val_ch2;
        std::string str_val_ch3;
        std::string str_val_ch4;
        std::string cmd_msg_send;
        std::string cmd_msg_read;

        getIntegerParam(param1, &val_ch1);
        getIntegerParam(param2, &val_ch2);
        getIntegerParam(param3, &val_ch3);
        getIntegerParam(param4, &val_ch4); 
        
        std::cout << "val1: " << val_ch1 << std::endl;
        std::cout << "val2: " << val_ch2 << std::endl;
        std::cout << "val3: " << val_ch3 << std::endl;
        std::cout << "val4: " << val_ch4 << std::endl;

        switch (channel)
        {
            case 1:
            str_val_ch1 = std::__cxx11::to_string(val);
            str_val_ch2 = std::__cxx11::to_string(val_ch2);
            str_val_ch3 = std::__cxx11::to_string(val_ch3);
            str_val_ch4 = std::__cxx11::to_string(val_ch4);
            break;

            case 2:
            str_val_ch1 = std::__cxx11::to_string(val_ch1);
            str_val_ch2 = std::__cxx11::to_string(val);
            str_val_ch3 = std::__cxx11::to_string(val_ch3);
            str_val_ch4 = std::__cxx11::to_string(val_ch4);
            break;
            
            case 3:
            str_val_ch1 = std::__cxx11::to_string(val_ch1);
            str_val_ch2 = std::__cxx11::to_string(val_ch2);
            str_val_ch3 = std::__cxx11::to_string(val);
            str_val_ch4 = std::__cxx11::to_string(val_ch4);
            break;

            case 4:
            str_val_ch1 = std::__cxx11::to_string(val_ch1);
            str_val_ch2 = std::__cxx11::to_string(val_ch2);
            str_val_ch3 = std::__cxx11::to_string(val_ch3);
            str_val_ch4 = std::__cxx11::to_string(val);
            break;
        }

        if (to_string){
                str_val_ch1 = "N";

            if (str_val_ch2.find("1") == 0)
                str_val_ch2 = "P";
            else
                str_val_ch2 = "N";

            if (str_val_ch3.find("1") == 0)
                str_val_ch3 = "P";
            else
                str_val_ch3 = "N";

            if (str_val_ch4.find("1") == 0)
                str_val_ch4 = "P";
            else
                str_val_ch4 = "N";
        }

        cmd_msg_send = command_set + str_val_ch1 + " " + str_val_ch2 + " "\
                                        + str_val_ch3 + " " + str_val_ch4 + " ";
        send_to_equipment(cmd_msg_send.c_str());
        cmd_msg_read = command_ask;
        res = get_parsed_response(send_to_equipment(cmd_msg_read.c_str()), channel);
        std::cout << "my res is: " << res << std::endl;
        return res;
}

double c400drv::set_2_vals(const char *command_set, const char *command_ask, int param1, int param2, 
                            int channel, double val, int is_float, int to_string)
{
        asynStatus status = asynSuccess;
        double res;
        double val_ch1;
        double val_ch2;
        std::string str_val_ch1;
        std::string str_val_ch2;
        std::string cmd_msg_send;
        std::string cmd_msg_read;

        // Check if the value to feed is a float or integer and handles it
        if (is_float){
            getDoubleParam(param1, &val_ch1);
            getDoubleParam(param2, &val_ch2);
        }

        else{
            int val_ch1;
            int val_ch2;
            int val_ch3;
            int val_ch4;
            getIntegerParam(param1, &val_ch1);
            getIntegerParam(param2, &val_ch2);
        }

        switch (channel)
        {
            case 1:
            str_val_ch1 = std::__cxx11::to_string(val);
            str_val_ch2 = std::__cxx11::to_string(val_ch2);
            break;

            case 2:
            str_val_ch1 = std::__cxx11::to_string(val_ch1);
            str_val_ch2 = std::__cxx11::to_string(val);
            break;
        }

        if (to_string){
            if (str_val_ch1.find("1") == 0)
                str_val_ch1 = "P";
            else
                str_val_ch1 = "N";

            if (str_val_ch2.find("1") == 0)
                str_val_ch2 = "P";
            else
                str_val_ch2 = "N";
        }

        cmd_msg_send = command_set + str_val_ch1 + " " + str_val_ch2;
        send_to_equipment(cmd_msg_send.c_str());
        cmd_msg_read = command_ask;

        if (is_float)
            res = get_parsed_response(send_to_equipment(cmd_msg_read.c_str()), channel);
        else
            res = get_parsed_response(send_to_equipment(cmd_msg_read.c_str()), channel);
        std::cout << "my res is: " << res << std::endl;
        return res;
}

void c400drv::get_n_set_4_channels(const char *command_ask, int param1, int param2, int param3, int param4, 
                                   int n_param1, int n_param2, int n_param3, int n_param4)
{
    std::string token;
    double result_array[15];
    std::string val;
    std::string delimiter = ",";
    // std::string cmd_msg_read;
    double ch1_val;
    double ch2_val;
    double ch3_val;
    double ch4_val;

    size_t pos = 0;

    val = send_to_equipment(command_ask);
    parse_counts(result_array, val);

    ch1_val = result_array[1];
    ch2_val = result_array[2];
    ch3_val = result_array[3];
    ch4_val = result_array[4];

    std::cout << "ch1: " << ch1_val << std::endl;
    std::cout << "ch2: " << ch2_val << std::endl;
    std::cout << "ch3: " << ch3_val << std::endl;
    std::cout << "ch4: " << ch4_val << std::endl;
    
    setDoubleParam (param1,      ch1_val);
    setDoubleParam (param2,      ch2_val);
    setDoubleParam (param3,      ch3_val);
    setDoubleParam (param4,      ch4_val);
}

void c400drv::parse_counts(double *result_array, std::string val)
{
    std::string delimiter = ",";
    size_t pos = 0;
    std::string token;
    std::string last_char;

    token = val.substr(0, val.find("\n")+1);
    val = val.substr(token.length(), val.length() - token.length());

    // result_array[0] = token;
    int array_idx = 0;
    while ((pos = val.find(delimiter)) != std::string::npos) {
        token = val.substr(0, pos);
        last_char = token[token.length() - 1];
        if (last_char == "S" or last_char == "V"){
            token = token.substr(0, token.length() - 2);
        }
        try
        {
            result_array[array_idx] = std::stod(token);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
        
        val.erase(0, pos + delimiter.length());
        
        array_idx++;
    }
        try
    {
        result_array[array_idx] = std::stod(val); //Append the last val
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    

}

void c400drv::set_mbbo(const char *command_set, const std::string *mbbo_list, int mbbo_value)
{
    std::string cmd_msg_send;
    std::cout << mbbo_list[mbbo_value] << std::endl;

    cmd_msg_send = command_set + mbbo_list[mbbo_value];
    send_to_equipment(cmd_msg_send.c_str());
}

void c400drv::update_buffer(int n_elements){
    int number_of_lines_in_buffer_message = 14;
    int n_args = 5;
    int p_data_array_pos = 0; // Position in the waveform array to be updated in the loop
    std::string val;
    double temp_array[20]; 
    int buffer_size;
    int counter = 0;
    std::string base_msg = "FETch:COUNts? ";
    std::string get_full_buffer_str;

    getIntegerParam (P_BUFFER,      &buffer_size);
    get_full_buffer_str = base_msg + std::__cxx11::to_string(buffer_size);
    // Need to parse several lines, each one ending with "\r\n"
    // The equipment also return a blank line at the end, wich means that the last line is \r\n\r\n
    // So we can use this to tell that this is the last line and get all previous ones as well
    // the problem is that asyn matches any \r\n and stop listening to the equipment
    // Soultion: Use the middle pattern that appears in the string \r\n\r\n, which is \n\r
    // Return to the standard input eos afterwards 


    int number_of_needed_buffer_reads = (int) ceil( (double) buffer_size/number_of_lines_in_buffer_message);
    std::cout <<  "calc " << buffer_size/number_of_lines_in_buffer_message << std::endl;
    std::cout <<  "Number of it " << number_of_needed_buffer_reads << std::endl;
    for (int j=0; j<number_of_needed_buffer_reads; j++){
        
        pasynOctetSyncIO->setInputEos(pasynUserEcho, "\n\r", strlen("\n\r"));
        val = send_to_equipment(get_full_buffer_str.c_str());
        pasynOctetSyncIO->setInputEos(pasynUserEcho, "\r\n", strlen("\r\n"));
        std::cout << "value gotten is " << val << std::endl;
        if (val == ""){
            std::cout << "Data not yet collected" << std::endl;
            return;
        }

        std::istringstream iss(val);
        for (std::string line; std::getline(iss, line); ){
            counter ++;
            if (counter == 1){
                continue; // Skip first line
            }

            std::cout << line << std::endl;
            parse_counts(temp_array, line);
            for (int i=1; i<=n_args; i++){
                std::cout << "Pdata pos " << p_data_array_pos << " Array data "  << pData_[p_data_array_pos] << std::endl;
                // pData_[p_data_array_pos] = temp_array[i];
                pData_[p_data_array_pos] = i;
                p_data_array_pos ++;
            }
        }
    }
    std::cout << "callback to array" << std::endl;
    doCallbacksFloat64Array(pData_, buffer_array_size,  P_READ_BUFFER, 0);
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
#define buffer_array_size 1000*8
