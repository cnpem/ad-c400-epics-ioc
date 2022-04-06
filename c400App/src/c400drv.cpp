#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <iostream>
#include <unistd.h>

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
#include <asynPortDriver.h>
#include "asynOctetSyncIO.h"

//c400's includes
#include "c400drv.h"
#include <epicsExport.h>

#define TIMEOUT 1.0
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

void update_counts(void *drvPvt);
static const char *driverName = "c400driver";
static std::string trigger_mode_mbbo[]={"CUSTom", "INTernal", "EXTERNAL_START", "EXTERNAL_START_STOP",
                                        "EXTERNAL_START_HOLD", "EXTERNAL_WINDOWED", "DISCRIMINATOR_SWEEP"};

c400drv::c400drv(const char *portName, char *ip)
   : asynPortDriver(portName,
                    1, /* maxAddr */
                    asynInt32Mask | asynFloat64Mask | asynEnumMask | asynDrvUserMask, /* Interface mask */
                    asynInt32Mask | asynFloat64Mask | asynEnumMask,  /* Interrupt mask */
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
    createParam(P_BUFFERString, asynParamFloat64, &P_BUFFER);
    createParam(P_BURSTString, asynParamFloat64, &P_BURST);
    createParam(P_COUNT1String, asynParamFloat64, &P_COUNT1);
    createParam(P_COUNT2String, asynParamFloat64, &P_COUNT2);
    createParam(P_COUNT3String, asynParamFloat64, &P_COUNT3);
    createParam(P_COUNT4String, asynParamFloat64, &P_COUNT4);
    createParam(P_TRIGGER_MODEString, asynParamInt32, &P_TRIGGER_MODE);
    createParam(P_TRIGGER_POLARITYString, asynParamInt32, &P_TRIGGER_POLARITY);
    createParam(P_TRIGGER_STARTString, asynParamInt32, &P_TRIGGER_START);
    createParam(P_TRIGGER_STOPString, asynParamInt32, &P_TRIGGER_STOP);

    pasynOctetSyncIO->connect(ip, 0, &pasynUserEcho, NULL);
    pasynOctetSyncIO->setInputEos(pasynUserEcho, "\r\n", strlen("\r\n"));
    pasynOctetSyncIO->setOutputEos(pasynUserEcho, "\n", strlen("\n"));

    sync_w_device();

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

    while (true){
        getIntegerParam(P_ACQUIRE, &is_counting);
        getDoubleParam(P_PERIOD, &sleep_for);
        if (is_counting){
            get_n_set_4_channels(C400_MSG_COUNTS_ASK, P_COUNT1, P_COUNT2, 
                                P_COUNT3, P_COUNT4, 2,3,4,5);
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
    /* Set the parameter in the parameter library. */
    status = (asynStatus) setIntegerParam(function, value);

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

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
        setIntegerParam (P_POLARITY4,      value);
    }
    else if (function == P_TRIGGER_MODE){
        set_mbbo(C400_MSG_TRIGGER_MODE_SET, trigger_mode_mbbo, value);
        setIntegerParam (P_TRIGGER_MODE,      value);
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
    std::cout << "Write" << std::endl;
    const char *paramName;
    const char* functionName = "writeFloat64";
    double result;
    
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
    else if (function == P_BUFFER){
        result = set_direct(C400_MSG_BUFFER_SET, C400_MSG_BUFFER_ASK, 0, value);
        setDoubleParam (P_BUFFER,      value);
    }
    else if (function == P_BURST){
        result = set_direct(C400_MSG_BURST_SET, C400_MSG_BURST_ASK, 0, value);
        setDoubleParam (P_BURST,      value);
    }

    status = (asynStatus) callParamCallbacks();

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


std::string c400drv::send_to_equipment(const char *writeBuffer)
{
    asynStatus status = asynSuccess;
    size_t nRead, nActual;
    int eomReason;
    double readValue;    
    char readBuffer[100];
    while (true){
        status = pasynOctetSyncIO->writeRead(pasynUserEcho, writeBuffer, strlen(writeBuffer), readBuffer,
                                     sizeof(readBuffer), TIMEOUT, &nActual, &nRead, &eomReason);
        if (status == 0)
            break;
        sleep(.1);
    }
    std::cout << "status: " << status << std::endl;
    std::cout << "Buffer: " << readBuffer << std::endl;
    return std::string (readBuffer);

}
float c400drv::get_parsed_response(std::string val, int n_element, std::string delimiter)
{
    size_t pos = 0;
    std::string token;
    std::string result_array[15];
    token = val.substr(0, val.find("\n")+1);
    val = val.substr(token.length(), val.length() - token.length());
    float res;

    result_array[0] = token;
    int array_idx = 1;
    while ((pos = val.find(delimiter)) != std::string::npos) {
        token = val.substr(0, pos);
        result_array[array_idx] = token;
        val.erase(0, pos + delimiter.length());
        array_idx++;
    }
    result_array[array_idx] = val; //Append the last val
    if (result_array[n_element] == "P"){
        std::cout << "Got: " << "P" << std::endl;
        return 1;
    }
    else if (result_array[n_element] == "N"){
        std::cout << "Got: " << "N" << std::endl;
        return 0;
    }

    try{
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
        if (command_set==C400_MSG_TRIGGER_START_SET or command_set==C400_MSG_TRIGGER_STOP_SET){
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
            if (str_val_ch1.find("1") == 0)
                str_val_ch1 = "P";
            else
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
    std::string result_array[15];
    std::string val;
    std::string delimiter = ",";
    // std::string cmd_msg_read;
    double ch1_val;
    double ch2_val;
    double ch3_val;
    double ch4_val;

    size_t pos = 0;

    val = send_to_equipment(command_ask);

    token = val.substr(0, val.find("\n")+1);
    val = val.substr(token.length(), val.length() - token.length());

    result_array[0] = token;
    int array_idx = 1;
    while ((pos = val.find(delimiter)) != std::string::npos) {
        token = val.substr(0, pos);
        result_array[array_idx] = token;
        val.erase(0, pos + delimiter.length());
        array_idx++;
    }
    result_array[array_idx] = val; //Append the last val

    try {
        ch1_val = std::stof(result_array[n_param1]);
        ch2_val = std::stof(result_array[n_param2]);
        ch3_val = std::stof(result_array[n_param3]);
        ch4_val = std::stof(result_array[n_param4]);
    }

    catch (...){
        getDoubleParam (param1,      &ch1_val);
        getDoubleParam (param2,      &ch2_val);
        getDoubleParam (param3,      &ch3_val);
        getDoubleParam (param4,      &ch4_val);;
    }

    std::cout << "ch1: " << ch1_val << std::endl;
    std::cout << "ch2: " << ch2_val << std::endl;
    std::cout << "ch3: " << ch3_val << std::endl;
    std::cout << "ch4: " << ch4_val << std::endl;
    setDoubleParam (param1,      ch1_val);
    setDoubleParam (param2,      ch2_val);
    setDoubleParam (param3,      ch3_val);
    setDoubleParam (param4,      ch4_val);
    callParamCallbacks();
}

void c400drv::set_mbbo(const char *command_set, const std::string *mbbo_list, int mbbo_value)
{
    std::string cmd_msg_send;
    std::cout << mbbo_list[mbbo_value] << std::endl;

    cmd_msg_send = command_set + mbbo_list[mbbo_value];
    send_to_equipment(cmd_msg_send.c_str());
}

void c400drv::sync_w_device()
{
    double res_DAC_ch1;
    double res_DAC_ch2;
    double res_DAC_ch3;
    double res_DAC_ch4;
    double res_DEAD;
    double res_DHI_ch1;
    double res_DHI_ch2;
    double res_DHI_ch3;
    double res_DHI_ch4;
    double res_DLO_ch1;
    double res_DLO_ch2;
    double res_DLO_ch3;
    double res_DLO_ch4;
    double res_HIVO_VOLTS_ch1;
    double res_HIVO_VOLTS_ch2;
    double res_HIVO_VOLTS_ch3;
    double res_HIVO_VOLTS_ch4;
    double res_HIVO_ENABLE_ch1;
    double res_HIVO_ENABLE_ch2;
    double res_HIVO_ENABLE_ch3;
    double res_HIVO_ENABLE_ch4;
    double res_PERIOD;
    double res_POLARITY_ch1;
    double res_POLARITY_ch2;
    double res_POLARITY_ch3;
    double res_POLARITY_ch4;
    double res_PULSER_Period;
    double res_PULSER_Width;
    double res_BUFFER;
    double res_BURST;

    // Get DAC val
    // res_DAC_ch1 = get_parsed_response(send_to_equipment(C400_MSG_DAC_ASK), 1);
    // setDoubleParam (P_DAC1,          res_DAC_ch1);
    // res_DAC_ch2 = get_parsed_response(send_to_equipment(C400_MSG_DAC_ASK), 2);
    // setDoubleParam (P_DAC2,          res_DAC_ch2);
    // res_DAC_ch3 = get_parsed_response(send_to_equipment(C400_MSG_DAC_ASK), 3);
    // setDoubleParam (P_DAC3,          res_DAC_ch3);
    // res_DAC_ch4 = get_parsed_response(send_to_equipment(C400_MSG_DAC_ASK), 4);
    // setDoubleParam (P_DAC4,          res_DAC_ch4);
    get_n_set_4_channels(C400_MSG_DAC_ASK, P_DAC1, P_DAC2, 
                        P_DAC3, P_DAC4, 1,2,3,4);

    // Get DEAD time
    res_DEAD = get_parsed_response(send_to_equipment(C400_MSG_DEAD_ASK), 1);
    setDoubleParam (P_DEAD,          res_DEAD);

    // Get DHI val
    // res_DHI_ch1 = get_parsed_response(send_to_equipment(C400_MSG_DHI_ASK), 1);
    // setDoubleParam (P_DHI1,          res_DHI_ch1);
    // res_DHI_ch2 = get_parsed_response(send_to_equipment(C400_MSG_DHI_ASK), 2);
    // setDoubleParam (P_DHI2,          res_DHI_ch2);
    // res_DHI_ch3 = get_parsed_response(send_to_equipment(C400_MSG_DHI_ASK), 3);
    // setDoubleParam (P_DHI3,          res_DHI_ch3);
    // res_DHI_ch4 = get_parsed_response(send_to_equipment(C400_MSG_DHI_ASK), 4);
    // setDoubleParam (P_DHI4,          res_DHI_ch4);
    get_n_set_4_channels(C400_MSG_DHI_ASK, P_DHI1, P_DHI2, 
                        P_DHI3, P_DHI4, 1,2,3,4);

    // Get DLO val
    // res_DLO_ch1 = get_parsed_response(send_to_equipment(C400_MSG_DLO_ASK), 1);
    // setDoubleParam (P_DLO1,          res_DLO_ch1);
    // res_DLO_ch2 = get_parsed_response(send_to_equipment(C400_MSG_DLO_ASK), 2);
    // setDoubleParam (P_DLO2,          res_DLO_ch2);
    // res_DLO_ch3 = get_parsed_response(send_to_equipment(C400_MSG_DLO_ASK), 3);
    // setDoubleParam (P_DLO3,          res_DLO_ch3);
    // res_DLO_ch4 = get_parsed_response(send_to_equipment(C400_MSG_DLO_ASK), 4);
    // setDoubleParam (P_DLO4,          res_DLO_ch4);
    get_n_set_4_channels(C400_MSG_DLO_ASK, P_DLO1, P_DLO2, 
                        P_DLO3, P_DLO4, 1,2,3,4);


    // Get HIVO_VOLTS val
    // res_HIVO_VOLTS_ch1 = get_parsed_response(send_to_equipment(C400_MSG_HIVO_VOLTS_ASK), 1);
    // setDoubleParam (P_HIVO_VOLTS1,          res_HIVO_VOLTS_ch1);
    // res_HIVO_VOLTS_ch2 = get_parsed_response(send_to_equipment(C400_MSG_HIVO_VOLTS_ASK), 2);
    // setDoubleParam (P_HIVO_VOLTS2,          res_HIVO_VOLTS_ch2);
    // res_HIVO_VOLTS_ch3 = get_parsed_response(send_to_equipment(C400_MSG_HIVO_VOLTS_ASK), 3);
    // setDoubleParam (P_HIVO_VOLTS3,          res_HIVO_VOLTS_ch3);
    // res_HIVO_VOLTS_ch4 = get_parsed_response(send_to_equipment(C400_MSG_HIVO_VOLTS_ASK), 4);
    // setDoubleParam (P_HIVO_VOLTS4,          res_HIVO_VOLTS_ch4);
    get_n_set_4_channels(C400_MSG_HIVO_VOLTS_ASK, P_HIVO_VOLTS1, P_HIVO_VOLTS2, 
                        P_HIVO_VOLTS3, P_HIVO_VOLTS4, 1,2,3,4);

    // Get HIVO_ENABLE val
    res_HIVO_ENABLE_ch1 = get_parsed_response(send_to_equipment(C400_MSG_HIVO_ENABLE_ASK), 1);
    setIntegerParam (P_HIVO_ENABLE1,          res_HIVO_ENABLE_ch1);
    res_HIVO_ENABLE_ch2 = get_parsed_response(send_to_equipment(C400_MSG_HIVO_ENABLE_ASK), 2);
    setIntegerParam (P_HIVO_ENABLE2,          res_HIVO_ENABLE_ch2);
    res_HIVO_ENABLE_ch3 = get_parsed_response(send_to_equipment(C400_MSG_HIVO_ENABLE_ASK), 3);
    setIntegerParam (P_HIVO_ENABLE3,          res_HIVO_ENABLE_ch3);
    res_HIVO_ENABLE_ch4 = get_parsed_response(send_to_equipment(C400_MSG_HIVO_ENABLE_ASK), 4);
    setIntegerParam (P_HIVO_ENABLE4,          res_HIVO_ENABLE_ch4);
    // get_n_set_4_channels(C400_MSG_HIVO_ENABLE_ASK, P_HIVO_ENABLE1, P_HIVO_ENABLE2, 
    //                     P_HIVO_ENABLE3, P_HIVO_ENABLE4, 1,2,3,4);

    // Get PERIOD time
    res_PERIOD = get_parsed_response(send_to_equipment(C400_MSG_PERIOD_ASK), 1);
    setDoubleParam (P_PERIOD,          res_PERIOD);

    // Get HIVO Polarity val
    res_POLARITY_ch1 = get_parsed_response(send_to_equipment(C400_MSG_POLARITY_ASK), 1);
    setIntegerParam (P_POLARITY1,          res_POLARITY_ch1);
    res_POLARITY_ch2 = get_parsed_response(send_to_equipment(C400_MSG_POLARITY_ASK), 2);
    setIntegerParam (P_POLARITY2,          res_POLARITY_ch2);
    res_POLARITY_ch3 = get_parsed_response(send_to_equipment(C400_MSG_POLARITY_ASK), 3);
    setIntegerParam (P_POLARITY3,          res_POLARITY_ch3);
    res_POLARITY_ch4 = get_parsed_response(send_to_equipment(C400_MSG_POLARITY_ASK), 4);
    setIntegerParam (P_POLARITY4,          res_POLARITY_ch4);

    // Get PULSER vals
    res_PULSER_Period = get_parsed_response(send_to_equipment(C400_MSG_PULSER_ASK), 1);
    setDoubleParam (P_PULSER_Period,          res_PULSER_Period);
    res_PULSER_Width = get_parsed_response(send_to_equipment(C400_MSG_PULSER_ASK), 2);
    setDoubleParam (P_PULSER_Width,          res_PULSER_Width);

    // Get BUFFER size
    res_BUFFER = get_parsed_response(send_to_equipment(C400_MSG_BUFFER_ASK), 1);
    setDoubleParam (P_BUFFER,          res_BUFFER);

    // Get BURST size
    res_BURST = get_parsed_response(send_to_equipment(C400_MSG_BURST_ASK), 1);
    setDoubleParam (P_BURST,          res_BURST);

    //Set default TRIGGER MODE as internal
    set_mbbo(C400_MSG_TRIGGER_MODE_SET, trigger_mode_mbbo, 1); 
    setIntegerParam (P_TRIGGER_MODE,      1);
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
