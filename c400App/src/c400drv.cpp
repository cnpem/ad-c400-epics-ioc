#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <iostream>
#include <unistd.h>

//EPICS's includes
#include <iocsh.h>
#include <epicsExport.h>
#include <registryFunction.h>
#include <epicsTypes.h>
#include <epicsString.h>
#include <epicsMutex.h>
#include <epicsEvent.h>

//AsynPortDriver's includes
#include <asynPortDriver.h>
#include "asynOctetSyncIO.h"

//Luciole's includes
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


static const char *driverName = "c400driver";


c400drv::c400drv(const char *portName, char *ip)
   : asynPortDriver(portName,
                    1, /* maxAddr */
                    asynInt32Mask | asynFloat64Mask | asynDrvUserMask, /* Interface mask */
                    asynInt32Mask | asynFloat64Mask | asynEnumMask,  /* Interrupt mask */
                    ASYN_CANBLOCK, /* asynFlags.  This driver does not block and it is not multi-device, so flag is 0 */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0) /* Default stack size*/
{
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

    pasynOctetSyncIO->connect(ip, 0, &pasynUserEcho, NULL);
    pasynOctetSyncIO->setInputEos(pasynUserEcho, "\r\n", strlen("\r\n"));
    pasynOctetSyncIO->setOutputEos(pasynUserEcho, "\n", strlen("\n"));

    sync_w_device();
    
}


//------------ asynPortDriver extended method ------------

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
        sleep(.25);
    }
    std::cout << "status: " << status << std::endl;
    std::cout << "Buffer: " << readBuffer << std::endl;
    return std::string (readBuffer);

}
float c400drv::get_channel_val(std::string val, int channel, std::string search_for, int size_sep)
{
    std::string parse = val;
    std::string str_now;
    int end;
    int query_val;
    int end_line = parse.find("\n", 0);

    str_now = parse.substr(end_line + 1);
    query_val = str_now.find(search_for, 0);
    for(int i = 1; i < channel; i++)
    {
        query_val = str_now.find(search_for, 0);
        if (search_for == " ")
            str_now = str_now.substr(query_val + size_sep);
        else if (search_for == ",")
            str_now = str_now.substr(query_val + 1);
    }

    if (channel==3 and search_for==" ")
        str_now = str_now.substr(0, query_val+1);
    else
        str_now = str_now.substr(0, query_val);

    std::cout << "str_now is " << str_now << std::endl;

    if (str_now.find("P") == 0)
        return 1;
    else if (str_now.find("N") == 0)
        return 0;

    try {
        return std::stof(str_now);
    }
    
    catch (const std::invalid_argument &e){
        std::cerr << "Error " << e.what() << std::endl;
        return 0;
    }
}

double c400drv::set_direct(const char *command_set, const char *command_ask, int channel, double val)
{
    // If 0 is passed to channel, them no specific channel is set
    double res;
    std::string str_channel;
    std::string str_val;
    std::string cmd_msg_send;
    std::string cmd_msg_read;
    str_channel = std::__cxx11::to_string(channel);
    str_val = std::__cxx11::to_string(val);

    if (channel){
        cmd_msg_send =  command_set + str_channel + " " + str_val;
        send_to_equipment(cmd_msg_send.c_str());
        cmd_msg_read = command_ask;
        res = get_channel_val(send_to_equipment(cmd_msg_read.c_str()), channel);
        return res;
    }

    else{
        cmd_msg_send =  command_set + str_val;
        send_to_equipment(cmd_msg_send.c_str());
        return 0;
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


        res = get_channel_val(send_to_equipment(cmd_msg_read.c_str()), channel);

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
        res = get_channel_val(send_to_equipment(cmd_msg_read.c_str()), channel, ",");
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
            res = get_channel_val(send_to_equipment(cmd_msg_read.c_str()), channel, " ", 4);
        else
            res = get_channel_val(send_to_equipment(cmd_msg_read.c_str()), channel, ",", 4);
        std::cout << "my res is: " << res << std::endl;
        return res;
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
    res_DAC_ch1 = get_channel_val(send_to_equipment(C400_MSG_DAC_ASK), 1);
    setDoubleParam (P_DAC1,          res_DAC_ch1);
    res_DAC_ch2 = get_channel_val(send_to_equipment(C400_MSG_DAC_ASK), 2);
    setDoubleParam (P_DAC2,          res_DAC_ch2);
    res_DAC_ch3 = get_channel_val(send_to_equipment(C400_MSG_DAC_ASK), 3);
    setDoubleParam (P_DAC3,          res_DAC_ch3);
    res_DAC_ch4 = get_channel_val(send_to_equipment(C400_MSG_DAC_ASK), 4);
    setDoubleParam (P_DAC4,          res_DAC_ch4);

    // Get DEAD time
    res_DEAD = get_channel_val(send_to_equipment(C400_MSG_DEAD_ASK), 1);
    setDoubleParam (P_DEAD,          res_DEAD);

    // Get DHI val
    res_DHI_ch1 = get_channel_val(send_to_equipment(C400_MSG_DHI_ASK), 1);
    setDoubleParam (P_DHI1,          res_DHI_ch1);
    res_DHI_ch2 = get_channel_val(send_to_equipment(C400_MSG_DHI_ASK), 2);
    setDoubleParam (P_DHI2,          res_DHI_ch2);
    res_DHI_ch3 = get_channel_val(send_to_equipment(C400_MSG_DHI_ASK), 3);
    setDoubleParam (P_DHI3,          res_DHI_ch3);
    res_DHI_ch4 = get_channel_val(send_to_equipment(C400_MSG_DHI_ASK), 4);
    setDoubleParam (P_DHI4,          res_DHI_ch4);

    // Get DLO val
    res_DLO_ch1 = get_channel_val(send_to_equipment(C400_MSG_DLO_ASK), 1);
    setDoubleParam (P_DLO1,          res_DLO_ch1);
    res_DLO_ch2 = get_channel_val(send_to_equipment(C400_MSG_DLO_ASK), 2);
    setDoubleParam (P_DLO2,          res_DLO_ch2);
    res_DLO_ch3 = get_channel_val(send_to_equipment(C400_MSG_DLO_ASK), 3);
    setDoubleParam (P_DLO3,          res_DLO_ch3);
    res_DLO_ch4 = get_channel_val(send_to_equipment(C400_MSG_DLO_ASK), 4);
    setDoubleParam (P_DLO4,          res_DLO_ch4);

    // Get HIVO_VOLTS val
    res_HIVO_VOLTS_ch1 = get_channel_val(send_to_equipment(C400_MSG_HIVO_VOLTS_ASK), 1);
    setDoubleParam (P_HIVO_VOLTS1,          res_HIVO_VOLTS_ch1);
    res_HIVO_VOLTS_ch2 = get_channel_val(send_to_equipment(C400_MSG_HIVO_VOLTS_ASK), 2);
    setDoubleParam (P_HIVO_VOLTS2,          res_HIVO_VOLTS_ch2);
    res_HIVO_VOLTS_ch3 = get_channel_val(send_to_equipment(C400_MSG_HIVO_VOLTS_ASK), 3);
    setDoubleParam (P_HIVO_VOLTS3,          res_HIVO_VOLTS_ch3);
    res_HIVO_VOLTS_ch4 = get_channel_val(send_to_equipment(C400_MSG_HIVO_VOLTS_ASK), 4);
    setDoubleParam (P_HIVO_VOLTS4,          res_HIVO_VOLTS_ch4);

    // Get HIVO_ENABLE val
    res_HIVO_ENABLE_ch1 = get_channel_val(send_to_equipment(C400_MSG_HIVO_ENABLE_ASK), 1, ",");
    setIntegerParam (P_HIVO_ENABLE1,          res_HIVO_ENABLE_ch1);
    res_HIVO_ENABLE_ch2 = get_channel_val(send_to_equipment(C400_MSG_HIVO_ENABLE_ASK), 2, ",");
    setIntegerParam (P_HIVO_ENABLE2,          res_HIVO_ENABLE_ch2);
    res_HIVO_ENABLE_ch3 = get_channel_val(send_to_equipment(C400_MSG_HIVO_ENABLE_ASK), 3, ",");
    setIntegerParam (P_HIVO_ENABLE3,          res_HIVO_ENABLE_ch3);
    res_HIVO_ENABLE_ch4 = get_channel_val(send_to_equipment(C400_MSG_HIVO_ENABLE_ASK), 4, ",");
    setIntegerParam (P_HIVO_ENABLE4,          res_HIVO_ENABLE_ch4);

    // Get PERIOD time
    res_PERIOD = get_channel_val(send_to_equipment(C400_MSG_PERIOD_ASK), 1);
    setDoubleParam (P_PERIOD,          res_PERIOD);

    // Get HIVO Polarity val
    res_POLARITY_ch1 = get_channel_val(send_to_equipment(C400_MSG_POLARITY_ASK), 1, ",");
    setIntegerParam (P_POLARITY1,          res_POLARITY_ch1);
    res_POLARITY_ch2 = get_channel_val(send_to_equipment(C400_MSG_POLARITY_ASK), 2, ",");
    setIntegerParam (P_POLARITY2,          res_POLARITY_ch2);
    res_POLARITY_ch3 = get_channel_val(send_to_equipment(C400_MSG_POLARITY_ASK), 3, ",");
    setIntegerParam (P_POLARITY3,          res_POLARITY_ch3);
    res_POLARITY_ch4 = get_channel_val(send_to_equipment(C400_MSG_POLARITY_ASK), 4, ",");
    setIntegerParam (P_POLARITY4,          res_POLARITY_ch4);

    // Get PULSER vals
    res_PULSER_Period = get_channel_val(send_to_equipment(C400_MSG_PULSER_ASK), 1, " ", 4);
    setDoubleParam (P_PULSER_Period,          res_PULSER_Period);
    res_PULSER_Width = get_channel_val(send_to_equipment(C400_MSG_PULSER_ASK), 2, " ", 4);
    setDoubleParam (P_PULSER_Width,          res_PULSER_Width);

    // Get BUFFER size
    res_BUFFER = get_channel_val(send_to_equipment(C400_MSG_BUFFER_ASK), 1);
    setDoubleParam (P_BUFFER,          res_BUFFER);

    // Get BURST size
    res_BURST = get_channel_val(send_to_equipment(C400_MSG_BURST_ASK), 1);
    setDoubleParam (P_BURST,          res_BURST);
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
