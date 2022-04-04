#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <iostream>

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

static const char *driverName = "c400driver";


c400drv::c400drv(const char *portName, char *ip)
   : asynPortDriver(portName,
                    1, /* maxAddr */
                    asynFloat64Mask | asynDrvUserMask, /* Interface mask */
                    asynFloat64Mask | asynEnumMask,  /* Interrupt mask */
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


    pasynOctetSyncIO->connect(ip, 0, &pasynUserEcho, NULL);
    pasynOctetSyncIO->setInputEos(pasynUserEcho, "\r\n", strlen("\r\n"));
    pasynOctetSyncIO->setOutputEos(pasynUserEcho, "\n", strlen("\n"));

    sync_w_device();
    
}


//------------ asynPortDriver extended method ------------
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
    status = pasynOctetSyncIO->writeRead(pasynUserEcho, writeBuffer, strlen(writeBuffer), readBuffer,
                                     sizeof(readBuffer), TIMEOUT, &nActual, &nRead, &eomReason);
    std::cout << "status: " << status << std::endl;
    std::cout << "Buffer: " << readBuffer << std::endl;
    return std::string (readBuffer);

}
float c400drv::get_channel_val(std::string val, int channel)
{
    std::string parse = val;
    std::string str_now;
    int end;
    int query_val;
    int end_line = parse.find("\n", 0);

    str_now = parse.substr(end_line);
    query_val = str_now.find(" ", 0);
    for(int i = 1; i < channel; i++)
    {
        query_val = str_now.find(" ", 0);
        str_now = str_now.substr(query_val + 3);
        // std::cout << str_now << std::endl;
    }

    str_now = str_now.substr(0, query_val);
    // std::cout << str_now << std::endl;
    return std::stof(str_now);
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

    // std::cout << "estoy vivo" << std::endl;
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

double c400drv::set_4_channels(const char *command_set, const char *command_ask, int param1, 
                                int param2, int param3, int param4, int channel, double val)
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

        getDoubleParam(param1, &val_ch1);
        getDoubleParam(param2, &val_ch2);
        getDoubleParam(param3, &val_ch3);
        getDoubleParam(param4, &val_ch4);

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
        std::cout << "my res is2222: " << res << std::endl;
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

    // Get DHI val
    res_DLO_ch1 = get_channel_val(send_to_equipment(C400_MSG_DLO_ASK), 1);
    setDoubleParam (P_DLO1,          res_DLO_ch1);
    res_DLO_ch2 = get_channel_val(send_to_equipment(C400_MSG_DLO_ASK), 2);
    setDoubleParam (P_DLO2,          res_DLO_ch2);
    res_DLO_ch3 = get_channel_val(send_to_equipment(C400_MSG_DLO_ASK), 3);
    setDoubleParam (P_DLO3,          res_DLO_ch3);
    res_DLO_ch4 = get_channel_val(send_to_equipment(C400_MSG_DLO_ASK), 4);
    setDoubleParam (P_DLO4,          res_DLO_ch4);

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
