/*____________________________________________________________________________________________________________________________________
|   Driver support for Arinax Luciole Cold Light Source developed with AsynPortDriver module from asyn/SynApps                    |
|   Brazilian Synchroton Light National Laboratory - Campinas, 05/21/2019                                                         |
|   Author: Allan S. B. Bugyi   (allan.bugyi@lnls.br)                                         |
|   Version: 1.0                                                              |
|   Tested                                                                                                                        |
|                                                                                                                                     |
|       License:                                                                                                                      |
|        This software is distributed under the following ISC license:                                                                |
|                                                                                                                                     |
|        Copyright © 2019 BRAZILIAN SYNCHROTRON LIGHT SOURCE <sol@lnls.br>                                                            |
|                                                                                                                                     |
|        Permission to use, copy, modify, and/or distribute this software for any                                                     |
|        purpose with or without fee is hereby granted, provided that the above                                                       |
|        copyright notice and this permission notice appear in all copies.                                                            |
|                                                                                                                                     |
|        THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES                                                     |
|        WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF                                                             |
|        MERCHANTABILITY AND FITNESS.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR                                                     |
|        ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES                                                       |
|        WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION                                                 |
|        OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN                                                       |
|        CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                                                     |
|_____________________________________________________________________________________________________________________________________|*/


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

#define C400_MSG_HEADER  "CONFigure:DHI?"



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
    createParam(P_VoltString1, asynParamFloat64, &P_Volt1);
    createParam(P_VoltString2, asynParamFloat64, &P_Volt2);
    createParam(P_VoltString3, asynParamFloat64, &P_Volt3);
    createParam(P_VoltString4, asynParamFloat64, &P_Volt4);

    setDoubleParam (P_Volt1,          1.0);
    setDoubleParam (P_Volt2,          2.0);
    setDoubleParam (P_Volt3,          3.0);
    setDoubleParam (P_Volt4,          4.0);

    pasynOctetSyncIO->connect(ip, 0, &pasynUserEcho, NULL);
    pasynOctetSyncIO->setInputEos(pasynUserEcho, "\r\n", strlen("\r\n"));
    pasynOctetSyncIO->setOutputEos(pasynUserEcho, "\n", strlen("\n"));
    
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
    
    if (function == P_Volt1) {
        result = set_voltage(1, value);
        setDoubleParam (P_Volt1,      result);
        std::cout << "my res is: " << result << std::endl;
        
    }
    else if (function == P_Volt2){
        result = set_voltage(2, value);
        setDoubleParam (P_Volt2,      result);
        std::cout << "my res is: " << result << std::endl;
    }
    else if (function == P_Volt3){
        result = set_voltage(3, value);
        setDoubleParam (P_Volt3,      result);
        std::cout << "value: " << value << std::endl;
        std::cout << "result: " << result << std::endl;
    }
    else if (function == P_Volt4){
        result = set_voltage(4, value);
        setDoubleParam (P_Volt4,      result);
        std::cout << "my res is: " << result << std::endl;
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

double c400drv::set_voltage(int channel, double val)
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

        cmd_msg_read = C400_MSG_HEADER;
        getDoubleParam(P_Volt1, &val_ch1);
        getDoubleParam(P_Volt2, &val_ch2);
        getDoubleParam(P_Volt3, &val_ch3);
        getDoubleParam(P_Volt4, &val_ch4);
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


        cmd_msg_send = "CONFigure:DHI " + str_val_ch1 + " " + str_val_ch2 + " "\
                                        + str_val_ch3 + " " + str_val_ch4 + " ";
        // std::cout << "sent string: " << cmd_msg_send << std::endl;
        send_to_equipment(cmd_msg_send.c_str());
        cmd_msg_read = "CONFigure:DHI?";
        res = get_channel_val(send_to_equipment(cmd_msg_read.c_str()), channel);
        std::cout << "my res is2222: " << res << std::endl;
        return res;
        
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
