#include "asynPortDriver.h"
#include <array>

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */

#define P_DACString1                    "DAC1"                /* asynFloat64,  r/w */
#define P_DACString2                    "DAC2"                /* asynFloat64,  r/w */
#define P_DACString3                    "DAC3"                /* asynFloat64,  r/w */
#define P_DACString4                    "DAC4"                /* asynFloat64,  r/w */
#define P_DEADString                    "DEAD"                /* asynFloat64,  r/w */
#define P_DHIString1                    "DHI1"                /* asynFloat64,  r/w */
#define P_DHIString2                    "DHI2"                /* asynFloat64,  r/w */
#define P_DHIString3                    "DHI3"                /* asynFloat64,  r/w */
#define P_DHIString4                    "DHI4"                /* asynFloat64,  r/w */
#define P_DLOString1                    "DLO1"                /* asynFloat64,  r/w */
#define P_DLOString2                    "DLO2"                /* asynFloat64,  r/w */
#define P_DLOString3                    "DLO3"                /* asynFloat64,  r/w */
#define P_DLOString4                    "DLO4"                /* asynFloat64,  r/w */
#define P_HIVO_VOLTSString1             "HIVO_VOLTS1"         /* asynFloat64,  r/w */
#define P_HIVO_VOLTSString2             "HIVO_VOLTS2"         /* asynFloat64,  r/w */
#define P_HIVO_VOLTSString3             "HIVO_VOLTS3"         /* asynFloat64,  r/w */
#define P_HIVO_VOLTSString4             "HIVO_VOLTS4"         /* asynFloat64,  r/w */
#define P_HIVO_ENABLEString1            "HIVO_ENABLE1"        /* asynInt32,    r/w */
#define P_HIVO_ENABLEString2            "HIVO_ENABLE2"        /* asynInt32,    r/w */
#define P_HIVO_ENABLEString3            "HIVO_ENABLE3"        /* asynInt32,    r/w */
#define P_HIVO_ENABLEString4            "HIVO_ENABLE4"        /* asynInt32,    r/w */
#define P_PERIODString                  "PERIOD"              /* asynFloat64,  r/w */
#define P_POLARITYString1               "POLARITY1"           /* asynInt32,    r/w */
#define P_POLARITYString2               "POLARITY2"           /* asynInt32,    r/w */
#define P_POLARITYString3               "POLARITY3"           /* asynInt32,    r/w */
#define P_POLARITYString4               "POLARITY4"           /* asynInt32,    r/w */
#define P_PULSER_PeriodString           "PULSER_Period"       /* asynFloat64,  r/w */
#define P_PULSER_WidthString            "PULSER_Width"        /* asynFloat64,  r/w */
#define P_ACQUIREString                 "ACQUIRE"             /* asynInt32,    r/w */
#define P_BUFFERString                  "BUFFER"              /* asynInt32,   r/w */
#define P_BURSTString                   "BURST"               /* asynFloat64,  r/w */
#define P_COUNT1String                  "COUNT1"              /* asynFloat64,  r/o */
#define P_COUNT2String                  "COUNT2"              /* asynFloat64,  r/o */
#define P_COUNT3String                  "COUNT3"              /* asynFloat64,  r/o */
#define P_COUNT4String                  "COUNT4"              /* asynFloat64,  r/o */
#define P_ENCODERString                 "ENCODER"             /* asynFloat64,  r/o */
#define P_TRIGGER_MODEString            "TRIGGER_MODE"        /* asynInt32,    r/w */
#define P_TRIGGER_POLARITYString        "TRIGGER_POLARITY"    /* asynInt32,    r/w */
#define P_TRIGGER_STARTString           "TRIGGER_START"       /* asynInt32,    r/w */
#define P_TRIGGER_STOPString            "TRIGGER_STOP"        /* asynInt32,    r/w */
#define P_TRIGGER_PAUSEString           "TRIGGER_PAUSE"       /* asynInt32,    r/w */
#define P_SYSTEM_IPMODEString           "SYSTEM_IPMODE"       /* asynInt32,    r/w */
#define P_READ_BUFFER1String            "READ_BUFFER1"         /* asynFloat64Array,  r/o */
#define P_READ_BUFFER2String            "READ_BUFFER2"         /* asynFloat64Array,  r/o */
#define P_READ_BUFFER3String            "READ_BUFFER3"         /* asynFloat64Array,  r/o */
#define P_READ_BUFFER4String            "READ_BUFFER4"         /* asynFloat64Array,  r/o */
#define P_READ_BUFFER_TIMEString        "READ_BUFFER_TIME"     /* asynFloat64Array,  r/o */

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
#define C400_MSG_ENCODER_ASK "FETch:ENCOder?"
#define C400_MSG_DIGITAL_ASK "FETch:DIGital?"
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

#define buffer_array_size 65536

static string trigger_mode_mbbo[]={"CUSTom", "INTernal", "EXTERNAL_START", "EXTERNAL_START_STOP",
                                        "EXTERNAL_START_HOLD", "EXTERNAL_WINDOWED", "DISCRIMINATOR_SWEEP"};
static string system_ipmode_mbbo[]={"DHCP", "Static"};

/** Class that demonstrates the use of the asynPortDriver base class to greatly simplify the task
  * of writing an asyn port driver.
  * This class does a simple simulation of a digital oscilloscope.  It computes a waveform, computes
  * statistics on the waveform, and does callbacks with the statistics and the waveform data itself.
  * I have made the methods of this class public in order to generate doxygen documentation for them,
  * but they should really all be private. */
class c400drv : public asynPortDriver {
public:
    c400drv(const char *portName, char *ip);

    /* These are the methods that we override from asynPortDriver */
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                        size_t nElements, size_t *nIn);

    /* These are the methods that are new to this class */
    void update_counts();
    
protected:
    /** Values used for pasynUser->reason, and indexes into the parameter library. */
    int P_DAC1;
    int P_DAC2;
    int P_DAC3;
    int P_DAC4;
    int P_DEAD;
    int P_DHI1;
    int P_DHI2;
    int P_DHI3;
    int P_DHI4;
    int P_DLO1;
    int P_DLO2;
    int P_DLO3;
    int P_DLO4;
    int P_HIVO_VOLTS1;
    int P_HIVO_VOLTS2;
    int P_HIVO_VOLTS3;
    int P_HIVO_VOLTS4;
    int P_HIVO_ENABLE1;
    int P_HIVO_ENABLE2;
    int P_HIVO_ENABLE3;
    int P_HIVO_ENABLE4;
    int P_PERIOD;
    int P_POLARITY1;
    int P_POLARITY2;
    int P_POLARITY3;
    int P_POLARITY4;
    int P_PULSER_Period;
    int P_PULSER_Width;
    int P_ACQUIRE;
    int P_BUFFER;
    int P_BURST;
    int P_COUNT1;
    int P_COUNT2;
    int P_COUNT3;
    int P_COUNT4;
    int P_ENCODER;
    int P_TRIGGER_MODE;
    int P_TRIGGER_POLARITY;
    int P_TRIGGER_START;
    int P_TRIGGER_STOP;
    int P_TRIGGER_PAUSE;
    int P_SYSTEM_IPMODE;
    int P_READ_BUFFER1;
    int P_READ_BUFFER2;
    int P_READ_BUFFER3;
    int P_READ_BUFFER4;
    int P_READ_BUFFER_TIME;

private:
    asynUser *pasynUserEcho;
    epicsFloat64 *pData_;
    epicsFloat64 *pData_ch1;
    epicsFloat64 *pData_ch2;
    epicsFloat64 *pData_ch3;
    epicsFloat64 *pData_ch4;
    epicsFloat64 *pData_time;

    asynStatus send_to_equipment(const char *writeBuffer, string &value);
    asynStatus send_to_equipment(const char *writeBuffer);

    float get_parsed_response(const char *writeBuffer, int n_element);

    double set_4_channels(const char *command_set, const char *command_ask, int param1, 
                            int param2, int param3, int param4, int channel, double val);
    double set_4_channels_int(const char *command_set, const char *command_ask, int param1, 
                            int param2, int param3, int param4, int channel, double val, int to_string=0);
    double set_2_vals(const char *command_set, const char *command_ask, int param1, 
                            int param2, int channel, double val, int is_float=1, int to_string=0);
    double set_direct(const char *command_set, const char *command_ask, int channel, double val);
    void get_n_set_4_channels(const char *command_ask, int param1, int param2, int param3, int param4, 
                                   int n_param1, int n_param2, int n_param3, int n_param4);
    void set_mbbo(const char *command_set, const string *mbbo_list, int mbbo_value);
    void update_buffer();
    void parse_counts(double *result_array, string received_line);
};
