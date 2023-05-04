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
#define P_TRIGGER_MODEString            "TRIGGER_MODE"        /* asynInt32,    r/w */
#define P_TRIGGER_POLARITYString        "TRIGGER_POLARITY"    /* asynInt32,    r/w */
#define P_TRIGGER_STARTString           "TRIGGER_START"       /* asynInt32,    r/w */
#define P_TRIGGER_STOPString            "TRIGGER_STOP"        /* asynInt32,    r/w */
#define P_TRIGGER_PAUSEString           "TRIGGER_PAUSE"       /* asynInt32,    r/w */
#define P_SYSTEM_IPMODEString           "SYSTEM_IPMODE"       /* asynInt32,    r/w */
#define P_UPDATE_BUFFERString           "UPDATE_BUFFER"       /* asynInt32,    r/w */
#define P_READ_BUFFERString             "READ_BUFFER"         /* asynFloat64Array,  r/o */


        /* asynFloat64,  r/w */
// #define P_VertGainString           "SCOPE_VERT_GAIN"            /* asynFloat64,  r/w */

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
    int P_TRIGGER_MODE;
    int P_TRIGGER_POLARITY;
    int P_TRIGGER_START;
    int P_TRIGGER_STOP;
    int P_TRIGGER_PAUSE;
    int P_SYSTEM_IPMODE;
    int P_UPDATE_BUFFER;
    int P_READ_BUFFER;

private:
    asynUser *pasynUserEcho;
    epicsFloat64 *pData_;
    int buffer_array_size = 1000*8;
    // float get_channel_val(std::string val, int channel, std::string search_for=" ", int size_sep=3);
    float get_parsed_response(std::string val, int n_element, std::string delimiter = ",");
    std::string send_to_equipment(const char *msg_ptr);
    double set_4_channels(const char *command_set, const char *command_ask, int param1, 
                            int param2, int param3, int param4, int channel, double val);
    double set_4_channels_int(const char *command_set, const char *command_ask, int param1, 
                            int param2, int param3, int param4, int channel, double val, int to_string=0);
    double set_2_vals(const char *command_set, const char *command_ask, int param1, 
                            int param2, int channel, double val, int is_float=1, int to_string=0);
    double set_direct(const char *command_set, const char *command_ask, int channel, double val);
    void get_n_set_4_channels(const char *command_ask, int param1, int param2, int param3, int param4, 
                                   int n_param1, int n_param2, int n_param3, int n_param4);
    void set_mbbo(const char *command_set, const std::string *mbbo_list, int mbbo_value);
    void update_buffer(int n_elements);
    void parse_counts(double *result_array, std::string received_line);
};
