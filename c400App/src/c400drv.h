#include "asynPortDriver.h"

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */

#define P_DACString1         "DAC1"         /* asynFloat64,  r/w */
#define P_DACString2         "DAC2"         /* asynFloat64,  r/w */
#define P_DACString3         "DAC3"         /* asynFloat64,  r/w */
#define P_DACString4         "DAC4"         /* asynFloat64,  r/w */
#define P_DEADString         "DEAD"         /* asynFloat64,  r/w */
#define P_DHIString1         "DHI1"         /* asynFloat64,  r/w */
#define P_DHIString2         "DHI2"         /* asynFloat64,  r/w */
#define P_DHIString3         "DHI3"         /* asynFloat64,  r/w */
#define P_DHIString4         "DHI4"         /* asynFloat64,  r/w */
#define P_DLOString1         "DLO1"         /* asynFloat64,  r/w */
#define P_DLOString2         "DLO2"         /* asynFloat64,  r/w */
#define P_DLOString3         "DLO3"         /* asynFloat64,  r/w */
#define P_DLOString4         "DLO4"         /* asynFloat64,  r/w */

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
    virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);

    /* These are the methods that are new to this class */
    
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

private:
    asynUser *pasynUserEcho;
    float get_channel_val(std::string val, int channel);
    std::string send_to_equipment(const char *msg_ptr);
    double set_4_channels(const char *command_set, const char *command_ask, int param1, 
                            int param2, int param3, int param4, int channel, double val);
    double set_direct(const char *command_set, const char *command_ask, int channel, double val);
    void sync_w_device();
};
