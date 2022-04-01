#include "asynPortDriver.h"

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */

#define P_VoltString1         "VOLT1"         /* asynFloat64,  r/w */
#define P_VoltString2         "VOLT2"         /* asynFloat64,  r/w */
#define P_VoltString3         "VOLT3"         /* asynFloat64,  r/w */
#define P_VoltString4         "VOLT4"         /* asynFloat64,  r/w */
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
    double set_voltage(int channel, double val);
protected:
    /** Values used for pasynUser->reason, and indexes into the parameter library. */
    int P_Volt1;
    int P_Volt2;
    int P_Volt3;
    int P_Volt4;

private:
    asynUser *pasynUserEcho;
    float get_channel_val(std::string val, int channel);
    std::string send_to_equipment(const char *msg_ptr);

};
