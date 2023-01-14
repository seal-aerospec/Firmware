#ifndef RTC_H
#define RTC_H

#include "RTClib.h"

#define RTC_INT 32
#define RTC_RESET 25

// initializes rtc, if lost power, sets time to compile time
bool initRtc();

// returns a DateTime object
DateTime getDateTime();

#endif
