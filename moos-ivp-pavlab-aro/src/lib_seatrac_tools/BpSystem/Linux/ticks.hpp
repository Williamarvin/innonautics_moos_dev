/*==============================================================================
Creates a class that provides basic access to a high resolution system ticks timer

04/08/2017 - Created v1.0 of file (R.Sharphouse)
==============================================================================*/
//Prevent multiple inclusions of this file
#ifndef TICKS_HPP
#define TICKS_HPP

//Include system libs
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include <stddef.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

//Include the base class
#include "common.h"
#include "events.hpp"

//------------------------------------------------------------------------------
/*! Class that impliments tick based timing */
class CTicks
{
    public:
        //Static Methods
        static double GetMicroseconds();
        static double GetMilliseconds();
        static int64 GetNanoseconds();
        static double GetSeconds();
        static int64 GetTicks();
        static uint32 GetTickFrequency();
		static void SleepMilliseconds(uint32 msecs);
};

//==============================================================================
#endif
