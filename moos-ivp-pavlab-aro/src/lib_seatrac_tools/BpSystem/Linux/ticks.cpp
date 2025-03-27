#include "ticks.hpp"

//==============================================================================
//Class Implimentation...
//==============================================================================
//CTicks
//==============================================================================
/*!-----------------------------------------------------------------------------
*/
double CTicks::GetMicroseconds()
{
    return CTicks::GetSeconds() * 1000000.0;
}

/*!-----------------------------------------------------------------------------
*/
double CTicks::GetMilliseconds()
{
    return CTicks::GetSeconds() * 1000.0;
}

/*!-----------------------------------------------------------------------------
*/
int64 CTicks::GetNanoseconds()
{
    return CTicks::GetNanoseconds();
}

/*!-----------------------------------------------------------------------------
*/
double CTicks::GetSeconds()
{
    //Get the time ellapsed
    struct timespec now;
    bool success = (clock_gettime(CLOCK_MONOTONIC, &now) == 0);
    if(success)
        return (double)now.tv_sec + ((double)now.tv_nsec * 0.000000001);
    else
        return -1.0;
}

/*!-----------------------------------------------------------------------------
(see https://msdn.microsoft.com/en-us/library/windows/desktop/ms724411(v=vs.85).aspx)
*/
int64 CTicks::GetTicks()
{
    //Get the time ellapsed
    struct timespec now;
    bool success = (clock_gettime(CLOCK_MONOTONIC, &now) == 0);

    if(success)
        return (now.tv_sec * 1000000000LL) + (now.tv_nsec);
    else
        return -1;
}

/*!-----------------------------------------------------------------------------
Function that returns the frequency that the system tick counter value is incrimented
at, in counts per second.
*/
uint32 CTicks::GetTickFrequency()
{
    return 1000000000LL;
}

/*!-----------------------------------------------------------------------------
Function that sleeps for the specified number of milliseconds
*/
void CTicks::SleepMilliseconds(uint32 msecs)
{
    timespec tim;
    tim.tv_sec  = 0;

    while (msecs > 999) {
        tim.tv_sec++;
        msecs -= 1000;
    }

    tim.tv_nsec = msecs * 1000000;

    while (nanosleep(&tim , &tim)) { }
}
//==============================================================================
