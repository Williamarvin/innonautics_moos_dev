/*==============================================================================
Creates a simple Mutex class using the underlying operatring system function
calls.

For further information, see

http://www.unix.com/man-page/linux/3/pthread_mutex_init/
http://www.unix.com/man-page/linux/3/pthread_mutex_lock/
http://www.unix.com/man-page/linux/3/pthread_mutex_trylock/
http://www.unix.com/man-page/linux/3/pthread_mutex_unlock/


28/07/2017 - Created v1.0 of file (R.Sharphouse)
==============================================================================*/
//Prevent multiple inclusions of this file
#ifndef MUTEX_HPP
#define MUTEX_HPP

//Include system libs
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>

#include "pthread.h"

//Include the base class
#include "common.h"

//------------------------------------------------------------------------------
//! Define an enumration that can convey the status of the mutex
enum EMutexStatus {
    MUTEX_FAILED,
    MUTEX_SIGNALED,
    MUTEX_ABANDONED,
    MUTEX_TIMEOUT,
};

typedef EMutexStatus* PMutexStatus;

//------------------------------------------------------------------------------
/*! Define a class that wraps an impliments a Mutex Semaphore */
class CMutex
{
    protected:
        //Protected Members
        pthread_mutex_t _mutex;
        bool _valid;

    public:
        //Construction and Disposal
        CMutex();
        virtual ~CMutex();

        //Methods        
        bool Lock(PMutexStatus status = NULL);
        bool Unlock();
};

/*! Define a pointer to a Mutex object */
typedef CMutex* PMutex;

//==============================================================================
#endif
