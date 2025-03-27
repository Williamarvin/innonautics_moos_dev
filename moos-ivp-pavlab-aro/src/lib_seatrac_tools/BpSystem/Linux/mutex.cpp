#include "mutex.hpp"

//==============================================================================
// Class Implimentation
//==============================================================================
/*!-----------------------------------------------------------------------------
Constructor
*/
CMutex::CMutex()
{    
    pthread_mutexattr_t attr;

    //Specify the mutex should be recursive (allow relocking by the same
    //thread without creating deadlocks)
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);

    //Create the mutex object
    _valid = (pthread_mutex_init(&_mutex, &attr) == 0);

    //Check a handle was created
    assert(_valid);
}

/*!-----------------------------------------------------------------------------
Destructor
*/
CMutex::~CMutex()
{
    if(_valid) {
        pthread_mutex_destroy(&_mutex);
    }
}

/*!-----------------------------------------------------------------------------
Function that is called to wait for the Mutex to lock.
If the Mutex is already locked, this funciton will block until the mutex unlocks
or the optionally specified timeout expired.
@param status points to a variable for optional locking information is stored.
@result True is the mutex has been locked and can be used, false is an error occured and the mutex can't lock.
*/
bool CMutex::Lock(PMutexStatus status)
{
    EMutexStatus localStatus;
    bool success = false;

    if(_valid) {
        //Try to lock the mutex, and block until lock acheived
        int result = pthread_mutex_lock(&_mutex);

        switch(result) {
            case 0 : { localStatus = MUTEX_SIGNALED; success = true; break; }
            case ETIMEDOUT : { localStatus = MUTEX_TIMEOUT; success = false; break; }
            case EBUSY : { localStatus = MUTEX_FAILED; success = false; break; }
            default: { localStatus = MUTEX_FAILED; success = false; break; }
        }
    }
    else {
        localStatus = MUTEX_FAILED;
        success = false;
    }

    //Store the status code
    if(status)
        *status = localStatus;

    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CMutex::Unlock()
{    
    bool success = false;

    if(_valid) {
        success = (pthread_mutex_unlock(&_mutex) == 0);
        if(success) {
            _valid = false;
        }
    }
    return success;
}

//==============================================================================
