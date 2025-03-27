/*==============================================================================
File that defines commonly used types throughout the firmware framework
==============================================================================*/
//Prevent multiple inclusions of this file
#ifndef TYPES_H
#define TYPES_H

//------------------------------------------------------------------------------
//Include C++ Libraries
#ifdef __cplusplus
	using namespace std;	//Allow use of the standard namespace without explicit declaration
	#include <string>
#endif

#include <stdbool.h>

//==============================================================================
//Define Fundamental Constants
//==============================================================================
#ifndef FALSE
	#define  FALSE  false                /* Boolean value FALSE. FALSE is defined always as a zero value. */
#endif

#ifndef TRUE
	#define  TRUE   true                /* Boolean value TRUE. TRUE is defined always as a non zero value. */
#endif

#ifndef null    
    #ifdef __cplusplus
        #define null 0
    #else
        #define null ((void*)0)
    #endif
#endif

#ifndef NULL
	#define  NULL   null
#endif

#ifndef NULLPTR
	#define NULLPTR	((void*)NULL)
#endif

//==============================================================================
//Fundamental Types
//==============================================================================
//Define lowercase types
//typedef bool boolean;
/*
typedef uint8_t byte;
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef int64_t int64;
typedef float single;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;
*/
//typedef unsigned char		byte;
typedef signed char			int8;
typedef signed short		int16;
typedef signed int			int32;
typedef signed long long	int64;
typedef float				single;
typedef unsigned char		uint8;
typedef unsigned short		uint16;
typedef unsigned int		uint32;
typedef unsigned long long	uint64;

typedef float				float32;
typedef double				float64;
typedef long double			float128;

//Constant numeric types
typedef const int8			cint8;
typedef const int16			cint16;
typedef const int32			cint32;
typedef const uint8			cuint8;
typedef const uint16		cuint16;
typedef const uint32		cuint32;

//Pointer Lowercase Types
typedef void*				pointer;
typedef char*				pchar;
typedef double*				pdouble;
typedef float*				pfloat;
typedef int8*				pint8;
typedef int16*				pint16;
typedef int32*				pint32;
typedef int64*				pint64;
typedef float*				psingle;
typedef uint8*				puint8;
typedef uint16*				puint16;
typedef uint32*				puint32;
typedef uint64*				puint64;

//Pointers to constant types
typedef const int8*			pcint8;
typedef const int16*		pcint16;
typedef const int32*		pcint32;
typedef const uint8*		pcuint8;
typedef const uint16*		pcuint16;
typedef const uint32*		pcuint32;


#ifdef __cplusplus

typedef bool*				pbool;
typedef string*				pstring;

#endif

//==============================================================================
//C++ Standard Library Helper Definitions
//==============================================================================
#ifdef __cplusplus


/*! Define a vector that holds bytes */
//typedef std::vector<uint8> TByteVector;

/*! Define a vector that holds characters */
//typedef std::vector<char> TCharVector;

/*! Define a pointer to a stream buffer */
//typedef std::streambuf* PStreambuf;

#endif

//==============================================================================
#endif
