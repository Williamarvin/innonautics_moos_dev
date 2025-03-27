/*==============================================================================
File that defines commonly used types throughout the BPX framework
==============================================================================*/
//Prevent multiple inclusions of this file
#ifndef COMMON_H
#define COMMON_H

//------------------------------------------------------------------------------
//Include C++ Libraries
#ifdef __cplusplus
	//#include <iostream>
	using namespace std;	//Allow use of the standard namespace without explicit declaration
#endif

//------------------------------------------------------------------------------
//Include system libraries
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>	//Include library with the malloc, realloc and free functions

//------------------------------------------------------------------------------
//Include the CMSIS platform definitions

//Include other definitions
#include "types.h"
#include "macros.h"

//==============================================================================
//Definitions...
//==============================================================================


//==============================================================================
#endif
