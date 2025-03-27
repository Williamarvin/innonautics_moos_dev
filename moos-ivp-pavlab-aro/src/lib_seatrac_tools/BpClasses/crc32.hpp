/*==============================================================================
C++ Module that provides the definitions and implementation for CRC32
calculation algorithm

21/05/2014 - Created v1.0 of file
==============================================================================*/
//Prevent multiple inclusions of this file
#ifndef CRC32_HPP
#define CRC32_HPP

//Include system libraries

//Include common type definitions and macros
#include "common.h"

//==============================================================================
//==============================================================================
#define	CRC32_GEN_POLY		0xEDB88320u		/* CRC32 Generator (reversed) polynomial */

#define CRC32_LEN			sizeof(uint32)

/*!
Class of static helper functions for manipulating acoustic messages and packets
*/
class CCrc32 {
	public:
		//Static Methods
		static uint32 CalcBuffer(puint8 data, uint32 len, uint32 poly, uint32 last = 0);
		static uint32 CalcBuffer(puint32 data, uint32 len, uint32 poly, uint32 last = 0);
		static uint32 CalcValue(uint8 value, uint32 poly, uint32 last = 0);
		static uint32 CalcValue(uint32 value, uint32 poly, uint32 last = 0);
};

//==============================================================================
#endif
