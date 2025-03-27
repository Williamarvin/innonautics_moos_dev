/*==============================================================================
C++ Module that provides the definitions and implementation for a lookup-table
enhanced CRC16 cyclic redundancy check algorithm

10/09/2013 - Created v1.0 of file based on code by JA Neesham, Newcastle SEA Lab
==============================================================================*/
//Prevent multiple inclusions of this file
#ifndef CRC16_HPP
#define CRC16_HPP

//Include system libraries

//Include common type definitions and macros
#include "common.h"

//==============================================================================
//==============================================================================
#define CRC16_GEN_POLY		0xA001u			/* CRC16-IBM Generator polynomial(x^16 + x^15 + x^2 + 1) - LSB first code */

#define CRC16_LEN			sizeof(uint16)

/*!
Class of static helper functions for manipulating acoustic messages and packets
*/
class CCrc16 {
	private:
		//Declare static variables
		static uint16 _lut[256];

	public:
		//Static Methods
		static void Init(uint32 poly = CRC16_GEN_POLY);
		static uint16 Calc(puint8 buf, uint32 offset, uint32 length, uint16 init = 0);
};

//==============================================================================
#endif
