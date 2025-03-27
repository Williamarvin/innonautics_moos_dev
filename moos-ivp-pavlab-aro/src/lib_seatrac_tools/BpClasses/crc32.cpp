#include "crc32.hpp"

//==============================================================================
//Class Implementation...
//==============================================================================
//CCrc32
//==============================================================================
/*!-----------------------------------------------------------------------------
Function that computes the CRC32 for an array of 8-bit data.
*/
uint32 CCrc32::CalcBuffer(puint8 data, uint32 len, uint32 poly, uint32 last)
{
	uint32 crc = last;	
	for(uint32 i = 0; i < len; i++) {
		crc = CCrc32::CalcValue(*data, poly, crc);
		data++;
	}
	return crc;
}

/*!-----------------------------------------------------------------------------
Function that computes the CRC32 for an array of 32-bit data.
*/
uint32 CCrc32::CalcBuffer(puint32 data, uint32 len, uint32 poly, uint32 last)
{
	uint32 crc = last;	
	for(uint32 i = 0; i < len; i++) {
		crc = CCrc32::CalcValue(*data, poly, crc);
		data++;
	}
	return crc;
}

/*!-----------------------------------------------------------------------------
Function that computes the CRC value and adds it into the input CRC value,
allowing a rolling CRC function to be performed on arrays etc.
NB: If using CRC's of less than 32 bits, truncate/cast the output to the
appropriate length of the CRC polynomial (i.e. 8 or 16 bits)
@param value The value to add into the CRC
@param poly The polynomial (bit reversed) to use
@param last The previous CRC value to add the byte into
@result The new CRC value
*/
uint32 CCrc32::CalcValue(uint8 value, uint32 poly, uint32 last)
{	
	uint32 crc = last;
    for(uint8 i = 0; i < 8; i++) {
        if((value & 0x00000001) ^ (crc & 0x00000001)) {
            crc >>= 1;
            crc ^= poly;
        }
        else
            crc >>= 1;
        value >>= 1;
    }
    return crc;
}

/*!-----------------------------------------------------------------------------
Function that computes the CRC value and adds it into the input CRC value,
allowing a rolling CRC function to be performed on arrays etc.
NB: If using CRC's of less than 32 bits, truncate/cast the output to the
appropriate length of the CRC polynomial (i.e. 8 or 16 bits)
@param value The value to add into the CRC
@param poly The polynomial (bit reversed) to use
@param last The previous CRC value to add the byte into
@result The new CRC value
*/
uint32 CCrc32::CalcValue(uint32 value, uint32 poly, uint32 last)
{
	uint32 crc = last;
    for(uint8 i = 0; i < 32; i++) {
        if((value & 0x00000001) ^ (crc & 0x00000001)) {
            crc >>= 1;
            crc ^= poly;
        }
        else
            crc >>= 1;
        value >>= 1;
    }
    return crc;
}

