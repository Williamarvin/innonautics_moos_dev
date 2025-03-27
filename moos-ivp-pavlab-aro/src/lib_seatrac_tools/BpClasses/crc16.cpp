#include "crc16.hpp"

//==============================================================================
//Class Implementation...
//==============================================================================
//CAcoustic
//==============================================================================
//Define static variables
uint16 CCrc16::_lut[256];

/*!-----------------------------------------------------------------------------
Function that initialises the CRC16 lookup table to save on arithmetic when
further CRC functions are performed
@param[in] poly	Generator polynomial used for the lookup tables
*/
void CCrc16::Init(uint32 poly)
{
	uint32 i, j, temp;
	puint16 lut = CCrc16::_lut;
	
	for(i = 0; i < 256; i++) {
		temp = i;
		for(j = 0; j < 8; j++) {
			if(temp & 0x0001u)
				temp = (temp >> 1) ^ poly;
			else
				temp >>= 1;
		}
		*lut = temp;
		lut++;
	}
}

/*!-----------------------------------------------------------------------------
Function that computes the CRC16 value for a buffer of data
@param[in]	buf		Pointer to the buffer containing the data to analyse.
@param[in]	offset	The offset from the begining of the buffer to start checksumming data at.
@param[in]	length	The number of bytes to run through the checksum.
@param[in]	init	The inital value to start computing the CRC from (this may be a previous CRC operation)
@result				The computed CRC value.
*/
uint16 CCrc16::Calc(puint8 buf, uint32 offset, uint32 length, uint16 init)
{	
	uint32 ctr = 0;
	uint32 csum = init;
	uint32 tmp;	
	
	for(ctr = offset; ctr < (length + offset); ctr++) {
		//tmp = *(buf + ctr) ^ ((uint8)csum);
		//csum >>= 8;
		//csum ^= CCrc16::_lut[tmp];
		tmp = *(buf + ctr) ^ (csum & 0xFF);
		csum >>= 8;
		csum ^= CCrc16::_lut[(uint8)tmp];
	}
	return csum & 0xFFFFu;
}

