#include "maths.hpp"

//==============================================================================
//Class Implementation...
//==============================================================================
//CMaths
//==============================================================================
/*!-----------------------------------------------------------------------------
Function that converts an angle in degrees to radians
@param angle The degrees angle to convert
@result The angle in radians
*/
inline float CMaths::DegreesToRadians(float angle)
{
    return angle * (float)DEG_TO_RAD;
}

/*!-----------------------------------------------------------------------------
Function that converts an angle in degrees to radians
@param angle The degrees angle to convert
@result The angle in radians
*/
inline double CMaths::DegreesToRadians(double angle)
{
	return angle * DEG_TO_RAD;
}

/*!-----------------------------------------------------------------------------
Function that returns the fractional part of a floating point number
*/
float CMaths::Fraction(float value)
{
	return value - (float)((int32)value);
}

/*!-----------------------------------------------------------------------------
Function that returns the fractional part of a floating point number
*/
double CMaths::Fraction(double value)
{
	return value - (double)((int32)value);
}

/*!-----------------------------------------------------------------------------
Fast inverse square root implementation
@see http://en.wikipedia.org/wiki/Fast_inverse_square_root
*/
inline float CMaths::InverseSqrt(float number)
{
	int32 i;
	float x, y;
	const float f = 1.5F;

	x = number * 0.5F;
	y = number;
	i = *(long*)&y;
	i = 0x5f375a86 - (i >> 1);
	y = *(float*)&i;
	y = y * (f - (x * y * y));
	return y;
}

/*!-----------------------------------------------------------------------------
Function where the angle is contstrained to the interval 0.0 <= x < 360.0
This function is simmilar to RolloverDegrees, but just uses positive values
@param angle The angle to constrain
@result The contstrained angle
*/
inline float CMaths::NormaliseDegrees(float angle)
{
    if(angle >= 360.0F)
        return angle - 360.0F;
    else if(angle < 0.0F)
        return angle + 360.0F;
	else
		return angle;
}

/*!-----------------------------------------------------------------------------
Function where the angle is contstrained to the interval 0.0 <= x < 360.0
This function is simmilar to RolloverDegrees, but just uses positive values
@param angle The angle to constrain
@result The contstrained angle
*/
inline double CMaths::NormaliseDegrees(double angle)
{
    if(angle >= 360.0F)
        return angle - 360.0F;
    else if(angle < 0.0F)
        return angle + 360.0F;
	else
		return angle;
}

/*!-----------------------------------------------------------------------------
Function where the angle is contstrained to the interval 0.0 <= x < TWO_PI
This function is simmilar to RolloverDegrees, but just uses positive values
@param angle The angle to constrain
@result The contstrained angle
*/
inline float CMaths::NormaliseRadians(float angle)
{
    if(angle >= (float)TWO_PI)
        return angle - (float)TWO_PI;
    else if(angle < 0.0F)
        return angle + (float)TWO_PI;
	else
		return angle;
}

/*!-----------------------------------------------------------------------------
Function where the angle is contstrained to the interval 0.0 <= x < TWO_PI
This function is simmilar to RolloverDegrees, but just uses positive values
@param angle The angle to constrain
@result The contstrained angle
*/
inline double CMaths::NormaliseRadians(double angle)
{
	if(angle >= TWO_PI)
		return angle - TWO_PI;
    else if(angle < 0.0)
		return angle + TWO_PI;
	else
		return angle;
}

/*!-----------------------------------------------------------------------------
Function that converts an angle in radians to degrees
@param angle The radians angle to convert
@result The angle in degrees
*/
inline float CMaths::RadiansToDegrees(float angle)
{
    return angle * (float)RAD_TO_DEG;
}

/*!-----------------------------------------------------------------------------
Function that converts an angle in radians to degrees
@param angle The radians angle to convert
@result The angle in degrees
*/
inline double CMaths::RadiansToDegrees(double angle)
{
	return angle * RAD_TO_DEG;
}

/*!-----------------------------------------------------------------------------
Function that ensures the specified value is wrapped cycliclally between zero
and the maximum value. Useful for ensuring bearings lie in the range of 0 to 360 degrees etc.

template <typename T>
inline void CMaths::RangeCyclic(T* value, T max)
{
	while(*value < T(0))
		*value += max;
	while(*value >= max)
		*value -= max;
}
*/

/*!-----------------------------------------------------------------------------
Function that limits the specified value to an acceptable range of values between
a minimum and maxiumum.

template <typename T>
inline void CMaths::RangeLimit(T* value, T min, T max)
{
	if(*value < min)
		*value = min;
	else if(*value > max)
		*value = max;
}
*/

/*!-----------------------------------------------------------------------------
Function that checks the specified values lies within the specified range limits inclusive

template <typename T>
inline bool CMaths::RangeValidate(T value, T min, T max)
{
	return ((value >= min) && (value <= max));
}
*/

/*!-----------------------------------------------------------------------------
Function that contains an angle (in degrees) to the range -180.0 < x <= 180.0
@param angle The angle to constrain (in degrees)
@result The constrained angle (in degrees)
*/
inline float CMaths::RolloverDegrees(float angle)
{
    if(angle > 180.0F)
        return angle - 360.0F;
    else if(angle <= -180.0F)
        return angle + 360.0F;
	else
		return angle;
}

/*!-----------------------------------------------------------------------------
Function that contains an angle (in degrees) to the range -180.0 < x <= 180.0
@param angle The angle to constrain (in degrees)
@result The constrained angle (in degrees)
*/
inline double CMaths::RolloverDegrees(double angle)
{
	if(angle > 180.0)
		return angle - 360.0;
	else if(angle <= -180.0)
		return angle + 360.0;
	else
		return angle;
}

/*!-----------------------------------------------------------------------------
Function that contains an angle (in radians) to the range -PI < x <= PI
@param angle The angle to constrain (in radians)
@result The constrained angle (in radians)
*/
inline float CMaths::RolloverRadians(float angle)
{
    if(angle > (float)PI)
        return angle - (float)TWO_PI;
    else if(angle <= (float)-PI)
        return angle + (float)TWO_PI;
	else
		return angle;
}

/*!-----------------------------------------------------------------------------
Function that contains an angle (in radians) to the range -PI < x <= PI
@param angle The angle to constrain (in radians)
@result The constrained angle (in radians)
*/
inline double CMaths::RolloverRadians(double angle)
{
	if(angle > PI)
		return angle - TWO_PI;
	else if(angle <= -PI)
		return angle + TWO_PI;
	else
		return angle;
}

/*!-----------------------------------------------------------------------------
Function that rounds a value to the nearest integer
*/
int32 CMaths::Round(float value)
{
    return (int32)(value + 0.5F);
}

/*!-----------------------------------------------------------------------------
Function that rounds a value to the nearest integer
*/
int32 CMaths::Round(double value)
{
	return (int32)(value + 0.5);
}
/*!-----------------------------------------------------------------------------
Function that rounds a value upwards to the nearest value of the specified mutliple.
i.e. Rounding 13 up to a multiple of 4 will yeild a result of 16, while
rounding 12 up will yield a value of 12.
*/
inline int32 CMaths::RoundMultipleUp(int32 value, int32 multiple)
{
	if(value % multiple)
		return ((value / multiple) + 1) * multiple;
	else
		return value;
}

/*!-----------------------------------------------------------------------------
Function that returns either +1 or -1 based on the sign of the value.
This function never returns zero!
@param value The value to find the sign of
@result +1 or -1 depending on the sign on the input value
*/
inline float CMaths::Sign(float value)
{
    return (value < 0.0F) ? -1.0F : 1.0F;
}

/*!-----------------------------------------------------------------------------
Function that returns either +1 or -1 based on the sign of the value.
This function never returns zero!
@param value The value to find the sign of
@result +1 or -1 depending on the sign on the input value
*/
inline double CMaths::Sign(double value)
{
    return (value < 0.0) ? -1.0 : 1.0;
}

//==============================================================================
