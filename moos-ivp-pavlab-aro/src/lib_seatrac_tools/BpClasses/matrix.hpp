
#ifndef MATRIX_HPP
#define MATRIX_HPP

//Include common type definitions and macros
#include "common.h"
//#include "exception.hpp"
#include "maths.hpp"

//==============================================================================
//Declarations...
//==============================================================================
/*!-----------------------------------------------------------------------------
Class that provides static helper functions for 3x3 float matrix operations.
*/
class CMatrix33F {
	public:
		static void InitDcmDeg(PAttitudeF attDeg, float out[3][3]);
		static void InitDcmRad(PAttitudeF attRad, float out[3][3]);
		static void MultiplyByMatrix(const float a[3][3], const float b[3][3], float out[3][3]);
		static void MultiplyByVector(const float a[3][3], const float b[3], float out[3]);
};

/*!-----------------------------------------------------------------------------
template<typename T, size_t R, size_t C>
struct TMatrix {
	public:
		//Declare variables to hold the size of the matrix
		uint8 Rows = R;
		uint8 Cols = C;

		//Declare the data array for the matrix
		T Data[R][C];

		//Matrix constructor
		CMatrix(T value = T(0)) : Rows(R), Cols(C) {
			SetValues(value);
		}

		//Sets the all elements in the matrix to a specific value
		void SetValues(T value) {
			for(uint8 r = 0; r < R; r++) {
				for(uint8 c = 0; c < C; c++) {
					Data[r][c] = value;
				}
			}
		}

		//Sets the contents of the matrix to zeroes
		inline void Zeroes() {
			SetValues(T(0));
		}

		//Sets the contents of the matrix to ones
		inline void Ones() {
			SetValues(T(1));
		}

		//Function the sets the matrix to be an identity matrix
		void Identity() {
			for(uint8 r = 0; r < R; r++) {
				for(uint8 c = 0; c < C; c++) {
					this->Data[r][c] = (r == c) ? T(1) : T(0);
				}
			}
		}

};

struct TMatrix33F : public TMatrix<float, 3, 3> {
	public:

};
*/

//==============================================================================
// Implementation...
//==============================================================================
/*!-----------------------------------------------------------------------------
Initalise a Direction Cosine Matrix using attitude euler angles
NB: The specified attitude must be in DEGREES (not RADIANS!!!)
@param att	Pointer to the attitude structure to use when initalising the matrix - must be in DEGREES
*/
void CMatrix33F::InitDcmDeg(PAttitudeF attDeg, float out[3][3])
{
	//Convert the angles into radians
	TAttitudeF attRad = *attDeg;
	attRad.DegToRad();

	//Calcualte the DCM using radians
	CMatrix33F::InitDcmRad(&attRad, out);
}

/*!-----------------------------------------------------------------------------
Initalise a Direction Cosine Matrix using attitude euler angles
Angles are applied in the order Yaw, Pitch, Roll as per the NASA aircraft
definitions of axis and rotations.
NB: The specified attitude must be in RADIANS (not DEGREES!!!)
@param att	Pointer to the attitude structure to use when initalising the matrix - must be in RADIANS
*/
void CMatrix33F::InitDcmRad(PAttitudeF attRad, float out[3][3])
{
	//Compute common trig functions
	float cY = cosf(attRad->Yaw);
	float sY = sinf(attRad->Yaw);
	float cP = cosf(attRad->Pitch);
	float sP = sinf(attRad->Pitch);
	float cR = cosf(attRad->Roll);
	float sR = sinf(attRad->Roll);

	//Attitude euler angles use right-handed, intrinsic, XYZ convention (as per
	//NASA aircraft definition at http://www.grc.nasa.gov/WWW/k-12/airplane/rotations.html)
	//(which means: rotate around body axes Z, Y', X'')
	//Row 1
	out[0][0] = cP * cY;
	out[0][1] = (sR * sP * cY) - (cR * sY);
	out[0][2] = (cR * sP * cY) + (sR * sY);
	//Row 2
	out[1][0] = cP * sY;
	out[1][1] = (sR * sP * sY) + (cR * cY);
	out[1][2] = (cR * sP * sY) - (sR * cY);
	//Row 3
	out[2][0] = -sP;
	out[2][1] = sR * cP;
	out[2][2] = cR * cP;
}

/*!-----------------------------------------------------------------------------
Multiply two 3x3 matrices: out = a * b
NB: a must be a[3][3] and b must be b[3][3]
*/
void CMatrix33F::MultiplyByMatrix(const float a[3][3], const float b[3][3], float out[3][3])
{
	/*for(int x = 0; x < 3; x++) {  // rows
		for(int y = 0; y < 3; y++) {  // columns
			out->M[x][y] = (a->M[x][0] * b->M[0][y]) + (a->M[x][1] * b->M[1][y]) + (a->M[x][2] * b->M[2][y]);
		}
	}
	*/

	out[0][0] = (a[0][0] * b[0][0]) + (a[0][1] * b[1][0]) + (a[0][2] * b[2][0]);
	out[0][1] = (a[0][0] * b[0][1]) + (a[0][1] * b[1][1]) + (a[0][2] * b[2][1]);
	out[0][2] = (a[0][0] * b[0][2]) + (a[0][1] * b[1][2]) + (a[0][2] * b[2][2]);

	out[1][0] = (a[1][0] * b[0][0]) + (a[1][1] * b[1][0]) + (a[1][2] * b[2][0]);
	out[1][1] = (a[1][0] * b[0][1]) + (a[1][1] * b[1][1]) + (a[1][2] * b[2][1]);
	out[1][2] = (a[1][0] * b[0][2]) + (a[1][1] * b[1][2]) + (a[1][2] * b[2][2]);

	out[2][0] = (a[2][0] * b[0][0]) + (a[2][1] * b[1][0]) + (a[2][2] * b[2][0]);
	out[2][1] = (a[2][0] * b[0][1]) + (a[2][1] * b[1][1]) + (a[2][2] * b[2][1]);
	out[2][2] = (a[2][0] * b[0][2]) + (a[2][1] * b[1][2]) + (a[2][2] * b[2][2]);
}

/*!-----------------------------------------------------------------------------
Multiply 3x3 matrix with vector: out = a * b
NB: a must be a[3][3] and b must be b[3]
*/
void CMatrix33F::MultiplyByVector(const float a[3][3], const float b[3], float out[3])
{
	/*for(int x = 0; x < 3; x++) {
		out->V[x] = (a->M[x][0] * b->V[0]) + (a->M[x][1] * b->V[1]) + (a->M[x][2] * b->V[2]);
	}
	*/

	out[0] = (a[0][0] * b[0]) + (a[0][1] * b[1]) + (a[0][2] * b[2]);
	out[1] = (a[1][0] * b[0]) + (a[1][1] * b[1]) + (a[1][2] * b[2]);
	out[2] = (a[2][0] * b[0]) + (a[2][1] * b[1]) + (a[2][2] * b[2]);
}

#endif
