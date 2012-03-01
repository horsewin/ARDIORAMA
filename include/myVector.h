/*
 * myVector.h
 *
 *  Created on: 2010/12/20
 *      Author: umakatsu
 *      TODO templateの使い方に関してhelpers.hの180行目あたりを参考にする
 */

#ifndef MYVECTOR_H_
#define MYVECTOR_H_

#include <TooN/TooN.h>
#include <TooN/se3.h>
//#include <TooN/numerics.h>
#include <TooN/numhelpers.h>
#include <assert.h>
#include <GL/gl.h>
#include <cmath>

template< int S >
inline TooN::Vector< 3 > outerProduct(const TooN::Vector< S > & src1 , const TooN::Vector< S > & src2){
	assert( S == 2 || S == 3);
	TooN::Vector< 3 > v1;
	TooN::Vector< 3 > v2;
	if( S == 2 ){
		v1[2] = src1[2];
		v2[2] = src2[2];
	}
	for(int i = 0; i < S; i++){
		v1[i] = src1[i];
		v2[i] = src2[i];
	}
	//calculation
	TooN::Vector< 3 > dst;
	dst[0] = v1[1]*v2[2] - v1[2]*v2[1];
	dst[1] = v1[2]*v2[0] - v1[0]*v2[2];
	dst[2] = v1[0]*v2[1] - v1[1]*v2[0];
	return dst;
}

template < int S>
inline double innerProduct3(const TooN::Vector< S > & src1 , const TooN::Vector< S > & src2){
	double dst = 0;
	for(int i = 0; i < S; i++){
		dst += src1[i] * src2[i];
	}
	return dst;
}

template < int S>
inline double norm3( const TooN::Vector< S > & vect ){
	double norm_value = 0.0;
	REP(element , S){
		norm_value += vect[element] * vect[element];
	}
	return std::sqrt(norm_value);
}

template <int S>
inline void ReverseVector3( TooN::Vector< S > & vect){
	REP(loop , S){
		vect[loop] = -1 * vect[loop];
	}
}

inline void glVertexVector(const TooN::Vector< 3 > & vertex_vector){
	glVertex3d((GLdouble)vertex_vector[0] , (GLdouble)vertex_vector[1] ,(GLdouble) vertex_vector[2]);
}

#endif /* MYVECTOR_H_ */
