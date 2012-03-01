/*
 * define.h
 *
 *  Created on: 2011/03/03
 *      Author: umakatsu
 */

#ifndef DEFINE_H_
#define DEFINE_H_

#include <TooN/TooN.h>
#include <time.h>
#include <sys/time.h>

#define REP(i,n) for(int i=0;i<(int)n;++i)
typedef unsigned long int ulint;
typedef unsigned char uchar;
typedef double SCALAR;
typedef TooN::Vector< 3 > Vertex;

/* 時間計測関数 */
double inline TIME()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + (double)tv.tv_usec*1e-6;
}

double inline PROCESSTIME(double start){
	double end = TIME();
	return( (double)(end - start) );
}

#endif /* DEFINE_H_ */
