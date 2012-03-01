/*
 * parameter.h
 *
 *  Created on: 2010/11/12
 *      Author: umakatsu
 */

#ifndef PARAMETER_H_
#define PARAMETER_H_

#include <math.h>
#include <float.h>

typedef double SCALAR;
struct SamplingData{
	int K;			/* the number of class*/
	int D;			/* the number of dimension */
	SCALAR *x;		/* RGB value */
	SCALAR *p;		/* pixel coordinate */
	SCALAR *E;		/* average */

	SamplingData(int k, int d): K(k), D(d) {
		x = new SCALAR[D];
		p = new SCALAR[D-1];
		E = new SCALAR[K];
	}

	~SamplingData() {
		delete[] x;
		delete[] p;
		delete[] E;
	}
};

struct Theta {
	int D;
	SCALAR Phi;
	SCALAR *Mu;
	SCALAR *invS;
	SCALAR *S;
	SCALAR detS;
	SCALAR weight;

	Theta(int d): D(d) {
		Mu 		= new SCALAR[D];
		invS 	= new SCALAR[D*D];
		S 		= new SCALAR[D*D];
		detS	= 0.0;
		weight = 0.0;
	}

	void init(int k, SCALAR *x) {
		Phi = 1.0 / (SCALAR)k;
		for(int id = 0; id < D; id++) {
			Mu[id] = x[id];
			for(int id2 = 0; id2 < D; id2++) {
				if(id == id2) {
					invS[id*D+id2] = 1.0;
					S[id*D+id2] = 1.0;
				} else {
					invS[id*D+id2] = 0.0;
					S[id*D+id2] = 0.0;
				}
			}
		}
		detS = 1.0;
	}

	SCALAR N(SCALAR *x) {
		SCALAR r = 0.0;

		for(int id0 = 0; id0 < D; id0++) {
			SCALAR r1 = 0.0;

			for(int id1 = 0; id1 < D; id1++) {
				r1 += invS[id1*D+id0] * (x[id1] - Mu[id1]);
			}

			r += r1 * (x[id0] - Mu[id0]);
		}
//		r = fabs(r);
		double sqrt_detS = sqrt(fabs(detS));
		/* 共分散行列のdeterminantがおかしな値になるときの補正 */
		if(isnan(sqrt_detS) || isinf(sqrt_detS) ) sqrt_detS = DBL_MAX;
		else if(sqrt_detS < DBL_MIN) sqrt_detS = DBL_MIN;
		double tmp = exp(-0.5 * r) / pow(2.0*M_PI, D) / sqrt_detS;
//		printf("detS = %lf\n",sqrt_detS);
		/* 値の発散を防止 */
		if(isinf(tmp) || isnan(tmp)) 	tmp = DBL_MAX;
		else if(tmp < DBL_MIN)			tmp = DBL_MIN;
		return tmp;
	}

	~Theta() {
		delete[] S;
		delete[] invS;
		delete[] Mu;
	}
};

#endif /* PARAMETER_H_ */
