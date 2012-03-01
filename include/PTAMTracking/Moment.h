/*
 * PTAMTracking/Moment.h
 *
 *  Created on: 2011/06/29
 *      Author: umakatsu
 */

#ifndef MOMENT_H_
#define MOMENT_H_

#include <vector>

class Moment{
public:
	Moment( void );
	~Moment( void );

	void 	Reset( void );
	void	SetValue( int val );
	double GetMean( void );
	double GetVariance( void );
	double GetSkewness( void );
	double GetKurtosis( void );

private:
	std::vector< int > values;		// A set of data
	std::vector< double > caches;
	double mean;
	double sigma2;					// Variance calculated by "values"
	double kurtosis;
	bool	update;
};

#endif /* MOMENT_H_ */
