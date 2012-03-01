/*
 * Moment.cc
 *
 *  Created on: 2011/06/29
 *      Author: umakatsu
 */

#include "PTAMTracking/Moment.h"
#include <iostream>

using namespace std;

namespace{
	template<typename X> bool CheckValid( vector< X> obj){
		if( obj.size() > 0 ) return true;
		else return false;
	}
}

Moment::Moment( void )
: mean(-1.0) , sigma2(-1.0) , kurtosis(-1.0) , update(true)
{
	values.clear();
	caches.clear();
}

Moment::~Moment( void ){
	values.clear();
}


void Moment::SetValue( int val ){
	values.push_back( val );
	update = true;
}


double Moment::GetMean( void ) {
	if( CheckValid(values) );
	else return (-1.0);
	if( update ){
	  mean=0.0;
	  vector< int >::iterator it = values.begin();
		while( it != values.end() ){
			mean += static_cast< double >( (*it) );
			++it;
		}
		mean /= values.size();
		cout << "mean = " << mean;
//		update = false;
	}
	return mean;
}


double Moment::GetVariance( void ){
	if( CheckValid(values) );
	else return -1.0;
	if( update ){
		if( mean < 0.0 ){
			GetMean();
		}
		sigma2 = 0.0;
		caches.clear();
		vector< int >::iterator it = values.begin();
		while( it != values.end() ){
			double tmp = ( static_cast< double >( (*it) ) - mean ) * ( static_cast< double >( (*it) ) - mean );
			caches.push_back( tmp );
			sigma2 += tmp;
			++it;
		}
		sigma2 /= values.size();
		cout << "sigma = " << sigma2;
//		update = false;
	}
	return sigma2;
}


double Moment::GetKurtosis( void ){
	if( CheckValid(values) );
	else return -1.0;
	if( update ){
		if( mean < 0.0 ){
			GetMean();
		}
		if( sigma2 < 0.0 ){
			GetVariance();
		}
		double kurtosis = 0.0;
		caches.clear();
		vector< double >::iterator it = caches.begin();
		while( it != caches.end() ){
			double tmp = (*it) * (*it);
			kurtosis += tmp;
			++it;
		}
		kurtosis = kurtosis / ( sigma2 * sigma2) - 3;
		cout << "Kurtosis = " << kurtosis;
//		update = false;
		return kurtosis;
	}
	return kurtosis;
}
