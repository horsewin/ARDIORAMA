#include "Window/Analysis.h"

#include <boost/random.hpp>
#include <ctime>
#include <vector>
#include <cmath>
#include <algorithm>
#include <utility>

#include <boost/foreach.hpp>
#include <boost/iterator/counting_iterator.hpp>


namespace {
	const double EPSILON( 1.0e-20 );
	inline bool isEqual( double l, double r ){
		return ( fabs( l - r ) < EPSILON );
	}

	struct CompFunc{
	public:
		bool operator( )( int a, int b ) const{
			return d[ a ] < d[ b ];
		}

		const std::vector< double > &  d;
		CompFunc(
			const std::vector< double > &  d_ 
		):
			d(d_)
		{ }
	};
};


class AnalysisImpl: public Analysis{
public:
	AnalysisImpl( void ){
		data.clear( );
	}
 
	~AnalysisImpl( void ){ }

	void add( double d ){
		data.push_back( d );
	}




	void runKMeans( void ){
		double  pre_means[ 2 ];
		for( int i( 0 ); i < 2; i ++){
			pre_means[ i ] = 0.0;
			means[ i ]     = 0.0;
		}

//初期グループとしてランダムに分配
		randomClassify( );
		evalMeans( );

		for(;;){
			for( int i( 0 ); i < 2; i ++){
				pre_means[ i ] = means[ i ];
			}
			reClassify( );
			evalMeans( );
   	   //平均値に変動が見られなくなったら、ループ終了
			if( isEqual( means[ 0 ], pre_means[ 0 ] ) && isEqual( means[ 1 ], pre_means[ 1 ] ) ){
				break;
			}
		}
	}

	void  get( std::vector< int > &  ret, int  i ) const{
		ret.resize( group[ i ].size( ) );
		std::copy( group[ i ].begin( ), group[ i ].end( ), ret.begin( ) );
	}

	unsigned size( int i) const{
		return group[ i ].size( );
	}

	void clear( void ){
		data.clear( );
		for( int i( 0 );i < 2; i ++){
			group[ i ].clear( );
			means[ i ] = 0.0;
		}
	}



	bool smirnovGrubbsTest( std::vector< int > &  ret ){
		bool  has_outlier( false );
		int  data_size( data.size( ) );
		std::vector< int >  d( data_size );
		std::copy( boost::make_counting_iterator( 0 ), boost::make_counting_iterator( data_size ), d.begin( ) );
    
    
		CompFunc  compFunc(data);
		std::sort( d.begin( ), d.end( ), compFunc );
    
		std::pair< double, double > meanAndVar = evalMeansAndUnbiasedVariance( d );
		double  mean( meanAndVar.first ), rootVar( sqrt( meanAndVar.second ) );

		const double  THRESHOLD( 2.557 );//http://aoki2.si.gunma-u.ac.jp/lecture/Grubbs/Grubbs.html

		std::vector< int >::reverse_iterator  itr;
		for( itr = d.rbegin( ); itr != d.rend( ); itr ++){
			double  d( fabs( data[* itr ] - mean ) / rootVar );
			if( d < THRESHOLD ){//ソートされているため、この時点で外れ値でないと検定されたならば以降は全てOK
				break;
			}else{
				has_outlier = true;
			}
		}
		ret.clear( );
		for( ;itr != d.rend( ); itr ++){
			ret.push_back(* itr );
		}
		return has_outlier;
	}

private:
	std::vector< double >   data;
	std::vector< int >      group[ 2 ];
	double                  means[ 2 ];
  
  //各クラスタの平均を求める
	void  evalMeans( void ){
		for( int i( 0 ); i < 2; i ++){
			means[ i ] = 0.0;
			for( std::vector< int >::iterator itr( group[ i ].begin( ) ); itr != group[ i ].end( ); itr ++){
				means[ i ] += data[(* itr ) ];
			}
			means[ i ] /= group[ i ].size( );
		}
	}

	std::pair< double, double >  evalMeansAndUnbiasedVariance( const std::vector< int > &  d){
 		std::pair< double, double > ret;
		double  mean( 0.0 );
		double  var( 0.0 );

		for( unsigned i( 0 ); i < d.size( ); i ++){
			mean += data[ d[ i ] ];
		}
		mean /= d.size( );

		for( unsigned i( 0 );i < d.size( ); i ++){
			double e = ( mean - data[ d[ i ] ] );
			var += e * e;
		}
		var /= d.size( ) - 1;
    
		ret.first  = mean;
		ret.second = var;

		return ret;
	}

	void randomClassify( void ){
		boost::mt19937 gen( static_cast< unsigned long >( time( 0 ) ) );
		boost::uniform_smallint< > dst( 0, 1 );
		boost::variate_generator< boost::mt19937 &, boost::uniform_smallint< > >  rand( gen, dst );

		unsigned data_size( data.size( ) );
		for( unsigned i( 0 ); i < data_size; i ++){ 
			group[ rand( ) ].push_back( i );
		}
	}

	void reClassify( void ){
    //求めた平均に従って再分配 0に小さいがわ、1に大きいがわ
		double border( ( means[ 0 ] + means[ 1 ] ) / 2 );
		group[ 0 ].clear( );
		group[ 1 ].clear( );

		unsigned data_size( data.size( ) );
		for( unsigned i( 0 ); i < data_size; i ++){
			if( data[ i ] < border ){
				group[ 0 ].push_back( i );
			} else{
				group[ 1 ].push_back( i );
			}
		}
	}
};



Analysis *  Analysis::create( void ){
	return new AnalysisImpl( );
}

