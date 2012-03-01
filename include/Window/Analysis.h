#ifndef INCLUDED__ANALYSIS_H_
#define INCLUDED__ANALYSIS_H_
#include <vector>
class Analysis{
public:
	static Analysis *  create( void );

	virtual ~Analysis( void ){ };

	virtual void add( double  data ) = 0;

	virtual void runKMeans( void ) = 0;

	virtual void get( std::vector< int > &  ret, int idx ) const = 0;

	virtual size_t size( int i )const = 0;

	virtual void clear( void ) = 0;

	virtual bool smirnovGrubbsTest( std::vector< int > &  ret ) = 0;

protected:
	Analysis( void ){ };
};



#endif
