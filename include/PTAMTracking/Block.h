#ifndef INCLUDED__BLOCK_H_
#define INCLUDED__BLOCK_H_

#include "ODE.h"
#include <TooN/numerics.h>
namespace TooN{
	class SE3;
}
class Matrix;
class ModelImpl;



class Block{
public:
	static Block *  create( const ModelImpl &  model_impl, const ODERoot &  ode_root, const ::Matrix & w );
	virtual ~Block( void );
	virtual void draw( const TooN::SE3 &  pose, const TooN::Matrix< 4 > & proj ) const = 0;
protected:
	Block( void );
};
#endif
