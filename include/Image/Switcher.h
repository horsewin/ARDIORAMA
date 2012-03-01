/*
 * =====================================================================================
 *
 *       Filename:  Image/Switcher.h
 *
 *    Description:  Version
 *
 *        Version:  1.0
 *        Created:  2010年01月28日 18時46分40秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Tateishi Takahiro (), 
 *        Company:  Osaka univ
 *
 * =====================================================================================
 */
#ifndef INCLUDED__SWITCHER_H_
#define INCLUDED__SWITCHER_H_
#include <cassert>

namespace PTAMM{

//状態
typedef enum{
	CONSTRUCT, COMPOSE
} State;

inline void inaccessibilityLine( void ){
	assert( 0 );
}
//
//イベント
class Switcher{
public:
	virtual ~Switcher( void );

	virtual State get( void ) const = 0;
	virtual void toggle( void ) = 0;

	static Switcher * create( void );
protected:
	Switcher( void );
};
}
#endif

