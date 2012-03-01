/*
 * =====================================================================================
 *
 *       Filename:  Switcher.cc
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2010年01月31日 23時07分47秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Tateishi Takahiro (), 
 *        Company:  Osaka univ
 *
 * =====================================================================================
 */

#include "Image/Switcher.h"

namespace PTAMM{

class SwitcherImpl : public Switcher{
public:
	SwitcherImpl( void ){
		s = CONSTRUCT;
	}
	virtual ~SwitcherImpl( void ){ }

	void toggle( void ){
		switch( s ){
			case CONSTRUCT:
				s = COMPOSE;
				break;
			case COMPOSE:
				s = CONSTRUCT;
				break;
			default:
				inaccessibilityLine( );
				break;
		}
	}

	inline State get( void ) const{
		return s;
	}
private:
	State s;
};

Switcher::Switcher( void ){ }
Switcher::~Switcher( void ){ }
Switcher * Switcher::create( void ){
	return new SwitcherImpl( );
}
}
