/*
 * =====================================================================================
 *
 *       Filename:  Button.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2010年01月29日 19時47分34秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Tateishi Takahiro (), 
 *        Company:  Osaka univ
 *
 * =====================================================================================
 */
#ifndef INCLUDED__MOUSE_H_
#define INCLUDED__MOUSE_H_
class Button{
public:
	inline Button( void ){
		is_on = false;
		is_pressed = false;
		is_released = false;
	}
	virtual ~Button( void ){ }

	inline bool isPressed( void ) const{
		return is_pressed;
	}
	inline bool isReleased( void ) const{
		return is_released;
	}

	void set( bool current ){
		is_pressed = false;
		is_released = false;
		bool prev = is_on;
		if( current && ( ! prev ) ){
			is_pressed = true;
		}
		if( ( ! current ) && prev ){
			is_released = true;
		}
		is_on = current;
	}

	inline bool isDown( void )const {
		return is_on;
	}
private:
	bool is_on; //現在押されているかどうか

	bool is_pressed; //押された瞬間
	bool is_released; //離された瞬間
};

#endif
