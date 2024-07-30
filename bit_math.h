/**
 * @file    bit_math
 * @author  Hossam Elwahsh - https://github.com/HossamElwahsh / hossam.e.elwahsh@gmail.com
 * @date    30/7/2024
 */


#ifndef RX_2024_BIT_MATH_H
#define RX_2024_BIT_MATH_H

#define SET_BIT( REGISTER, BIT_NUMBER )		REGISTER = ( REGISTER | ( 1 << BIT_NUMBER ) )
#define CLR_BIT( REGISTER, BIT_NUMBER )		REGISTER = ( REGISTER & ~( 1 << BIT_NUMBER ) )
#define TOG_BIT( REGISTER, BIT_NUMBER )		REGISTER = ( REGISTER ^ ( 1 << BIT_NUMBER ) )
#define GET_BIT( REGISTER, BIT_NUMBER )		( ( REGISTER & ( 1 << BIT_NUMBER ) ) >> BIT_NUMBER ) // ( ( REGISTER >> BIT_NUMBER ) & 1 )

#define WRITE_BIT(REGISTER,BIT_NUMBER, BIT_VAL) ((BIT_VAL) ? ((REGISTER) |= (1UL << (BIT_NUMBER))) : ((REGISTER) &= ~(1UL << (BIT_NUMBER))))

#endif //RX_2024_BIT_MATH_H
