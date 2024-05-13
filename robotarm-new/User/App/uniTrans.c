#include "uniTrans.h"

q15 UniTrans_F2Q(float input, float inputRange);


// -2PI~2PI -> 0~65535
q15 UniTrans_FRad2QRad(float input)
{
	return UniTrans_F2Q(input+2*PI, 4*PI);	// ( 1 q15 Rad = 4 Pi / 65535 ）
}

float UniTrans_QRad2FRad(q15 input)
{
	return (input/Q15_COEF*4*PI);
}


//float UniTrans_Q2F(q15 input, )

q15 UniTrans_F2Q(float input, float inputRange)
{
	q15 ret;
	return ret = input/inputRange*Q15_COEF;
}
