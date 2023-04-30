#ifndef MATH_H
#define MATH_H


//help to finish DDA
namespace Math
{
template<typename T>

    T lerp(const T & fromV,const T & toV,float delta)
    {
       return fromV * (1-delta)  + toV * delta ;
    }
};


#endif // MATH_H
