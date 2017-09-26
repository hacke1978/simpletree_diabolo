#ifndef SIMPLEMATH_HPP
#define SIMPLEMATH_HPP

#include "src/math/simplemath.h"

template <typename T>
const float SimpleMath<T>::_PI = 3.1415926f;

template <typename T>
SimpleMath<T>::SimpleMath()
{

}

template <typename T>
T SimpleMath<T>::get_median(QVector<T> number)
{
    qSort(number);
    T median = 0;
    int size = number.size();
    if(size>0)
    {
        if(size%2==0)
        {
            median = (number.at(size/2-1)+number.at(size/2))/2;
        } else
        {
            median = number.at(size/2);
        }
    }
    return median;
}

template <typename T>
T SimpleMath<T>::get_mean(QVector<T> number)
{
    int size = number.size();
    if(size>0)
    {
        T sum = 0;
        QVectorIterator<float> it(number);
        while(it.hasNext())
        {
            sum += it.next();
        }
        return (sum/size);
    }
    return 0;
}


#endif // SIMPLEMATH_HPP
