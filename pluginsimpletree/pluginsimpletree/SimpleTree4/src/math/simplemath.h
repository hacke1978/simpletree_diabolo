#ifndef SIMPLEMATH_H
#define SIMPLEMATH_H


#include <QVector>
template <typename T>
class SimpleMath
{
public:
    SimpleMath();
    /**
     * @brief get_median Returns the median of a templated vector of numbers
     * @param number the vector of templated numbers
     * @return the median
     */
    static T get_median(QVector<T> number);

    /**
     * @brief get_mean Returns the mean of a templated vector of numbers
     * @param number the vector of templated numbers
     * @return  the mean
     */
    static T get_mean(QVector<T> number);

    static const float _PI;

};

#include "src/math/simplemath.hpp"

#endif // SIMPLEMATH_H
