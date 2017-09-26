#include "testsimplemath.h"

TestSimpleMath::TestSimpleMath()
{
    test_median();
    test_mean();
}

void TestSimpleMath::test_median()
{
    QVector<float> vec;
    vec.push_back(14);
    vec.push_back(12);
    vec.push_back(5);
    vec.push_back(3);



    qDebug() << "Starting test for SimpleMath median check.";
    float median =  SimpleMath<float>::get_median(vec);

    if(std::abs(median-8.5)>_max)
    {
        qWarning() << "Median test 1 failed, the median deviation is :" <<std::abs(median-8.5) << " (should be near 0)";
    } else
    {
        qDebug() << "Median test 1 passed, the median deviation is :" << std::abs(median-8.5) << " (should be near 0)";
    }

    vec.push_back(200);
    median = SimpleMath<float>::get_median(vec);

    if(std::abs(median-12)>_max)
    {
        qWarning() << "Median test 2 failed, the median deviation is :" <<std::abs(median-12) << " (should be near 0)";
    } else
    {
        qDebug() << "Median test 2 passed, the median deviation is :" << std::abs(median-12) << " (should be near 0)";
    }
}

void TestSimpleMath::test_mean()
{
    QVector<float> vec;
    vec.push_back(14);
    vec.push_back(12);
    vec.push_back(5);
    vec.push_back(3);

    qDebug() << "Starting test for SimpleMath mean check.";
    float mean =  SimpleMath<float>::get_mean(vec);

    if(std::abs(mean-8.5)>_max)
    {
        qWarning() << "Mean test 1 failed, the mean deviation is :" <<std::abs(mean-8.5) << " (should be near 0)";
    } else
    {
        qDebug() << "Mean test 1 passed, the mean deviation is :" << std::abs(mean-8.5) << " (should be near 0)";
    }

    vec.push_back(200);
    mean = SimpleMath<float>::get_mean(vec);

    if(std::abs(mean-46.8)>_max)
    {
        qWarning() << "Mean test 2 failed, the mean deviation is :" <<std::abs(mean-46.8) << " (should be near 0)";
    } else
    {
        qDebug() << "Mean test 2 passed, the mean deviation is :" << std::abs(mean-46.8) << " (should be near 0)";
    }
}
