#include "testcylinder.h"
     const float TestCylinder::_MAX_DIST = 0.0001f;


TestCylinder::TestCylinder()
{
    test_volume();
    test_distance();

}

void TestCylinder::test_distance()
{

        pcl::ModelCoefficients coeff;
        coeff.values.push_back(3);
        coeff.values.push_back(5);
        coeff.values.push_back(-1);
        coeff.values.push_back(1);
        coeff.values.push_back(10);
        coeff.values.push_back(11);
        coeff.values.push_back(1);

        QSharedPointer<PointI> p (new PointI);
        p->x = 1;
        p->y = 0;
        p->z = 0;


        QSharedPointer<Cylinder> cylinder (new Cylinder(coeff));


        qDebug() << "Starting test for Cylinder, check distance to point.";
        float dist = cylinder->dist_to_point(p);

//        Q_ASSERT(std::abs(dist-4.63987)>_max_dist);
//        assert(1==2);
//        qDebug() << "Distance test passed" << std::abs(dist-4.63987);

        if(std::abs(dist-4.63987)>_MAX_DIST)
        {
            qWarning() << "Distance test failed, the distance deviation is :" << std::abs(dist-4.63987) << " (should be near 0)";
        } else
        {
            qDebug() << "Distance test passed, the distance deviation is :" << std::abs(dist-4.63987) << " (should be near 0)";
        }


}

void TestCylinder::test_volume()
{
    pcl::ModelCoefficients coeff;
    coeff.values.push_back(3);
    coeff.values.push_back(5);
    coeff.values.push_back(-1);
    coeff.values.push_back(1);
    coeff.values.push_back(10);
    coeff.values.push_back(11);
    coeff.values.push_back(1);



    QSharedPointer<Cylinder> cylinder (new Cylinder(coeff));


    qDebug() << "Starting test for Cylinder, check volume of cylinder.";

    float vol = cylinder->get_volume();


    if(std::abs(vol-46.8087)>_MAX_DIST)
    {
        qWarning() << "Volume test failed, the volume deviation is :" << std::abs(vol-46.8087) << " (should be near 0)";
    } else
    {
        qDebug() << "Volume test passed, the volume deviation is :" << std::abs(vol-46.8087) << " (should be near 0)";
    }
}
