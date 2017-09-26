#include "testsegment.h"

const float TestSegment::_MAX_DISTANCE = 0.001;

TestSegment::TestSegment()
{
    test_add_cylinder();
    test__child_connection();
    test_leave_root();
    test_median_smooth();
   // test_mean_smooth();
    test_merge();

}

void TestSegment::test_volume()
{
    pcl::ModelCoefficients coeff1;
    coeff1.values.push_back(3);
    coeff1.values.push_back(5);
    coeff1.values.push_back(-1);
    coeff1.values.push_back(1);
    coeff1.values.push_back(10);
    coeff1.values.push_back(11);
    coeff1.values.push_back(1);
    QSharedPointer<Cylinder> cylinder1 (new Cylinder(coeff1));


    pcl::ModelCoefficients coeff2;
    coeff2.values.push_back(3);
    coeff2.values.push_back(5);
    coeff2.values.push_back(-1);
    coeff2.values.push_back(1);
    coeff2.values.push_back(10);
    coeff2.values.push_back(11);
    coeff2.values.push_back(1);
    QSharedPointer<Cylinder> cylinder2 (new Cylinder(coeff2));

    pcl::ModelCoefficients coeff3;
    coeff3.values.push_back(3);
    coeff3.values.push_back(5);
    coeff3.values.push_back(-1);
    coeff3.values.push_back(1);
    coeff3.values.push_back(10);
    coeff3.values.push_back(11);
    coeff3.values.push_back(1);
    QSharedPointer<Cylinder> cylinder3 (new Cylinder(coeff3));

    QSharedPointer<Segment> segment1 (new Segment);


    qDebug() << "Starting test for segment volume.";
    segment1->add_cylinder(cylinder1);
    segment1->add_cylinder(cylinder2);
    segment1->add_cylinder(cylinder3);
    if(std::abs(segment1->get_volume()-140.4261)>_MAX_DISTANCE)
    {
        qWarning() << "Volume test 1 failed, volume deviation is  :" << std::abs(segment1->get_volume()-140.4261) << " (should be near zero)";
    } else
    {
        qWarning() << "Volume test 1 passed, volume deviation is  :" << std::abs(segment1->get_volume()-140.4261) << " (should be near zero)";

    }
}

void TestSegment::test_add_cylinder()
{
    pcl::ModelCoefficients coeff1;
    coeff1.values.push_back(3);
    coeff1.values.push_back(5);
    coeff1.values.push_back(-1);
    coeff1.values.push_back(1);
    coeff1.values.push_back(10);
    coeff1.values.push_back(11);
    coeff1.values.push_back(1);
    QSharedPointer<Cylinder> cylinder1 (new Cylinder(coeff1));


    pcl::ModelCoefficients coeff2;
    coeff2.values.push_back(3);
    coeff2.values.push_back(5);
    coeff2.values.push_back(-1);
    coeff2.values.push_back(1);
    coeff2.values.push_back(10);
    coeff2.values.push_back(11);
    coeff2.values.push_back(1);
    QSharedPointer<Cylinder> cylinder2 (new Cylinder(coeff2));

    pcl::ModelCoefficients coeff3;
    coeff3.values.push_back(3);
    coeff3.values.push_back(5);
    coeff3.values.push_back(-1);
    coeff3.values.push_back(1);
    coeff3.values.push_back(10);
    coeff3.values.push_back(11);
    coeff3.values.push_back(1);
    QSharedPointer<Cylinder> cylinder3 (new Cylinder(coeff3));

    QSharedPointer<Segment> segment1 (new Segment);


    qDebug() << "Starting test for adding cylinders to the Segment.";
    segment1->add_cylinder(cylinder1);
    segment1->add_cylinder(cylinder2);
    segment1->add_cylinder(cylinder3);
    qDebug() << "3 Different Cylinders have been added.";
    if(segment1->get_cylinders().size()!=3)
    {
        qWarning() << "Segment cylinder adding test 1 failed, the number of cylinders is  :" << segment1->get_cylinders().size() << " (should be 3)";
    } else
    {
        qDebug() << "Segment cylinder adding test 1 passed, the number of cylinders is  :" << segment1->get_cylinders().size() << " (should be 3)";
    }

    qDebug() << "One time the same cylinder is added.";
    segment1->add_cylinder(cylinder3);
    if(segment1->get_cylinders().size()!=3)
    {
        qWarning() << "Segment cylinder adding test 1 failed, the number of cylinders is  :" << segment1->get_cylinders().size() << " (should be 3)";
    } else
    {
        qDebug() << "Segment cylinder adding test 1 passed, the number of cylinders is  :" << segment1->get_cylinders().size() << " (should be 3)";
    }

    qDebug() << "Another cylinder (a forth) is added.";
    pcl::ModelCoefficients coeff4;
    coeff4.values.push_back(3);
    coeff4.values.push_back(5);
    coeff4.values.push_back(-1);
    coeff4.values.push_back(1);
    coeff4.values.push_back(10);
    coeff4.values.push_back(11);
    coeff4.values.push_back(1);
    QSharedPointer<Cylinder> cylinder4 (new Cylinder(coeff4));

    segment1->add_cylinder(cylinder4);
    if(segment1->get_cylinders().size()!=4)
    {
        qWarning() << "Segment cylinder adding test 1 failed, the number of cylinders is  :" << segment1->get_cylinders().size() << " (should be 4)";
    } else
    {
        qDebug() << "Segment cylinder adding test 1 passed, the number of cylinders is  :" << segment1->get_cylinders().size() << " (should be 4)";
    }


}

void TestSegment::test__child_connection()
{
    pcl::ModelCoefficients coeff1;
    coeff1.values.push_back(3);
    coeff1.values.push_back(5);
    coeff1.values.push_back(-1);
    coeff1.values.push_back(1);
    coeff1.values.push_back(10);
    coeff1.values.push_back(11);
    coeff1.values.push_back(1);
    QSharedPointer<Cylinder> cylinder1 (new Cylinder(coeff1));
    QSharedPointer<Segment> segment1 (new Segment);
    segment1->add_cylinder(cylinder1);


    pcl::ModelCoefficients coeff2;
    coeff2.values.push_back(3);
    coeff2.values.push_back(5);
    coeff2.values.push_back(-1);
    coeff2.values.push_back(1);
    coeff2.values.push_back(10);
    coeff2.values.push_back(11);
    coeff2.values.push_back(1);
    QSharedPointer<Cylinder> cylinder2 (new Cylinder(coeff2));
    QSharedPointer<Segment> segment2 (new Segment);
    segment2->add_cylinder(cylinder2);

    pcl::ModelCoefficients coeff3;
    coeff3.values.push_back(3);
    coeff3.values.push_back(5);
    coeff3.values.push_back(-1);
    coeff3.values.push_back(1);
    coeff3.values.push_back(10);
    coeff3.values.push_back(11);
    coeff3.values.push_back(1);
    QSharedPointer<Cylinder> cylinder3 (new Cylinder(coeff3));
    QSharedPointer<Segment> segment3 (new Segment);
    segment3->add_cylinder(cylinder3);




    qDebug() << "Starting test for adding child segments.";

    segment1->add_child_segment(segment2);
    segment1->add_child_segment(segment3);
    qDebug() << "Added two child segments.";
    if(segment1->get_child_segments().size()!=2)
    {
        qWarning() << "Segment cylinder adding child segments test 1 failed, the number of children is  :" << segment1->get_child_segments().size() << " (should be 2)";
    } else
    {
        qDebug() << "Segment cylinder adding child segments test 1 passed, the number of children is  :" << segment1->get_child_segments().size() << " (should be 2)";
    }

    segment1->add_child_segment(segment2);

    qDebug() << "Added the same segment another time.";
    if(segment1->get_child_segments().size()!=2)
    {
        qWarning() << "Segment cylinder adding child segments test 2 failed, the number of children is  :" << segment1->get_child_segments().size() << " (should be 2)";
    } else
    {
        qDebug() << "Segment cylinder adding child segments test 2 passed, the number of children is  :" << segment1->get_child_segments().size() << " (should be 2)";
    }

    qDebug() << "Removing segment2.";
    segment2->remove();
    if(segment1->get_child_segments().size()!=1)
    {
        qWarning() << "Segment cylinder removing child segments test 3 failed, the number of children is  :" << segment1->get_child_segments().size() << " (should be 1)";
    } else
    {
        qDebug() << "Segment cylinder removing child segments test 3 passed, the number of children is  :" << segment1->get_child_segments().size() << " (should be 1)";
    }

    segment1->add_child_segment(segment2);

    qDebug() << "Added the same segment another time.";
    if(segment1->get_child_segments().size()!=2)
    {
        qWarning() << "Segment cylinder adding child segments test 2 failed, the number of children is  :" << segment1->get_child_segments().size() << " (should be 2)";
    } else
    {
        qDebug() << "Segment cylinder adding child segments test 2 passed, the number of children is  :" << segment1->get_child_segments().size() << " (should be 2)";
    }




}

void TestSegment::test_remove()
{

}

void TestSegment::test_leave_root()
{
    pcl::ModelCoefficients coeff1;
    coeff1.values.push_back(3);
    coeff1.values.push_back(5);
    coeff1.values.push_back(-1);
    coeff1.values.push_back(1);
    coeff1.values.push_back(10);
    coeff1.values.push_back(11);
    coeff1.values.push_back(1);
    QSharedPointer<Cylinder> cylinder1 (new Cylinder(coeff1));
    QSharedPointer<Segment> segment1 (new Segment);
    segment1->add_cylinder(cylinder1);


    pcl::ModelCoefficients coeff2;
    coeff2.values.push_back(3);
    coeff2.values.push_back(5);
    coeff2.values.push_back(-1);
    coeff2.values.push_back(1);
    coeff2.values.push_back(10);
    coeff2.values.push_back(11);
    coeff2.values.push_back(1);
    QSharedPointer<Cylinder> cylinder2 (new Cylinder(coeff2));
    QSharedPointer<Segment> segment2 (new Segment);
    segment2->add_cylinder(cylinder2);

    pcl::ModelCoefficients coeff3;
    coeff3.values.push_back(3);
    coeff3.values.push_back(5);
    coeff3.values.push_back(-1);
    coeff3.values.push_back(1);
    coeff3.values.push_back(10);
    coeff3.values.push_back(11);
    coeff3.values.push_back(1);
    QSharedPointer<Cylinder> cylinder3 (new Cylinder(coeff3));
    QSharedPointer<Segment> segment3 (new Segment);
    segment3->add_cylinder(cylinder3);




    qDebug() << "Starting test for leaf root detection.";

    segment1->add_child_segment(segment2);
    segment1->add_child_segment(segment3);

    if(!segment1->is_root())
    {
        qWarning() << "Root test 1 failed, Segment 1 should be root"<< segment1->is_root();
    } else
    {
        qDebug() << "Root test 1 passed, Segment 1 is root" << segment1->is_root();
    }

    if(segment2->is_root())
    {
        qWarning() << "Root test 2 failed, Segment 2 should not be root";
    } else
    {
        qDebug() << "Root test 2 passed, Segment 2 is not root" << segment2->is_root();
    }


    if(segment1->is_leave())
    {
        qWarning() << "Leave test 1 failed, Segment 1 is leave"<< segment1->is_leave();
    } else
    {
        qDebug() << "Leave test 1 passed, Segment 1 is not leave" << segment1->is_leave();
    }

    if(!segment2->is_leave())
    {
        qWarning() << "Leave test 2 failed, Segment 2 is not leave"<< segment2->is_leave();
    } else
    {
        qDebug() << "Leave test 2 passed, Segment 2 is leave" << segment2->is_leave();
    }

    qDebug() << "all children are removed";
    segment2->remove();
    segment3->remove();


    if(!segment1->is_leave())
    {
        qWarning() << "Leave test 3 failed, Segment 1 is  not leave"<< segment1->is_leave();
    } else
    {
        qDebug() << "Leave test 3 passed, Segment 1 is  leave" << segment1->is_leave();
    }




}

void TestSegment::test_median_smooth()
{
    QSharedPointer<Segment> segment1 (new Segment);

    pcl::ModelCoefficients coeff1;
    coeff1.values.push_back(3);
    coeff1.values.push_back(5);
    coeff1.values.push_back(-1);
    coeff1.values.push_back(1);
    coeff1.values.push_back(10);
    coeff1.values.push_back(11);
    coeff1.values.push_back(1);
    QSharedPointer<Cylinder> cylinder1 (new Cylinder(coeff1));

    segment1->add_cylinder(cylinder1);


    pcl::ModelCoefficients coeff2;
    coeff2.values.push_back(3);
    coeff2.values.push_back(5);
    coeff2.values.push_back(-1);
    coeff2.values.push_back(1);
    coeff2.values.push_back(10);
    coeff2.values.push_back(11);
    coeff2.values.push_back(2);
    QSharedPointer<Cylinder> cylinder2 (new Cylinder(coeff2));
    segment1->add_cylinder(cylinder2);

    pcl::ModelCoefficients coeff3;
    coeff3.values.push_back(3);
    coeff3.values.push_back(5);
    coeff3.values.push_back(-1);
    coeff3.values.push_back(1);
    coeff3.values.push_back(10);
    coeff3.values.push_back(11);
    coeff3.values.push_back(4);
    QSharedPointer<Cylinder> cylinder3 (new Cylinder(coeff3));
    segment1->add_cylinder(cylinder3);

    QSharedPointer<Tree> tree(new Tree(segment1, "as"));
    float volume_old = tree->get_volume();
    ImproveByMedian improve(tree);

    float volume_new = tree->get_volume();


    qDebug() << "Testing the median smooth routine";



    if(std::abs(volume_old/volume_new*12/21-1) >_MAX_DISTANCE)
    {
        qWarning() << "Smooth routine test 1 failed";
    } else
    {
        qDebug() << "Smooth routine test 1  passed";
    }


}

void TestSegment::test_mean_smooth()
{
    QSharedPointer<Segment> segment1 (new Segment);

    pcl::ModelCoefficients coeff1;
    coeff1.values.push_back(3);
    coeff1.values.push_back(5);
    coeff1.values.push_back(-1);
    coeff1.values.push_back(1);
    coeff1.values.push_back(10);
    coeff1.values.push_back(11);
    coeff1.values.push_back(1);
    QSharedPointer<Cylinder> cylinder1 (new Cylinder(coeff1));

    segment1->add_cylinder(cylinder1);


    pcl::ModelCoefficients coeff2;
    coeff2.values.push_back(3);
    coeff2.values.push_back(5);
    coeff2.values.push_back(-1);
    coeff2.values.push_back(1);
    coeff2.values.push_back(10);
    coeff2.values.push_back(11);
    coeff2.values.push_back(2);
    QSharedPointer<Cylinder> cylinder2 (new Cylinder(coeff2));
    segment1->add_cylinder(cylinder2);

    pcl::ModelCoefficients coeff3;
    coeff3.values.push_back(3);
    coeff3.values.push_back(5);
    coeff3.values.push_back(-1);
    coeff3.values.push_back(1);
    coeff3.values.push_back(10);
    coeff3.values.push_back(11);
    coeff3.values.push_back(4);
    QSharedPointer<Cylinder> cylinder3 (new Cylinder(coeff3));
    segment1->add_cylinder(cylinder3);



    QSharedPointer<Tree> tree(new Tree(segment1, "as"));
    float volume_old = tree->get_volume();
    ImproveByMedian improve(tree);


            float volume_new = tree->get_volume();


    qDebug() << "Testing the mean smooth routine";



    if(std::abs(volume_old/volume_new*14.88888f/21-1) >_MAX_DISTANCE)
    {
        qWarning() << "Smooth mean routine test 1 failed";
    } else
    {
        qDebug() << "Smooth mean routine test 1  passed";
    }
}

void TestSegment::test_merge()
{
    QSharedPointer<Segment> segment1 (new Segment);

    pcl::ModelCoefficients coeff1;
    coeff1.values.push_back(3);
    coeff1.values.push_back(5);
    coeff1.values.push_back(-1);
    coeff1.values.push_back(1);
    coeff1.values.push_back(10);
    coeff1.values.push_back(11);
    coeff1.values.push_back(1);
    QSharedPointer<Cylinder> cylinder1 (new Cylinder(coeff1));

    segment1->add_cylinder(cylinder1);


    pcl::ModelCoefficients coeff2;
    coeff2.values.push_back(4);
    coeff2.values.push_back(15);
    coeff2.values.push_back(10);
    coeff2.values.push_back(1);
    coeff2.values.push_back(10);
    coeff2.values.push_back(11);
    coeff2.values.push_back(2);
    QSharedPointer<Cylinder> cylinder2 (new Cylinder(coeff2));
    segment1->add_cylinder(cylinder2);

    pcl::ModelCoefficients coeff3;
    coeff3.values.push_back(5);
    coeff3.values.push_back(25);
    coeff3.values.push_back(21);
    coeff3.values.push_back(1);
    coeff3.values.push_back(10);
    coeff3.values.push_back(11);
    coeff3.values.push_back(4);
    QSharedPointer<Cylinder> cylinder3 (new Cylinder(coeff3));
    segment1->add_cylinder(cylinder3);

    QVectorIterator<QSharedPointer<Cylinder> > it (segment1->get_cylinders());
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        qDebug() << cylinder->to_string();
    }

    segment1->merge();

    QVectorIterator<QSharedPointer<Cylinder> > it2 (segment1->get_cylinders());
    while(it2.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it2.next();
        qDebug() << cylinder->to_string();
    }
}
