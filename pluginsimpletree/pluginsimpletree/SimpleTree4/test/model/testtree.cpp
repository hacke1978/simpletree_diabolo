#include "testtree.h"

TestTree::TestTree()
{
    test_volume();
    test_build_tree();
    test_remove_false_cylinders();
    test_reorder_tree();
}

void TestTree::test_volume()
{
    pcl::ModelCoefficients coeff1;
    coeff1.values.push_back(0);
    coeff1.values.push_back(0);
    coeff1.values.push_back(1);
    coeff1.values.push_back(1);
    coeff1.values.push_back(0);
    coeff1.values.push_back(0);
    coeff1.values.push_back(1);
    QSharedPointer<Cylinder> cylinder1 (new Cylinder(coeff1));
    QSharedPointer<Cylinder> cylinder2 (new Cylinder(coeff1));
    QSharedPointer<Cylinder> cylinder3 (new Cylinder(coeff1));

    QSharedPointer<Cylinder> cylinder4 (new Cylinder(coeff1));
    QSharedPointer<Cylinder> cylinder5 (new Cylinder(coeff1));
    QSharedPointer<Cylinder> cylinder6 (new Cylinder(coeff1));

    QSharedPointer<Cylinder> cylinder7 (new Cylinder(coeff1));
    QSharedPointer<Cylinder> cylinder8 (new Cylinder(coeff1));
    QSharedPointer<Cylinder> cylinder9 (new Cylinder(coeff1));

    QSharedPointer<Segment> segment1 (new Segment);
    segment1->add_cylinder(cylinder1);
    segment1->add_cylinder(cylinder2);
    segment1->add_cylinder(cylinder3);

    QSharedPointer<Segment> segment2 (new Segment);
    segment2->add_cylinder(cylinder4);
    segment2->add_cylinder(cylinder5);
    segment2->add_cylinder(cylinder6);

    QSharedPointer<Segment> segment3 (new Segment);
    segment3->add_cylinder(cylinder7);
    segment3->add_cylinder(cylinder8);

    segment1->add_child_segment(segment2);
    segment1->add_child_segment(segment3);

    QSharedPointer<Tree> tree (new Tree(segment1, "foo"));

    qDebug() << tree->get_volume()/cylinder1->get_volume();
    qDebug() << tree->get_growth_volume(cylinder2)/cylinder1->get_volume();
    segment3->merge();
    qDebug() << tree->get_growth_volume(cylinder2)/cylinder1->get_volume();

}

void TestTree::test_build_tree()
{
    pcl::ModelCoefficients coeff1;
    coeff1.values.push_back(0);
    coeff1.values.push_back(0);
    coeff1.values.push_back(0);
    coeff1.values.push_back(0);
    coeff1.values.push_back(0);
    coeff1.values.push_back(1);
    coeff1.values.push_back(1);

    pcl::ModelCoefficients coeff2;
    coeff2.values.push_back(0);
    coeff2.values.push_back(0);
    coeff2.values.push_back(1);
    coeff2.values.push_back(0);
    coeff2.values.push_back(0);
    coeff2.values.push_back(1);
    coeff2.values.push_back(1);

    pcl::ModelCoefficients coeff3;
    coeff3.values.push_back(0);
    coeff3.values.push_back(0);
    coeff3.values.push_back(2);
    coeff3.values.push_back(0);
    coeff3.values.push_back(0);
    coeff3.values.push_back(1);
    coeff3.values.push_back(1);


    pcl::ModelCoefficients coeff4;
    coeff4.values.push_back(0);
    coeff4.values.push_back(0);
    coeff4.values.push_back(3);
    coeff4.values.push_back(0);
    coeff4.values.push_back(0);
    coeff4.values.push_back(1);
    coeff4.values.push_back(1);

    pcl::ModelCoefficients coeff5;
    coeff5.values.push_back(0);
    coeff5.values.push_back(0);
    coeff5.values.push_back(4);
    coeff5.values.push_back(0);
    coeff5.values.push_back(0);
    coeff5.values.push_back(1);
    coeff5.values.push_back(1);

    pcl::ModelCoefficients coeff6;
    coeff6.values.push_back(0);
    coeff6.values.push_back(0);
    coeff6.values.push_back(5);
    coeff6.values.push_back(0);
    coeff6.values.push_back(0);
    coeff6.values.push_back(1);
    coeff6.values.push_back(1);

    pcl::ModelCoefficients coeff7;
    coeff7.values.push_back(0);
    coeff7.values.push_back(0);
    coeff7.values.push_back(3);
    coeff7.values.push_back(0);
    coeff7.values.push_back(1);
    coeff7.values.push_back(0);
    coeff7.values.push_back(1);

    pcl::ModelCoefficients coeff8;
    coeff8.values.push_back(0);
    coeff8.values.push_back(0);
    coeff8.values.push_back(3);
    coeff8.values.push_back(0);
    coeff8.values.push_back(-1);
    coeff8.values.push_back(0);
    coeff8.values.push_back(1);

    pcl::ModelCoefficients coeff9;
    coeff9.values.push_back(0);
    coeff9.values.push_back(-1);
    coeff9.values.push_back(3);
    coeff9.values.push_back(0);
    coeff9.values.push_back(-1);
    coeff9.values.push_back(0);
    coeff9.values.push_back(1);


    std::vector<pcl::ModelCoefficients> coeff;
    coeff.push_back(coeff1);
    coeff.push_back(coeff2);
    coeff.push_back(coeff3);
    coeff.push_back(coeff4);
    coeff.push_back(coeff5);
    coeff.push_back(coeff6);
    coeff.push_back(coeff7);
    coeff.push_back(coeff8);
    coeff.push_back(coeff9);

    BuildTree build(coeff);

    QSharedPointer<Tree> tree(new Tree(build.getRoot_segment(),"ff"));
    if(std::abs(tree->get_volume()/9-3.14) >0.1)
    {
        qWarning() << "Build Tree has failed";
    } else
    {
        qDebug() << "Build Tree was successfull";
    }


}

void TestTree::test_remove_false_cylinders()
{
    pcl::ModelCoefficients coeff1;
    coeff1.values.push_back(0);
    coeff1.values.push_back(0);
    coeff1.values.push_back(0);
    coeff1.values.push_back(0);
    coeff1.values.push_back(0);
    coeff1.values.push_back(1);
    coeff1.values.push_back(1);

    pcl::ModelCoefficients coeff2;
    coeff2.values.push_back(0);
    coeff2.values.push_back(0);
    coeff2.values.push_back(1);
    coeff2.values.push_back(0);
    coeff2.values.push_back(0);
    coeff2.values.push_back(1);
    coeff2.values.push_back(1);

    pcl::ModelCoefficients coeff3;
    coeff3.values.push_back(0);
    coeff3.values.push_back(0);
    coeff3.values.push_back(2);
    coeff3.values.push_back(0);
    coeff3.values.push_back(0);
    coeff3.values.push_back(1);
    coeff3.values.push_back(1);


    pcl::ModelCoefficients coeff4;
    coeff4.values.push_back(0);
    coeff4.values.push_back(0);
    coeff4.values.push_back(3);
    coeff4.values.push_back(0);
    coeff4.values.push_back(0);
    coeff4.values.push_back(1);
    coeff4.values.push_back(1);

    pcl::ModelCoefficients coeff5;
    coeff5.values.push_back(0);
    coeff5.values.push_back(0);
    coeff5.values.push_back(4);
    coeff5.values.push_back(0);
    coeff5.values.push_back(0);
    coeff5.values.push_back(1);
    coeff5.values.push_back(1);

    pcl::ModelCoefficients coeff6;
    coeff6.values.push_back(0);
    coeff6.values.push_back(0);
    coeff6.values.push_back(5);
    coeff6.values.push_back(0);
    coeff6.values.push_back(0);
    coeff6.values.push_back(1);
    coeff6.values.push_back(1);

    pcl::ModelCoefficients coeff7;
    coeff7.values.push_back(0);
    coeff7.values.push_back(0);
    coeff7.values.push_back(3);
    coeff7.values.push_back(0);
    coeff7.values.push_back(1);
    coeff7.values.push_back(0);
    coeff7.values.push_back(1);

    pcl::ModelCoefficients coeff8;
    coeff8.values.push_back(0);
    coeff8.values.push_back(0);
    coeff8.values.push_back(3);
    coeff8.values.push_back(0);
    coeff8.values.push_back(-1);
    coeff8.values.push_back(0);
    coeff8.values.push_back(1);

    pcl::ModelCoefficients coeff9;
    coeff9.values.push_back(0);
    coeff9.values.push_back(-1);
    coeff9.values.push_back(3);
    coeff9.values.push_back(0);
    coeff9.values.push_back(-1);
    coeff9.values.push_back(0);
    coeff9.values.push_back(1);

    pcl::ModelCoefficients coeff10;
    coeff10.values.push_back(0);
    coeff10.values.push_back(-2);
    coeff10.values.push_back(3);
    coeff10.values.push_back(0);
    coeff10.values.push_back(-1);
    coeff10.values.push_back(0);
    coeff10.values.push_back(1);

    pcl::ModelCoefficients coeff11;
    coeff11.values.push_back(0);
    coeff11.values.push_back(-3);
    coeff11.values.push_back(3);
    coeff11.values.push_back(0);
    coeff11.values.push_back(-1);
    coeff11.values.push_back(0);
    coeff11.values.push_back(1);

    pcl::ModelCoefficients coeff12;
    coeff12.values.push_back(0);
    coeff12.values.push_back(-2);
    coeff12.values.push_back(3);
    coeff12.values.push_back(0);
    coeff12.values.push_back(0);
    coeff12.values.push_back(1);
    coeff12.values.push_back(1);


    std::vector<pcl::ModelCoefficients> coeff;
    coeff.push_back(coeff1);
    coeff.push_back(coeff2);
    coeff.push_back(coeff3);
    coeff.push_back(coeff4);
    coeff.push_back(coeff5);
    coeff.push_back(coeff6);
    coeff.push_back(coeff7);
    coeff.push_back(coeff8);
    coeff.push_back(coeff9);
    coeff.push_back(coeff10);
    coeff.push_back(coeff11);
    coeff.push_back(coeff12);

    BuildTree build(coeff);

    QSharedPointer<Tree> tree(new Tree(build.getRoot_segment(),"ff"));
    RemoveFalseCylinders remove (tree);
    if(std::abs(tree->get_volume()/10-3.14) >0.1)
    {
        qWarning() << "Remove wrong cylinders from Tree has failed" ;
    } else
    {
        qDebug() << "Remove wrong cylinders from Tree was successfull";
    }
    QVector<QSharedPointer<Segment> > segments = tree->get_all_segments();
    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        // qDebug() << segment->to_string();
    }
    //plot(tree->get_root_segment());

}

void TestTree::test_reorder_tree()
{
    pcl::ModelCoefficients coeff1;
    coeff1.values.push_back(0);
    coeff1.values.push_back(0);
    coeff1.values.push_back(0);
    coeff1.values.push_back(0);
    coeff1.values.push_back(0);
    coeff1.values.push_back(1);
    coeff1.values.push_back(1);
    QSharedPointer<Cylinder> cyl1 (new Cylinder(coeff1));

    pcl::ModelCoefficients coeff2;
    coeff2.values.push_back(0);
    coeff2.values.push_back(0);
    coeff2.values.push_back(1);
    coeff2.values.push_back(0);
    coeff2.values.push_back(0);
    coeff2.values.push_back(1);
    coeff2.values.push_back(1);
    QSharedPointer<Cylinder> cyl2 (new Cylinder(coeff2));

    pcl::ModelCoefficients coeff3;
    coeff3.values.push_back(0);
    coeff3.values.push_back(0);
    coeff3.values.push_back(2);
    coeff3.values.push_back(0);
    coeff3.values.push_back(0);
    coeff3.values.push_back(1);
    coeff3.values.push_back(1);
    QSharedPointer<Cylinder> cyl3 (new Cylinder(coeff3));


    pcl::ModelCoefficients coeff4;
    coeff4.values.push_back(0);
    coeff4.values.push_back(0);
    coeff4.values.push_back(3);
    coeff4.values.push_back(0);
    coeff4.values.push_back(0);
    coeff4.values.push_back(1);
    coeff4.values.push_back(1);
    QSharedPointer<Cylinder> cyl4 (new Cylinder(coeff4));

    pcl::ModelCoefficients coeff5;
    coeff5.values.push_back(0);
    coeff5.values.push_back(0);
    coeff5.values.push_back(4);
    coeff5.values.push_back(0);
    coeff5.values.push_back(0);
    coeff5.values.push_back(1);
    coeff5.values.push_back(1);
    QSharedPointer<Cylinder> cyl5 (new Cylinder(coeff5));

    pcl::ModelCoefficients coeff6;
    coeff6.values.push_back(0);
    coeff6.values.push_back(0);
    coeff6.values.push_back(5);
    coeff6.values.push_back(0);
    coeff6.values.push_back(0);
    coeff6.values.push_back(1);
    coeff6.values.push_back(1);
    QSharedPointer<Cylinder> cyl6 (new Cylinder(coeff6));

    pcl::ModelCoefficients coeff7;
    coeff7.values.push_back(0);
    coeff7.values.push_back(0);
    coeff7.values.push_back(3);
    coeff7.values.push_back(0);
    coeff7.values.push_back(1);
    coeff7.values.push_back(0);
    coeff7.values.push_back(1);
    QSharedPointer<Cylinder> cyl7 (new Cylinder(coeff7));

    pcl::ModelCoefficients coeff8;
    coeff8.values.push_back(0);
    coeff8.values.push_back(0);
    coeff8.values.push_back(3);
    coeff8.values.push_back(0);
    coeff8.values.push_back(-1);
    coeff8.values.push_back(0);
    coeff8.values.push_back(1);
    QSharedPointer<Cylinder> cyl8 (new Cylinder(coeff8));

    pcl::ModelCoefficients coeff9;
    coeff9.values.push_back(0);
    coeff9.values.push_back(-1);
    coeff9.values.push_back(3);
    coeff9.values.push_back(0);
    coeff9.values.push_back(-1);
    coeff9.values.push_back(0);
    coeff9.values.push_back(1);
    QSharedPointer<Cylinder> cyl9 (new Cylinder(coeff9));

    pcl::ModelCoefficients coeff10;
    coeff10.values.push_back(0);
    coeff10.values.push_back(-2);
    coeff10.values.push_back(3);
    coeff10.values.push_back(0);
    coeff10.values.push_back(-1);
    coeff10.values.push_back(0);
    coeff10.values.push_back(1);
    QSharedPointer<Cylinder> cyl10 (new Cylinder(coeff10));

    pcl::ModelCoefficients coeff11;
    coeff11.values.push_back(0);
    coeff11.values.push_back(-3);
    coeff11.values.push_back(3);
    coeff11.values.push_back(0);
    coeff11.values.push_back(-1);
    coeff11.values.push_back(0);
    coeff11.values.push_back(1);
    QSharedPointer<Cylinder> cyl11 (new Cylinder(coeff11));

    pcl::ModelCoefficients coeff12;
    coeff12.values.push_back(0);
    coeff12.values.push_back(-2);
    coeff12.values.push_back(3);
    coeff12.values.push_back(0);
    coeff12.values.push_back(0);
    coeff12.values.push_back(1);
    coeff12.values.push_back(1);
    QSharedPointer<Cylinder> cyl12 (new Cylinder(coeff12));

    QSharedPointer<Segment> seg1 (new Segment);
    seg1->add_cylinder(cyl1);
    seg1->add_cylinder(cyl2);
    seg1->add_cylinder(cyl3);
    seg1->set_id(1);

    QSharedPointer<Segment> seg2 (new Segment);
    seg2->add_cylinder(cyl4);
    seg2->add_cylinder(cyl5);
    seg2->add_cylinder(cyl6);
    seg2->set_id(2);

    QSharedPointer<Segment> seg3 (new Segment);
    seg3->add_cylinder(cyl7);
    seg3->set_id(3);

    QSharedPointer<Segment> seg4 (new Segment);
    seg4->add_cylinder(cyl8);
    seg4->add_cylinder(cyl9);
    seg4->set_id(4);


    QSharedPointer<Segment> seg5 (new Segment);
    seg5->add_cylinder(cyl10);
    seg5->add_cylinder(cyl11);
    seg5->set_id(5);


    QSharedPointer<Segment> seg6 (new Segment);
    seg6->add_cylinder(cyl12);
    seg6->set_id(6);

    seg1->add_child_segment(seg2);
    seg1->add_child_segment(seg3);
    seg1->add_child_segment(seg4);
    seg4->add_child_segment(seg6);
    seg4->add_child_segment(seg5);

    QSharedPointer<Tree> tree(new Tree(seg1,"tree"));
    plot(tree->get_root_segment(), "");
    ReorderTree ro (tree);
    plot(tree->get_root_segment(), "");
}

void TestTree::plot(QSharedPointer<Segment> segment, QString begin)
{
     begin.append("  ");
    QString res (begin);
    QString res2 = res.append(segment->to_string());
    qDebug() << res2;
    QVector<QSharedPointer<Segment> > children = segment->get_child_segments();
    QVectorIterator<QSharedPointer<Segment> > it(children);
    while(it.hasNext())
    {
        QSharedPointer<Segment> child = it.next();
        plot(child, begin);
    }

}
