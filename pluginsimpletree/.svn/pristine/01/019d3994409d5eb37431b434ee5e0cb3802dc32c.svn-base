#include "buildtree.h"

QSharedPointer<Segment> BuildTree::getRoot_segment() const
{
    return _root_segment;
}

void BuildTree::generate_cylinders()
{
    _cylinders.clear();
//    _cylinders.resize(_coeff.size());
    for(size_t i = 0; i < _coeff.size(); i++)
    {
        QSharedPointer<Cylinder> cylinder (new Cylinder(_coeff.at(i)));
//        _cylinders.at(i) = cylinder;
        _cylinders.push_back(cylinder);
    }
}

void BuildTree::generate_octree()
{
    _octree.reset(new pcl::octree::OctreePointCloudSearch<PointI> (0.02f));
    QVectorIterator<QSharedPointer<Cylinder> > it (_cylinders);
    PointCloudI::Ptr cloud (new PointCloudI);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        QSharedPointer<PointI> start = cylinder->get_start();
        cloud->push_back(*start);
    }
    _octree->setInputCloud(cloud);
    _octree->addPointsFromInputCloud();
}

const QVector<QSharedPointer<Cylinder> > BuildTree::get_child_cylinders(const QSharedPointer<Cylinder> cylinder) const
{


    QVector<QSharedPointer<Cylinder> > children;
    QVector<QSharedPointer<Cylinder> > candidates;
    QSharedPointer<PointI> end = cylinder->get_end();

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 0.01f;

    if (_octree->radiusSearch (*end, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            candidates.push_back(_cylinders.at(pointIdxRadiusSearch.at(i)));
    }
    QVectorIterator<QSharedPointer<Cylinder> > it (candidates);
    while(it.hasNext())
    {

        QSharedPointer<Cylinder> candidate = it.next();
        if(candidate->is_child_of(cylinder))
        {
            children.push_back(candidate);
        }
    }
    return children;
}


BuildTree::BuildTree(std::vector<pcl::ModelCoefficients> coeff)
{
    _coeff = coeff;
    generate_cylinders();
    generate_octree();
    _root_segment.reset(new Segment);
    QSharedPointer<Cylinder> root_cylinder = _cylinders.at(0);
    _root_segment->add_cylinder(root_cylinder);
    build_tree(root_cylinder,_root_segment);
}

void BuildTree::build_tree(QSharedPointer<Cylinder> cylinder, QSharedPointer<Segment> segment)
{
        QVector<QSharedPointer<Cylinder> > children = get_child_cylinders(cylinder);
        if(children.size()>0)
        {

            if(children.size()==1)
            {
                segment->add_cylinder(children.at(0));
                build_tree(children.at(0), segment);
            } else {
                QVectorIterator<QSharedPointer<Cylinder> > it( children);
                while(it.hasNext())
                {
                    QSharedPointer<Cylinder> child = it.next();
                    QSharedPointer<Segment> child_segment (new Segment);
                    child_segment->add_cylinder(child);
                    segment->add_child_segment(child_segment);
                    build_tree(child, child_segment);
                }
            }
        }
}
