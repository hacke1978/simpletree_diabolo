#include "improvefit.h"

const float ImproveFit::_OCTREE_RESOLUTION = 0.02f;
const int ImproveFit::_MIN_PTS_CYLINDER_IMPROVEMENT = 3;
const float ImproveFit::_MIN_DIST_TO_CYLINDER = 0.03f;
const float ImproveFit::_CYLINDER_SIZE_MULTIPLIER = 1.1f;
const  float ImproveFit::_MIN_RADIUS_MULTIPLIER = 0.5f;
const float ImproveFit::_MAX_RADIUS_MULTIPLIER = 1.5f;

ImproveFit::ImproveFit(QSharedPointer<Tree> tree, PointCloudI::Ptr cloud)
{
    _tree = tree;
    _cloud = cloud;
    _octree.reset (new  pcl::octree::OctreePointCloudSearch<PointI> (_OCTREE_RESOLUTION));
    _octree->setInputCloud(_cloud);
    _octree->addPointsFromInputCloud();
}

PointCloudI::Ptr ImproveFit::extract_points_near_cylinder(QSharedPointer<Cylinder> cylinder)
{
    PointCloudI::Ptr output_cloud (new PointCloudI);
    QSharedPointer<PointI> query_point = cylinder->get_center();
    float cylinder_size = cylinder->get_half_size();
    float radius = qMax<float>(cylinder_size+_MIN_DIST_TO_CYLINDER, cylinder_size*_CYLINDER_SIZE_MULTIPLIER);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    _octree->radiusSearch (*query_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    output_cloud->points.resize(pointIdxRadiusSearch.size());
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
    {
        output_cloud->points.at(i) = _cloud->points.at(pointIdxRadiusSearch.at(i));
    }
    return output_cloud;
}

void ImproveFit::improve()
{
    QVector<QSharedPointer<Cylinder> > cylinders = _tree->get_all_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        PointCloudI::Ptr cloud = extract_points_near_cylinder(cylinder);
        if(cloud->points.size() > _MIN_PTS_CYLINDER_IMPROVEMENT)
        {
            pcl::ModelCoefficients ransac_coeff = RANSAC(cloud);
            if(ransac_coeff.values.size()==6)
            {
                if(ransac_coeff.values.at(6)<cylinder->get_radius()*_MAX_RADIUS_MULTIPLIER&&ransac_coeff.values.at(6)>cylinder->get_radius()*_MIN_RADIUS_MULTIPLIER)
                {
                    improve_with_RANSAC(cylinder,ransac_coeff);
                    cylinder->setImprovement(ImprovementType::RANSAC);
                } else
                {
                    improve_with_median(cylinder,cloud);
                    cylinder->setImprovement(ImprovementType::MEDIAN);
                }
            }
        } else
        {
            cylinder->setImprovement(ImprovementType::NO);
        }
    }
}

pcl::ModelCoefficients ImproveFit::RANSAC(pcl::PointCloud::Ptr cloud)
{
   MethodCoefficients cf;
    pcl::ModelCoefficients::Ptr coeff ( new pcl::ModelCoefficients );
    pcl::SACSegmentationFromNormals<PointI, PointI> seg;
    seg.setOptimizeCoefficients ( true );
    seg.setModelType ( pcl::SACMODEL_CYLINDER );
    seg.setMethodType (cf.ransac_type);
    seg.setNormalDistanceWeight ( 0.12 );
    seg.setMaxIterations ( 100 );
    seg.setDistanceThreshold ( 0.03 );
    seg.setRadiusLimits ( 0, 2.2 );
    seg.setInputCloud ( cylinderPoints );
    seg.setInputNormals ( cylinderPoints );
    pcl::PointIndices::Ptr inliers_cylinder ( new pcl::PointIndices );
    seg.segment ( *inliers_cylinder, *coeff );
    return coeff;
}

void ImproveFit::improve_with_RANSAC(QSharedPointer<Cylinder> cylinder, pcl::ModelCoefficients coeff)
{
    QSharedPointer<PointI> start = cylinder->get_start();
    QSharedPointer<PointI> end  = cylinder->get_end();

    QSharedPointer<Cylinder> new_cylinder (new Cylinder(coeff));
    QSharedPointer<PointI> start2 =  new_cylinder->projection_on_axis(start);
    QSharedPointer<PointI> end2 =  new_cylinder->projection_on_axis(end);

    float x, y, z, x2, y2, z2, r;

    x = start2->x;
    y = start2->y;
    z = start2->z;
    x2 = end2->x - start2->x;
    y2 = end2->y - start2->y;
    z2 = end2->z - start2->z;
    r = coeff->values.at(6);

    cylinder->values.resize(7);
    cylinder->values.at(0) =  x ;
    cylinder->values.at(1) =  y ;
    cylinder->values.at(2) =  z ;
    cylinder->values.at(3) =  x2 ;
    cylinder->values.at(4) =  y2 ;
    cylinder->values.at(5) =  z2 ;
    cylinder->values.at(6) =  r ;
}

void ImproveFit::improve_with_median(QSharedPointer<Cylinder> cylinder, pcl::PointCloud::Ptr cloud)
{

}


