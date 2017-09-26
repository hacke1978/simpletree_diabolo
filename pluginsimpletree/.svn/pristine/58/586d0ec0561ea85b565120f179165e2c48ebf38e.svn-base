#include "spherefollowingrecursivest.h"

QVector<pcl::ModelCoefficients> SphereFollowingRecursiveST::get_cylinders() const
{
    return _cylinders;
}



MethodCoefficients SphereFollowingRecursiveST::get_coeff() const
{
    return _coeff;
}

void SphereFollowingRecursiveST::set_coeff(const MethodCoefficients &coeff)
{
    _coeff = coeff;
}

QSharedPointer<Tree> SphereFollowingRecursiveST::get_tree()
{
    return _tree;
}


void SphereFollowingRecursiveST::remove_attractor(PointS &attr)
{
    int index = -1;
    for(int i = 0; i < _attractors->points.size(); i++)
    {
        if(SimpleMath<float>::are_equal(_attractors->points.at(i),attr))
        {
            index = i;
        }
    }
    std::vector<PointS>::iterator it = _attractors->points.begin() + index;
    _attractors->points.erase(it);
    _list.removeAt(index);
}

void SphereFollowingRecursiveST::initiate_list()
{
    _list.clear();
    for(int i = 0; i< _attractors->points.size(); i++)
    {
        PointS attr = _attractors->points.at(i);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        _octree->nearestKSearch(attr,1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        PointS end_point = _end_pts->points.at(pointIdxRadiusSearch.at(0));
        float dist = std::sqrt(pointRadiusSquaredDistance.at(0));
        TripletST trip;
        trip.attr = attr;
        trip.end  = end_point;
        trip.dist = dist;
        _list.push_back(QVariant::fromValue(trip));
    }
}

void SphereFollowingRecursiveST::update_list(PointS &p)
{
    for(int i = 0; i < _list.size(); i++)
    {
        TripletST trip = _list.at(i).value<TripletST>();
        PointS attr = trip.attr;
        float dist = SimpleMath<float>::get_distance(p,attr);
        if(dist<trip.dist)
        {
            TripletST trip_new;
            trip_new.attr = attr;
            trip_new.end = p;
            trip_new.dist = dist;
            _list.replace(i,QVariant::fromValue(trip_new));
        }

    }
}

QPair<PointS, PointS> SphereFollowingRecursiveST::find_pair()
{
    int index = -1;
    float min_dist = std::numeric_limits<float>::max();
    for(int i = 0; i < _list.size(); i++)
    {
        TripletST trip = _list.at(i).value<TripletST>();
        if(trip.dist<min_dist)
        {
            min_dist = trip.dist;
            index =i;
        }
    }
    TripletST trip = _list.at(index).value<TripletST>();
    QPair<PointS,PointS> pair;
    pair.first = trip.end;
    pair.second = trip.attr;
    return pair;
}

void SphereFollowingRecursiveST::do_recursion()
{
    QTime time;
    time.restart();

    OptimizationSphereFollowingST optim (_cloud, _coeff, _subdivide_stem_and_branch_points, _is_multithreaded);
    optim.optimize();
    _coeff = OptimizationSphereFollowing::_coeff_end;



    QSharedPointer<OptimizationDownHillSimplexST> downhill
            (new OptimizationDownHillSimplexST(0.00000000001,_coeff,_cloud,_subdivide_stem_and_branch_points, _is_multithreaded));

//    QObject::connect(downhill.data(), SIGNAL (emit_counter(int)), this, SLOT (receive_counter(int)));
//    QObject::connect(downhill.data(), SIGNAL (emit_qstring(QString)), this, SLOT (receive_qstring(QString)));

    if(_coeff.optimze_stem)
    {
        downhill->set_ndim(6);
    } else
    {
        downhill->set_ndim(5);
    }

    downhill->minimize();
    _coeff = downhill->get_end_coeff();

    SphereFollowing2 spherefollowing(_coeff,_cloud, _subdivide_stem_and_branch_points);
    spherefollowing.sphere_following();
    _end_pts.reset(new PointCloudS);

    _attractors.reset(new PointCloudS);
    _cylinders = spherefollowing.get_cylinders();
    _end_pts = generate_end_point_cloud(_cylinders);

    PointCloudS::Ptr remaining_points (new PointCloudS);

    if(_coeff.optimze_stem)
    {

        remaining_points = spherefollowing.get_remaining_points();
    } else
    {
        ExtractFittedPoints extract (_cloud, spherefollowing.get_cylinders());
        remaining_points = extract.get_cloud_out();
    }





    VoxelGridFilter voxelgrid(remaining_points,0.1f);
    voxelgrid.compute();
    _attractors = voxelgrid.get_cloud_out();

    _octree.reset(new pcl::octree::OctreePointCloudSearch<PointS>(SimpleMath<float>::_OCTREE_RESOLUTION));
    _octree->setInputCloud (_end_pts);
    _octree->addPointsFromInputCloud ();
    initiate_list();

    while(!(_attractors->points.empty()))
    {
        QPair<PointS, PointS> pair = find_pair();

        PointS model = pair.first;
        PointS attractor = pair.second;

        remove_attractor(attractor);
        update_list(attractor);
        float x = attractor.x - model.x;
        float y = attractor.y - model.y;
        float z = attractor.z - model.z;

        PointS between;
        between.x = model.x + (x/2);
        between.y = model.y + (y/2);
        between.z = model.z + (z/2);
        //        _end_pts->points.push_back(between);
        _end_pts->points.push_back(attractor);
//        _octree.reset(new pcl::octree::OctreePointCloudSearch<PointS>(SimpleMath<float>::_OCTREE_RESOLUTION));
//        _octree->setInputCloud (_end_pts);
//        _octree->addPointsFromInputCloud ();
        pcl::ModelCoefficients coeff1;
        //coeff1.values.resize(7);
        coeff1.values.push_back(model.x);
        coeff1.values.push_back(model.y);
        coeff1.values.push_back(model.z);
        coeff1.values.push_back(x/2);
        coeff1.values.push_back(y/2);
        coeff1.values.push_back(z/2);
        //  coeff1.values.push_back(_radius);
        coeff1.values.push_back(0);

        pcl::ModelCoefficients coeff2;
        //coeff2.values.resize(7);
        coeff2.values.push_back(between.x);
        coeff2.values.push_back(between.y);
        coeff2.values.push_back(between.z);
        coeff2.values.push_back(x/2);
        coeff2.values.push_back(y/2);
        coeff2.values.push_back(z/2);
        coeff2.values.push_back(0);
        //coeff2.values.push_back(_radius);


        _cylinders.push_back(coeff1);
        _cylinders.push_back(coeff2);


    }




    qDebug() << "Optimization fit starting";
    OptimizationFitST fit2 (_cloud,_coeff,_cylinders, _is_multithreaded);
    fit2.optimize();
    _coeff = fit2._coeff_end;
    _tree = fit2._tree;
    qDebug() << "Optimization fit ended" << time.elapsed();



}

QPair<PointS, PointS> SphereFollowingRecursiveST::find_closest_pair(PointCloudS::Ptr cloud_model, PointCloudS::Ptr cloud_attractor)
{
    PointS model = cloud_model->points.at(0);
    PointS attractor = cloud_attractor->points.at(0);
    QPair<PointS, PointS> pair (model,attractor);
    float min_dist = SimpleMath<float>::get_distance(model,attractor);
    size_t index = 0;
    size_t index_model = 0;
    for(size_t i = 0; i < cloud_attractor->points.size(); i++)
    {
        attractor = cloud_attractor->points.at(i);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        _octree->nearestKSearch(attractor,1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        model = cloud_model->points.at(pointIdxRadiusSearch.at(0));
        float dist = std::sqrt(pointRadiusSquaredDistance.at(0));
        if(dist<min_dist)
        {
            min_dist = dist;
            index = i;
            index_model = pointIdxRadiusSearch.at(0);
            pair = QPair<PointS, PointS>(model,attractor);
        }
    }
    std::vector<PointS>::iterator it = cloud_attractor->points.begin() + index;
    cloud_attractor->points.erase(it);
    _radius = _cylinders.at(index_model).values.at(6);
    return pair;




}

PointCloudS::Ptr SphereFollowingRecursiveST::generate_end_point_cloud(QVector<pcl::ModelCoefficients> coeff)
{
    PointCloudS::Ptr end_pts (new PointCloudS);
    end_pts->points.resize(coeff.size());
    for(size_t i = 0; i< coeff.size(); i++)
    {
        pcl::ModelCoefficients cf = coeff.at(i);
        float x = cf.values[0] + cf.values[3];
        float y = cf.values[1] + cf.values[4];
        float z = cf.values[2] + cf.values[5];
        PointS p;
        p.x = x;
        p.y = y;
        p.z = z;
        end_pts->points[i] = p;
    }
    return end_pts;
}

PointCloudS::Ptr SphereFollowingRecursiveST::generate_start_point_cloud(QVector<pcl::ModelCoefficients> coeff)
{
    PointCloudS::Ptr start_pts (new PointCloudS);
    start_pts->points.resize(coeff.size());
    for(size_t i = 0; i< coeff.size(); i++)
    {
        pcl::ModelCoefficients cf = coeff.at(i);
        float x = cf.values[0];
        float y = cf.values[1];
        float z = cf.values[2];
        PointS p;
        p.x = x;
        p.y = y;
        p.z = z;
        start_pts->points[i] = p;
    }
    return start_pts;
}

void SphereFollowingRecursiveST::receive_qstring(QString qstr)
{
    emit emit_qstring_spherefollowingrecursive(qstr);
}

void SphereFollowingRecursiveST::receive_counter(int counter)
{
    emit emit_counter_spherefollowingrecursive(counter);
}

SphereFollowingRecursiveST::SphereFollowingRecursiveST(PointCloudS::Ptr cloud, MethodCoefficients coeff, bool subdivide_stem_and_branch_points, bool is_multithreaded)
{
    _cloud = cloud;
    _coeff = coeff;
    _subdivide_stem_and_branch_points = subdivide_stem_and_branch_points;
    _is_multithreaded = is_multithreaded;

}
