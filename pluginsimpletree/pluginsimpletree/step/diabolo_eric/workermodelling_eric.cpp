/****************************************************************************

 Copyright (C) 2016-2017 INRA (Institut National de la Recherche Agronomique, France) and IGN (Institut National de l'information Géographique et forestière, France)
 All rights reserved.

 Contact : jan.hackenberg@posteo.de

 Developers : Jan Hackenberg

 This file is part of Simpletree plugin Version 4 for Computree.

 Simpletree plugin is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Simpletree plugin is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Simpletree plugin.  If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/

#include "workermodelling_eric.h"

WorkerModelling_Eric::WorkerModelling_Eric(StepParameter_Eric param, QSharedPointer<ModellingThreadPool_Eric> mtp): _param(param), _mtp(mtp)
{

}

void WorkerModelling_Eric::run()
{


    pcl::console::TicToc tt;
    tt.tic ();

    QString id = _param.id;
    QString id_string = id;
    QString cloud_name = id_string;
    QString file_name = cloud_name;
    MethodCoefficients coeff = _param.coeff;

    QString start = "Starting computation for cloud ";
    start.append(cloud_name);
    _mtp->sent_qstring_tp(start);

    _cloud = _param.pcl_cloud;

    start = "Cloud ";
    start.append(cloud_name);
    start.append(" was enriched successfully and contains ");
    start.append(QString::number( _cloud->points.size()));
    start.append(" points.");
    _mtp->sent_qstring_tp(start);

    EnrichCloud enrich3(_cloud, _knn, 0.03, false);

    StemPointDetection stempts3 (0,0.15,0.3,1,0,0.8,0.035,_cloud,1) ;
    stempts3.compute();

    float cut_height = 0.2;
    PredictStartParameters_Eric psp(_cloud, cut_height);
    coeff = psp.get_coeff();
    coeff.use_dhs = true;
    coeff.cut_height = cut_height;
    coeff.id = cloud_name;
//qDebug() << "a4";
        split_cloud();
//        qDebug() << "a5";
    QSharedPointer<SphereFollowingRecursive> spherefollowing(new SphereFollowingRecursive (_cloud,_cloud_noise, coeff, true,false));
    qRegisterMetaType<QString>("QString");
    QObject::connect(spherefollowing.data(), SIGNAL(emit_qstring_spherefollowingrecursive(QString)), this, SLOT(sent_qstring_worker(QString)) ,Qt::DirectConnection);
    spherefollowing->do_recursion();

    int seconds_parameter_find = tt.toc ()/1000;
    tt.tic();

    coeff = spherefollowing->get_coeff();
    QVector<pcl::ModelCoefficients> cylinder_coeff = spherefollowing->get_cylinders();
    /**************************************************************/
    QString path = "D:/DIABOLO/WP222/Simpletree/Beta33/results/";
    _mtp->sent_qstring_tp("Computree data creation");


    SphereFollowing2 sphere (coeff,_cloud,true);
    sphere.sphere_following();
    cylinder_coeff = sphere.get_cylinders();

    int seconds_covert = 0;
    int seconds_single_model = tt.toc ()/1000;
    tt.tic();
    if(cylinder_coeff.size()>3)
    {
        PointCloudS::Ptr remaining_cloud = sphere.get_remaining_points();
        cylinder_coeff = iterative_run(remaining_cloud,cylinder_coeff,coeff);
        int seconds_iterative_model = tt.toc ()*1000;
        tt.tic();
        ImproveByAttractor ia2 (_cloud_noise,_cloud_noise,coeff,cylinder_coeff, 9);
        cylinder_coeff = ia2.get_cylinders();
        int seconds_attractor = tt.toc ()*1000;
        tt.tic();
        BuildTree builder(cylinder_coeff);
        QSharedPointer<Tree> tree (new Tree(builder.getRoot_segment(),coeff.id));


        if(coeff.use_remove_false_cylinders)
        {
            RemoveFalseCylinders remove(tree);
        }


        if(coeff.use_improve_by_median)
        {
            ImproveByMedian improve_by_median(tree);
        }

                ImproveByPipeModel pype(tree, true, 0.7);

        ReorderTree reorder(tree);



        int seconds_build_tree = tt.toc ()*1000;
        tt.tic();
        ImproveByPipeModel pype2(tree, false, 0.5);

        if(coeff.use_stem_taper)
        {
            Stem_Taper taper(tree,coeff);
            coeff = taper.coeff();
        }
        tt.tic();
        OptimizationFit fit2 (_cloud,coeff, tree->get_pcl_coefficients(), true);
        fit2.optimize();
        coeff = fit2._coeff_end;
        int i = 0;
        while(i < coeff.number_of_merges )
        {
            ImproveByMerge improve_merge(tree);
            i++;
        }

        ImproveFit fit(tree,_cloud,coeff);
        int seconds_improve = tt.toc ()*1000;

        int seconds_pype = tt.toc ()*1000;

        tt.tic();

        if(coeff.use_median_filter_later)
        {
            ImprovedByAdvancedMedian improve_by_median2(tree);
        }


        QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
        if(coeff.use_median_filter_later)
        {
           // qDebug() << "median filter later";
            QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
            while(it.hasNext())
            {
                QSharedPointer<Cylinder> cyl = it.next();
                QVector<QSharedPointer<Cylinder> > neighbors = tree->get_neighbor_cylinders(cyl);
                QVector<float> radii;
                QVectorIterator<QSharedPointer<Cylinder> > git (neighbors);
                while(git.hasNext())
                {
                    QSharedPointer<Cylinder> n_cyl = git.next();
                    radii.push_back(n_cyl->get_radius());
                }
                float average = SimpleMath<float>::get_median(radii);
                float radius = cyl->get_radius();
                if(radius > 1.1f*average || radius < 0.9*average)
                {
                    cyl->set_radius(average);
                    cyl->set_allometry_improvement(AllometryImproved::ALLOM_MEDIAN);
                }
                if(radius < 0.0045)
                {
                    radius = 0.0045;
                }
            }
        } else {
          //  qDebug() << "not median filter later";
        }

        if(coeff.use_improve_branch_junctions)
        {
            ImproveBranchJunctions improve_junctions(tree);
        }
        ReorderTree reorder4(tree);
        tree->setTreeID(cloud_name);
        _mtp->count_cylinders(cylinders.size());






        QString start;


        start = "Cloud ";
        start.append(cloud_name);
        start.append(" was modelled successfully.");
        _mtp->sent_qstring_tp(start);


        QString timings;
        timings.append(QString::number(seconds_covert)).append(";");
        timings.append(QString::number(seconds_parameter_find)).append(";");
        timings.append(QString::number(seconds_single_model)).append(";");
        timings.append(QString::number(seconds_iterative_model)).append(";");
        timings.append(QString::number(seconds_attractor)).append(";");
        timings.append(QString::number(seconds_build_tree)).append(";");
        timings.append(QString::number(seconds_pype)).append(";");
        timings.append(QString::number(seconds_improve)).append(";");
        timings.append(QString::number(_cloud->points.size())).append(";");
        timings.append(coeff.id).append(";");
        _mtp->sent_finished_tp();
        _mtp->sent_timings(timings);


        {


            QSharedPointer<Tree> tree_old = tree;
            if(tree_old!=0)
            {



//                QSharedPointer<Tree> tree = tree_old->clone();

//                {

//                                QString path_2 = path;
//                                path_2.append(_param._cloud_name_ori);
//                                path_2.append("/");
//                                QString name = file_name;
//                                name.append("_no_allom");
//                                Export::export_ply_color ( path_2, name,tree,8);
//                                Export::export_tree_detail(tree,path_2, name,coeff);
//                                }
                ImproveByPipeModel pype(tree, true, 0.2);

                ComputeAllometrySave ca2( tree,true);
                coeff.a = ca2.get_a();
                coeff.b = ca2.get_b();
                coeff.length_a = ca2.get_a();
                coeff.length_b = ca2.get_b();



{

                QString path_2 = path;
                path_2.append(_param._cloud_name_ori);
                path_2.append("/");
                QString name = file_name;
                name.append("_vol_allom");
                Export::export_ply_color ( path_2, name,tree,8);
                Export::export_tree_detail(tree,path_2, name,coeff);
                }
            }

        }





    } else {

        QString start;


        start = "Cloud ";
        start.append(cloud_name);
        start.append(" could not be modelled. Please check manually");
        _mtp->sent_qstring_tp(start);

    }


    _cloud.reset(new PointCloudS);
    _cloud_noise.reset(new PointCloudS);
}

Eigen::Vector3f WorkerModelling_Eric::get_vec(PointCloudS::Ptr cloud_in, QVector<pcl::ModelCoefficients> tree_coeff)
{
    PointS p1;
    PointS p2;
    PointCloudS::Ptr tree_cloud (new PointCloudS);
    for(size_t i = 0; i < tree_coeff.size(); i++)
    {
        PointS p (tree_coeff.at(i).values[0]+tree_coeff.at(i).values[3],
                tree_coeff.at(i).values[1]+tree_coeff.at(i).values[4],
                tree_coeff.at(i).values[2]+tree_coeff.at(i).values[5]);
        tree_cloud->push_back(p);
    }

    pcl::search::KdTree<PointS>::Ptr kd_tree (new pcl::search::KdTree<PointS>);
    kd_tree->setInputCloud (tree_cloud);
    float min_dist = std::numeric_limits<float>::max();
    float max_dist = std::numeric_limits<float>::min();
    for(size_t i = 0; i < cloud_in->points.size(); i++)
    {
        PointS query = cloud_in->points.at(i);
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        kd_tree->nearestKSearch (query, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        if(pointNKNSquaredDistance[0]<min_dist)
        {
            min_dist = pointNKNSquaredDistance[0];
            p1 = query;
        }
    }


    for(size_t i = 0; i < cloud_in->points.size(); i++)
    {
        PointS query = cloud_in->points.at(i);
        float distance = SimpleMath<float>::get_distance(p1,query);
        if(distance>max_dist)
        {
            max_dist = distance;
            p2 = query;
        }
    }


    Eigen::Vector3f vec;
    vec[0] = p2.x - p1.x;
    vec[1] = p2.y - p1.y;
    vec[2] = p2.z - p1.z;

    return vec;

}

void WorkerModelling_Eric::enrich_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp)
{

    const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();
    size_t size = index->size();
    CT_NormalCloudStdVector *normalCloud = new CT_NormalCloudStdVector( size );
    CT_StandardCloudStdVectorT<float> *stemCloud = new CT_StandardCloudStdVectorT<float>(size);
    pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS>);
    tree->setInputCloud (_cloud);
    CT_PointIterator it (index);
    for(size_t i =0; i < size; i ++)
    {
        const CT_Point &internalPoint =  it.next().currentPoint();
        PointS query (internalPoint(0),internalPoint(1),internalPoint(2));

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        tree->nearestKSearch(query,1,pointIdxRadiusSearch,pointRadiusSquaredDistance);
        PointS p = _cloud->points[pointIdxRadiusSearch[0]];
        float n1 = p.normal_x;
        float n2 = p.normal_y;
        float n3 = p.normal_z;
        float stem = p.is_stem;
        CT_Normal &ctNormal = normalCloud->normalAt(i);
        ctNormal.x() = n1;
        ctNormal.y() = n2;
        ctNormal.z() = n3;
        ctNormal.w() = p.curvature;
        stemCloud->tAt(i) = stem;
    }
    CT_PointsAttributesNormal * normals = new CT_PointsAttributesNormal
            (_param._cloud_out_normals, resCpy_res, itemCpy_cloud_in->getPointCloudIndexRegistered(),normalCloud);
    CT_PointsAttributesScalarTemplated<float> * stem =  new CT_PointsAttributesScalarTemplated<float>
            (_param._cloud_out_stem, resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(),stemCloud);
    grpCpy_grp->addItemDrawable(normals);
    grpCpy_grp->addItemDrawable(stem);
}

void WorkerModelling_Eric::split_cloud()
{
    MethodCoefficients cf;
    PointCloudS::Ptr temp( new PointCloudS);

    _cloud_noise.reset(new PointCloudS);


    pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS>);
    tree->setInputCloud (_cloud);
    std::vector<pcl::PointIndices> _cluster_indices;

    pcl::EuclideanClusterExtraction<PointS> ec;
    ec.setClusterTolerance (cf.clustering_distance); // 2cm
    ec.setSearchMethod (tree);
    ec.setInputCloud (_cloud);
    ec.extract (_cluster_indices);
    std::vector<bool> is_cluster;
    for(size_t i = 0; i < _cloud->points.size(); i++)
    {
        is_cluster.push_back(false);
    }

    int max_cluster = 1;
    max_cluster = std::max(cf.number_clusters_for_spherefollowing,max_cluster);
    max_cluster = std::min((int)_cluster_indices.size(),max_cluster);
    for(int i = 0; i <max_cluster; i++)
    {
        pcl::PointIndices c_ind = _cluster_indices.at(i);
        for(size_t j = 0; j < c_ind.indices.size(); j++)
        {
            int index = c_ind.indices.at(j);
            is_cluster[index] = true;
        }
    }

    for(size_t i = 0; i < is_cluster.size(); i++)
    {
        bool is_clstr = is_cluster.at(i);
        if(is_clstr)
        {
            temp->points.push_back(_cloud->points.at(i));
        }
        else
        {
            _cloud_noise->points.push_back(_cloud->points.at(i));
        }
    }
    _cloud = temp;
}

void WorkerModelling_Eric::sent_qstring_worker(QString str)
{
    _mtp->sent_qstring_tp(str);
}



void WorkerModelling_Eric::add_cylinder_data(Tree tree, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp, QString string)
{
    QVector<QSharedPointer<Cylinder> > cylinders = tree.get_all_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext()) {
        QSharedPointer<Cylinder> cylin = it.next();
        int branchID = cylin->get_segment()->get_branch_id();
        int branchOrder = cylin->get_segment()->get_branch_order();
        int segmentID = cylin->get_segment()->get_id();
        int parentSegmentID = -1;
        if (!(cylin->get_segment()->is_root()))
        {
            parentSegmentID = cylin->get_segment()->get_parent_segment()->get_id();
        }
        float growthVolume = tree.get_growth_volume(cylin);

        int detection = cylin->get_detection();

        int improvement = cylin->get_improvement();


        CT_StandardItemGroup* cylinderGroup = new CT_StandardItemGroup(_param._outCylinderGroupModelName, resCpy_res);
        grpCpy_grp->addGroup(cylinderGroup);


        QSharedPointer<PointS> center = cylin->get_center_ptr();
        QSharedPointer<PointS> start = cylin->get_start_ptr();
        QSharedPointer<PointS> stop = cylin->get_end_ptr();


        CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                    Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
                                                    cylin->get_radius(),
                                                    cylin->get_length());



        CT_Cylinder* cylinder = new CT_Cylinder(string, resCpy_res, data);
        cylinderGroup->addItemDrawable(cylinder);
        if (string == _param._outCylinderModelName_improved_by_allometry)
        {
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_param._branchIDModelName, CT_AbstractCategory::DATA_ID, resCpy_res, branchID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_param._branchOrderModelName, CT_AbstractCategory::DATA_NUMBER, resCpy_res, branchOrder));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_param._segmentIDModelName, CT_AbstractCategory::DATA_ID, resCpy_res, segmentID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_param._parentSegmentIDModelName, CT_AbstractCategory::DATA_ID, resCpy_res, parentSegmentID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(_param._growthVolumeModelName, CT_AbstractCategory::DATA_NUMBER, resCpy_res, growthVolume));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_param._detection_type, CT_AbstractCategory::DATA_NUMBER, resCpy_res, detection));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_param._improvement_type, CT_AbstractCategory::DATA_NUMBER, resCpy_res, improvement));
        }
    }
}

CT_TTreeGroup *WorkerModelling_Eric::constructTopology(const CT_AbstractResult *res_r, QSharedPointer<Tree> tree, QString string)
{
    QSharedPointer<Segment> segment = tree->get_root_segment();
    CT_TTreeGroup *topology = new CT_TTreeGroup(string, res_r);
    if(string == _param._topologyGroup)
    {
        CT_TNodeGroup *root = new CT_TNodeGroup(_param._stemGroup, res_r);
        topology->setRootNode(root);
        setCylinders(res_r, root, segment,tree, string);
        constructTopologyRecursively(res_r,root, segment, tree,string);
    }
    return topology;
}

void WorkerModelling_Eric::constructTopologyRecursively(const CT_AbstractResult *res_r, CT_TNodeGroup *parent, QSharedPointer<Segment> segment, QSharedPointer<Tree> tree, QString string)
{
    QVector<QSharedPointer<Segment> > children = segment->get_child_segments();
    QVectorIterator<QSharedPointer<Segment> > it (children);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment_child = it.next();
        if(string == _param._topologyGroup)
        {
            CT_TNodeGroup* branchGroup = new CT_TNodeGroup(_param._stemGroup, res_r);
            parent->addBranch(branchGroup);
            setCylinders(res_r, branchGroup, segment_child, tree, string);
            constructTopologyRecursively(res_r,branchGroup, segment_child,tree,string);
        }
    }
}

void WorkerModelling_Eric::setCylinders(const CT_AbstractResult *res_r, CT_TNodeGroup *root, QSharedPointer<Segment> segment, QSharedPointer<Tree> tree, QString string)
{

    QVector<QSharedPointer<Cylinder> >cylinders = segment->get_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylin = it.next();

        if(string == _param._topologyGroup)
        {
            CT_TNodeGroup* cylinderGroup = new CT_TNodeGroup(_param._stemGroup, res_r);
            root->addComponent(cylinderGroup);
            QSharedPointer<PointS> center = cylin->get_center_ptr();
            QSharedPointer<PointS> start = cylin->get_start_ptr();
            QSharedPointer<PointS> stop = cylin->get_end_ptr();
            CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                        Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
                                                        cylin->get_radius(),
                                                        cylin->get_length());
            CT_Cylinder* cylinder = new CT_Cylinder(_param._stemCylinders, res_r, data);
            cylinderGroup->addItemDrawable(cylinder);
        }
    }
}

QVector<pcl::ModelCoefficients> WorkerModelling_Eric::iterative_run(PointCloudS::Ptr remaining_cloud, QVector<pcl::ModelCoefficients> cylinder_coeff, MethodCoefficients coeff)
{
    //    qDebug() << "WorkerModelling2 01";
    pcl::search::KdTree<PointS>::Ptr kd_tree (new pcl::search::KdTree<PointS>);
    kd_tree->setInputCloud (remaining_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointS> ec;
    ec.setClusterTolerance (0.05); // 2cm
    ec.setMinClusterSize (1);
    ec.setMaxClusterSize (99000000);
    ec.setSearchMethod (kd_tree);
    ec.setInputCloud (remaining_cloud);
    ec.extract (cluster_indices);

    QVector<PointCloudS::Ptr> large_clusters;
    PointCloudS::Ptr small_clusters (new PointCloudS);
    int min_cluster_size = _cloud->points.size()*_percent;
    min_cluster_size = std::max(min_cluster_size, 200);
    for(size_t k = 0; k < cluster_indices.size(); k++)
    {
        pcl::PointIndices index_cluster = cluster_indices.at(k);
        if(index_cluster.indices.size()>= min_cluster_size)
        {
            PointCloudS::Ptr cloud_cluster (new PointCloudS);
            for (std::vector<int>::const_iterator pit = index_cluster.indices.begin (); pit != index_cluster.indices.end (); ++pit)
                cloud_cluster->points.push_back (remaining_cloud->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            large_clusters.push_back(cloud_cluster);
        } else {
            for (std::vector<int>::const_iterator pit = index_cluster.indices.begin (); pit != index_cluster.indices.end (); ++pit)
                small_clusters->points.push_back (remaining_cloud->points[*pit]); //*
        }
    }
    PointCloudS::Ptr end_pt_cloud (new PointCloudS);
    for(int i = 0; i < cylinder_coeff.size(); i++)
    {
        pcl::ModelCoefficients cyl_cf = cylinder_coeff.at(i);
        PointS end;
        end.x = cyl_cf.values[0] + cyl_cf.values[3];
        end.y = cyl_cf.values[1] + cyl_cf.values[4];
        end.z = cyl_cf.values[2] + cyl_cf.values[5];
        end_pt_cloud->points.push_back(end);
    }
    pcl::search::KdTree<PointS>::Ptr kd_tree2 (new pcl::search::KdTree<PointS>);
    kd_tree2->setInputCloud (end_pt_cloud);
    QVector<QPair<PointCloudS::Ptr,float> >pairs;
    QVectorIterator<PointCloudS::Ptr> pit(large_clusters);
    while(pit.hasNext())
    {
        PointCloudS::Ptr cloud = pit.next();
        float  min_dist = std::numeric_limits<float>::max();
        for(size_t i = 0; i < cloud->points.size(); i++)
        {
            PointS p = cloud->points.at(i);

            std::vector<int> pointIdxNKNSearch;
            std::vector<float> pointNKNSquaredDistance;
            if(kd_tree2->nearestKSearch(p,1, pointIdxNKNSearch, pointNKNSquaredDistance))
            {
                if(pointNKNSquaredDistance[0]<min_dist)
                    min_dist = pointNKNSquaredDistance[0];
            }
        }
        QPair<PointCloudS::Ptr,float> pair (cloud,min_dist);
        pairs.push_back(pair);
    }
    std::sort(pairs.begin(), pairs.end(), comparefunc);

    QVector<PointCloudS::Ptr> temp;
    QVectorIterator<QPair<PointCloudS::Ptr,float> > git(pairs);
    while(git.hasNext())
    {
        QPair<PointCloudS::Ptr,float> pair = git.next();
        temp.push_back(pair.first);
    }

    large_clusters = temp;

    small_clusters->width = small_clusters->points.size();
    small_clusters->height = 1;
    small_clusters->is_dense = true;
    QVectorIterator<PointCloudS::Ptr> it(large_clusters);
    while(it.hasNext())
    {
        PointCloudS::Ptr cluster = it.next();
        SphereFollowing2 sphere (coeff,cluster , true);
        sphere.sphere_following();
        QVector<pcl::ModelCoefficients> cluster_coeff = sphere.get_cylinders();
        if(cluster_coeff.size()>0)
        {
            cylinder_coeff = connect_iterative_cylinders(cylinder_coeff,cluster_coeff);
            *small_clusters += *sphere.get_remaining_points();
        }
        else {
            *small_clusters  += *cluster;
        }

    }
    ImproveByAttractor ia (_cloud,small_clusters,coeff,cylinder_coeff,9);
    cylinder_coeff = ia.get_cylinders();
    return cylinder_coeff;

}

QVector<pcl::ModelCoefficients> WorkerModelling_Eric::connect_iterative_cylinders(QVector<pcl::ModelCoefficients> cylinder_coeff_large, QVector<pcl::ModelCoefficients> cylinder_coeff_iterative)
{
    QVector<pcl::ModelCoefficients> result;
    result += cylinder_coeff_large;
    PointCloudS::Ptr end_pt_cloud (new PointCloudS);
    for(int i = 0; i < cylinder_coeff_large.size(); i++)
    {
        pcl::ModelCoefficients cyl_cf = cylinder_coeff_large.at(i);
        PointS end;
        end.x = cyl_cf.values[0] + cyl_cf.values[3];
        end.y = cyl_cf.values[1] + cyl_cf.values[4];
        end.z = cyl_cf.values[2] + cyl_cf.values[5];
        end_pt_cloud->points.push_back(end);
    }

    pcl::search::KdTree<PointS>::Ptr kd_tree (new pcl::search::KdTree<PointS>);
    kd_tree->setInputCloud (end_pt_cloud);

    PointS start;
    start.x =cylinder_coeff_iterative.at(0).values[0];
    start.y =cylinder_coeff_iterative.at(0).values[1];
    start.z =cylinder_coeff_iterative.at(0).values[2];

    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    if(kd_tree->nearestKSearch(start,1, pointIdxNKNSearch, pointNKNSquaredDistance))
    {
        PointS end = end_pt_cloud->points.at(pointIdxNKNSearch[0]);
        pcl::ModelCoefficients connection_coeff;
        connection_coeff.values.push_back(end.x);
        connection_coeff.values.push_back(end.y);
        connection_coeff.values.push_back(end.z);
        connection_coeff.values.push_back(start.x-end.x);
        connection_coeff.values.push_back(start.y-end.y);
        connection_coeff.values.push_back(start.z-end.z);
        connection_coeff.values.push_back(cylinder_coeff_iterative.at(0).values[6]);
        result.push_back(connection_coeff);
        result += cylinder_coeff_iterative;
    } else {
        qDebug() << "kd_tree error in WorkerModelling2::connect_iterative_cylinders";
    }
    return result;

}
