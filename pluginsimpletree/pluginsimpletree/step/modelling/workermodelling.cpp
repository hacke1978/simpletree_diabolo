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
#include "workermodelling.h"

WorkerModelling::WorkerModelling(StepParameter param, QSharedPointer<ModellingThreadPool> mtp): _param(param), _mtp(mtp)
{

}

void WorkerModelling::run()
{
    CT_StandardItemGroup* itmgrp = _param.itmgrp;
    CT_ResultGroup* resCpy_res = _param.resCpy_res;
    CT_FileHeader* itemCpy_header =  _param.itemCpy_header ;
    QMap<QString, FileCoefficients> map = _param.map;
    MethodCoefficients coeff = _param.coeff;
    QString cloud_name = itemCpy_header->getFileName();

    FileCoefficients coeff_file = map.value(cloud_name);

    CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in = _param.itemCpy_cloud_in;
    QString test = "";
    if(coeff_file.file == test)
    {
        coeff_file.file = cloud_name;
    }

    QString start = "Starting computation for cloud ";
    start.append(coeff_file.file);
    _mtp->sent_qstring_tp(start);
    _knn =16;
    ConvertCTtoST ctst(itemCpy_cloud_in,_knn);
    ctst.convert();
    _cloud = ctst.get_cloud();
    enrich_cloud(itemCpy_cloud_in, resCpy_res,itmgrp);
    start = "Cloud ";
    start.append(coeff_file.file);
    start.append(" was enriched successfully and contains ");
    start.append(QString::number( _cloud->points.size()));
    start.append(" points.");
    _mtp->sent_qstring_tp(start);

    PredictStartParameters psp(_cloud, coeff_file);
    coeff = psp.get_coeff();
    coeff.use_dhs = _param.use_dhs;
    coeff.cut_height = coeff_file.cut_height;
    split_cloud();
    QSharedPointer<SphereFollowingRecursive> spherefollowing(new SphereFollowingRecursive (_cloud,_cloud_noise, coeff, true,false));
    qRegisterMetaType<QString>("QString");
    QObject::connect(spherefollowing.data(), SIGNAL(emit_qstring_spherefollowingrecursive(QString)), this, SLOT(sent_qstring_worker(QString)) ,Qt::DirectConnection);
    spherefollowing->do_recursion();
    coeff = spherefollowing->get_coeff();
    QVector<pcl::ModelCoefficients> cylinder_coeff = spherefollowing->get_cylinders();

    /**************************************************************/

    QString outputpath = coeff_file.output_path;
    _mtp->sent_qstring_tp("Computree data creation");


    SphereFollowing2 sphere (coeff,_cloud,true);
    sphere.sphere_following();
    cylinder_coeff = sphere.get_cylinders();
    if(cylinder_coeff.size()>1)
    {
        ImproveByAttractor ia (_cloud,sphere.get_remaining_points(),coeff,cylinder_coeff);
        cylinder_coeff = ia.get_cylinders();

        ImproveByAttractor ia2 (_cloud_noise,_cloud_noise,coeff,cylinder_coeff);
        cylinder_coeff = ia2.get_cylinders();
        BuildTree builder(cylinder_coeff);
        QSharedPointer<Tree> tree (new Tree(builder.getRoot_segment(),coeff.id));
        RemoveFalseCylinders remove(tree);
        ReorderTree reorder(tree);
        ImproveByMedian improve_by_median(tree);
        int i = 0;
        while(i < coeff.number_of_merges )
        {
                ImproveByMerge improve_merge(tree);
                i++;
        }
        ImproveByPipeModel pype(tree,false,0.1);
        ImproveFit fit(tree,_cloud,coeff);
        ImprovedByAdvancedMedian improve_by_median2(tree);
        ImproveBranchJunctions improve_junctions(tree);
        ReorderTree reorder4(tree);
        tree->setTreeID(cloud_name);

        if(coeff_file.use_allom=="FALSE"||(!coeff.use_allom))
        {
            ExportTree exporttree(tree,coeff,outputpath);
            add_cylinder_data(*tree, resCpy_res, itmgrp, _param._outCylinderModelName_improved_by_allometry);
            CT_TTreeGroup *ctTree2 = constructTopology(resCpy_res,tree, _param._topologyGroup);
            if(ctTree2 != NULL)
                itmgrp->addGroup(ctTree2);
            ST_Tree * st_tree = new ST_Tree(_param._model,resCpy_res,tree);
            itmgrp->addItemDrawable(st_tree);
            ST_Coefficients *  st_coefficients = new ST_Coefficients(_param._coeff, resCpy_res,coeff);
            itmgrp->addItemDrawable(st_coefficients);
        }


        if(coeff_file.use_allom=="TRUE"&&coeff.use_allom)
        {
            if(coeff.tree_height < 10)
            {
                ComputeAllometry ca2(tree);
                coeff.a = ca2.get_a();
                coeff.b = ca2.get_b();
                ImproveByAllometry(tree,coeff,coeff.a,coeff.b);
                ComputeAllometry ca3(tree,true); //
                coeff.a = ca3.get_a();
                coeff.b = ca3.get_b();
                ImproveByAllometry(tree,coeff,coeff.a,coeff.b);
                ComputeAllometry ca4(tree,true);//,true
                coeff.a = ca4.get_a();
                coeff.b = ca4.get_b();
                ImproveByAllometry(tree,coeff,coeff.a,coeff.b,1.3f);
            } else {
                ComputeAllometry ca2(tree,true);
                coeff.a = ca2.get_a();
                coeff.b = ca2.get_b();
                ImproveByAllometry(tree,coeff,coeff.a,coeff.b);
            }


            ExportTree exporttree(tree,coeff,outputpath);
            add_cylinder_data(*tree, resCpy_res, itmgrp, _param._outCylinderModelName_improved_by_allometry);
            CT_TTreeGroup *ctTree2 = constructTopology(resCpy_res,tree, _param._topologyGroup);
            if(ctTree2 != NULL)
                itmgrp->addGroup(ctTree2);
            ST_Tree * st_tree = new ST_Tree(_param._model,resCpy_res,tree);
            itmgrp->addItemDrawable(st_tree);
            ST_Coefficients *  st_coefficients = new ST_Coefficients(_param._coeff, resCpy_res,coeff);
            itmgrp->addItemDrawable(st_coefficients);





        }
        QString start;


        start = "Cloud ";
        start.append(coeff_file.file);
        start.append(" was modelled successfully.");
        _mtp->sent_qstring_tp(start);
    } else {

        QString start;


        start = "Cloud ";
        start.append(coeff_file.file);
        start.append(" could not be modelled. Please check manually");
        _mtp->sent_qstring_tp(start);

    }

    _mtp->sent_finished_tp();
    _cloud.reset(new PointCloudS);
    _cloud_noise.reset(new PointCloudS);
}

void WorkerModelling::enrich_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp)
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
        const CT_PointData &internalPoint =  it.next().currentConstInternalPoint();
        PointS query (internalPoint[0],internalPoint[1],internalPoint[2]);

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

void WorkerModelling::split_cloud()
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

void WorkerModelling::sent_qstring_worker(QString str)
{
    _mtp->sent_qstring_tp(str);
}



void WorkerModelling::add_cylinder_data(Tree tree, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp, QString string)
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

CT_TTreeGroup *WorkerModelling::constructTopology(const CT_AbstractResult *res_r, QSharedPointer<Tree> tree, QString string)
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

void WorkerModelling::constructTopologyRecursively(const CT_AbstractResult *res_r, CT_TNodeGroup *parent, QSharedPointer<Segment> segment, QSharedPointer<Tree> tree, QString string)
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

void WorkerModelling::setCylinders(const CT_AbstractResult *res_r, CT_TNodeGroup *root, QSharedPointer<Segment> segment, QSharedPointer<Tree> tree, QString string)
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
