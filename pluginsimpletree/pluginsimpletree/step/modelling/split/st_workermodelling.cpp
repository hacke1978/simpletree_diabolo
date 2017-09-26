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
#include "st_workermodelling.h"

ST_Workemodelling::ST_Workemodelling(ST_StepParameter param, QSharedPointer<ST_ModellingThreadPool> mtp): _param(param), _mtp(mtp)
{

}

void ST_Workemodelling::run()
{
    CT_ResultGroup* resCpy_res = _param.resCpy_res;
    CT_StandardItemGroup* grpCpy_grp = _param.grpCpy_grp;
    CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in = _param.itemCpy_cloud_in;
    ST_Coefficients* coeff_in = _param.coeff_in;
    MethodCoefficients coeff = coeff_in->get_coeff();
    ConvertCTtoST ctst(itemCpy_cloud_in,_knn);
    ctst.convert();
    _cloud = ctst.get_cloud();
    split_cloud();
    SphereFollowing2 sphere (coeff,_cloud,true);
    sphere.sphere_following();
    QVector<pcl::ModelCoefficients> cylinder_coeff = sphere.get_cylinders();
    if(cylinder_coeff.size()>1)
    {
        ImproveByAttractor ia (_cloud,sphere.get_remaining_points(),coeff,cylinder_coeff);
        cylinder_coeff = ia.get_cylinders();
        ImproveByAttractor ia2 (_cloud_noise,_cloud_noise,coeff,cylinder_coeff);
        cylinder_coeff = ia2.get_cylinders();
        BuildTree builder(cylinder_coeff);
        QSharedPointer<Tree> tree (new Tree(builder.getRoot_segment(),coeff.id));
        RemoveFalseCylinders rfc(tree);
        ReorderTree rt(tree);
        ImproveByMedian ibm(tree);
        ImproveByMerge ibme(tree);
        ImproveByPipeModel ibpm (tree,false,0.01);
        ImproveFit imf(tree,_cloud,coeff);
        ImprovedByAdvancedMedian ibam(tree);
        ImproveBranchJunctions ibj(tree);
        ReorderTree rt2(tree);
        tree->setTreeID(coeff.id);
        add_cylinder_data(tree,resCpy_res,grpCpy_grp);


        QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
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
            }
        }


        QSharedPointer<Tree> tree_clone = tree->clone();

        ST_Tree * st_tree = new ST_Tree(_param._tree_out,resCpy_res,tree_clone);
        grpCpy_grp->addItemDrawable(st_tree);

        _mtp->sent_finished_tp();
        QString start;


        start = "Cloud ";
        start.append(coeff.id);
        start.append(" was modelled successfully.");
        _mtp->sent_qstring_tp(start);
    } else
    {
        QString start;
        start = "Cloud ";
        start.append(coeff.id);
        start.append(" could not be modelled. Please check manually");
        _mtp->sent_qstring_tp(start);
    }
    _cloud.reset(new PointCloudS);
    _cloud_noise.reset(new PointCloudS);
}


void ST_Workemodelling::split_cloud()
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

void ST_Workemodelling::sent_qstring_worker(QString str)
{
    _mtp->sent_qstring_tp(str);
}



void ST_Workemodelling::add_cylinder_data(QSharedPointer<Tree> tree, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp)
{
    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
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
        float growthVolume = tree->get_growth_volume(cylin);
        int detection = cylin->get_detection();
        int improvement = cylin->get_improvement();
        CT_StandardItemGroup* cylinderGroup = new CT_StandardItemGroup(_param._cylinder_grp, resCpy_res);
        grpCpy_grp->addGroup(cylinderGroup);
        QSharedPointer<PointS> center = cylin->get_center_ptr();
        QSharedPointer<PointS> start = cylin->get_start_ptr();
        QSharedPointer<PointS> stop = cylin->get_end_ptr();
        CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                    Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
                                                    cylin->get_radius(),
                                                    cylin->get_length());
        CT_Cylinder* cylinder = new CT_Cylinder(_param._cylinders, resCpy_res, data);
        cylinderGroup->addItemDrawable(cylinder);
        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_param._branchIDModelName, CT_AbstractCategory::DATA_ID, resCpy_res, branchID));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_param._branchOrderModelName, CT_AbstractCategory::DATA_NUMBER, resCpy_res, branchOrder));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_param._segmentIDModelName, CT_AbstractCategory::DATA_ID, resCpy_res, segmentID));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_param._parentSegmentIDModelName, CT_AbstractCategory::DATA_ID, resCpy_res, parentSegmentID));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(_param._growthVolumeModelName, CT_AbstractCategory::DATA_NUMBER, resCpy_res, growthVolume));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_param._detection_type, CT_AbstractCategory::DATA_NUMBER, resCpy_res, detection));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_param._improvement_type, CT_AbstractCategory::DATA_NUMBER, resCpy_res, improvement));

    }
}
