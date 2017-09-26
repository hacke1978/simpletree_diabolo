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
#include "st_stepmodelwithparam1.h"






ST_StepModelling1::ST_StepModelling1(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _knn = 16;
   // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

ST_StepModelling1::~ST_StepModelling1()
{
}


// Step description (tooltip of contextual menu)
QString ST_StepModelling1::getStepDescription() const
{
    return tr("QSM spherefollowing method - with precomputed parameters.");
}

// Step detailled description
QString ST_StepModelling1::getStepDetailledDescription() const
{
    return tr("This step calculates a model with pre found parameters without any post processing routines.");
}

// Step URL
QString ST_StepModelling1::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    //return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
ST_StepModelling1* ST_StepModelling1::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepModelling1(dataInit);
}





void ST_StepModelling1::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();

    configDialog->addTitle(tr("You will find help and description about the paramters in the following publication."));
    configDialog->addTitle(tr("This is also the correct citation you should give for scientific publications."));
    configDialog->addEmpty();
    configDialog->addTitle(tr("Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P."));
    configDialog->addTitle(tr("<em>SimpleTree —An Efficient Open Source Tool to Build Tree Models from TLS Clouds.</em>"));
    configDialog->addTitle(tr("Forests <b>2015</b>, 6, 4245-4294. "));
    configDialog->addEmpty();
    configDialog->addEmpty();
    configDialog->addText(tr("This step calculates a model with pre found parameters without any post processing routines."));

}


// Creation and affiliation of IN models
void ST_StepModelling1::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Isolated Tree cloud"));
    resIn_res->addItemModel(DEFin_grp, DEFin_coeff_in, CT_AbstractItemDrawableWithoutPointCloud::staticGetType(), tr ("parameter set"));
    resIn_res->addItemModel(DEFin_grp, DEFin_header, CT_FileHeader::staticGetType(), tr("File Header"));

}

// Creation and affiliation of OUT models
void ST_StepModelling1::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addItemModel(DEFin_grp, _tree_out, new ST_Tree(), tr("tree - modelled without allometry"));

        resCpy_res->addGroupModel(DEFin_grp, _cylinder_grp, new CT_StandardItemGroup(), tr("Cylinder group - without allometry"));
        resCpy_res->addItemModel(_cylinder_grp, _cylinders, new CT_Cylinder(), tr("cylinders - without allometry"));

        resCpy_res->addItemAttributeModel(_cylinders, _branchIDModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("branch_ID"));

        resCpy_res->addItemAttributeModel(_cylinders, _branchOrderModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("branch_order"));

        resCpy_res->addItemAttributeModel(_cylinders, _segmentIDModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("segment_ID"));

        resCpy_res->addItemAttributeModel(_cylinders, _parentSegmentIDModelName,
                                          new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_ID),
                                          tr("parent_segment_ID"));

        resCpy_res->addItemAttributeModel(_cylinders, _growthVolumeModelName,
                                          new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("growth_volume"));

        resCpy_res->addItemAttributeModel(_cylinders, _detection_type,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("detection_method"));

        resCpy_res->addItemAttributeModel(_cylinders, _improvement_type,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("improvement_method"));



    }
}

void ST_StepModelling1::split_cloud()
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

void ST_StepModelling1::sent_finished_step()
{
    _modelled_trees ++;
    int percentage = (((float)_modelled_trees)/((float)_total_size)*100.0f);
    setProgress(percentage);
}


void ST_StepModelling1::add_cylinder_data(QSharedPointer<Tree> tree, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp)
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



        CT_StandardItemGroup* cylinderGroup = new CT_StandardItemGroup(_cylinder_grp.completeName(), resCpy_res);
        grpCpy_grp->addGroup(cylinderGroup);


        QSharedPointer<PointS> center = cylin->get_center_ptr();
        QSharedPointer<PointS> start = cylin->get_start_ptr();
        QSharedPointer<PointS> stop = cylin->get_end_ptr();


        CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                    Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
                                                    cylin->get_radius(),
                                                    cylin->get_length());



        CT_Cylinder* cylinder = new CT_Cylinder(_cylinders.completeName(), resCpy_res, data);
        cylinderGroup->addItemDrawable(cylinder);

            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, branchID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchOrderModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, branchOrder));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_segmentIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, segmentID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_parentSegmentIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, parentSegmentID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(_growthVolumeModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, growthVolume));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_detection_type.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, detection));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_improvement_type.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, improvement));

    }
}


void ST_StepModelling1::compute()
{
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);

    CT_ResultGroupIterator itCpy_grp_temp(resCpy_res, this, DEFin_grp);
    float number_clouds = 0;
    while (itCpy_grp_temp.hasNext() && !isStopped())
    {
        itCpy_grp_temp.next();
        number_clouds++;
    }
    _total_size = number_clouds;

    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    QVector<ST_StepParameter> params;
    while (itCpy_grp.hasNext() && !isStopped())
    {
        ST_StepParameter param;
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in =
                (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        ST_Coefficients* coeff_in = (ST_Coefficients*) grpCpy_grp->firstItemByINModelName(this,DEFin_coeff_in );
        param.grpCpy_grp = grpCpy_grp;
        param.resCpy_res = resCpy_res;
        param.itemCpy_cloud_in = itemCpy_cloud_in;
        param.coeff_in = coeff_in;
        param._cylinders = _cylinders.completeName();
        param._cylinder_grp = _cylinder_grp.completeName();
        param._branchIDModelName = _branchIDModelName.completeName();
        param._branchOrderModelName = _branchOrderModelName.completeName();
        param._segmentIDModelName = _segmentIDModelName.completeName();
        param._parentSegmentIDModelName = _parentSegmentIDModelName.completeName();
        param._growthVolumeModelName = _growthVolumeModelName.completeName();
        param._detection_type = _detection_type.completeName();
        param._improvement_type = _improvement_type.completeName();
        param._tree_out = _tree_out.completeName();
        params.push_back(param);
    }
    QSharedPointer<ST_ModellingThreadPool> mtp (new ST_ModellingThreadPool(params));
    QObject::connect(mtp.data(), SIGNAL(emit_finished_tp()), this, SLOT(sent_finished_step()) );
    mtp->start_computation();
}
