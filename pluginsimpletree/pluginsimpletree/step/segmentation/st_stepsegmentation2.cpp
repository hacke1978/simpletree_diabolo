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
#include "st_stepsegmentation2.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_grp2 "grp2"
#define DEFin_cloud_in "cloud_in"
#define DEFin_tree_in "tree_in"
#define DEFin_source_in "source_in"
#define DEFin_target_in "target_in"


//#define DEF_resultIn_inputResult "inputResult"
//#define DEF_groupIn_inputScene "inputGroup"
//#define DEF_itemIn_scene "inputScene"


#define DEFin_normals_in "no_in"
#define DEFin_id_in "id_in"


//// Alias for indexing out models
#define DEF_resultOut_translated "extractedResult"
#define DEF_groupOut_pointCloud "extractedGroup"
#define DEF_itemOut_scene "extractedScene"



// Constructor : initialization of parameters
ST_StepSegmentation2::ST_StepSegmentation2(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{

}

ST_StepSegmentation2::~ST_StepSegmentation2()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepSegmentation2::getStepDescription() const
{
    return tr("Segmentation step 2 - transfers segmenting attribute from one cloud to another.");
}

// Step detailled description
QString ST_StepSegmentation2::getStepDetailledDescription() const
{
    return tr("Each point of the target cloud gets the selected attribute of the nearest point of the source cloud.");
}

// Step URL
QString ST_StepSegmentation2::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    // return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepSegmentation2::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepSegmentation2(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepSegmentation2::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));

//    resIn_res->addGroupModel("", DEFin_grp2, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));

    resIn_res->addItemModel(DEFin_grp, DEFin_source_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("partial Tree"));
    resIn_res->addItemModel(DEFin_grp, DEFin_id_in,  CT_PointsAttributesScalarTemplated<int>::staticGetType(),tr("partial detected tree ID"));
    resIn_res->addItemModel(DEFin_grp, DEFin_target_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("complete trees"));

}

// Creation and affiliation of OUT models
void ST_StepSegmentation2::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {

        res->addItemModel(DEFin_grp, _cloud_out_id, new CT_PointsAttributesScalarTemplated<int>(),tr("detected tree ID"));
    }

}

// Semi-automatic creation of step parameters DialogBox
void ST_StepSegmentation2::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addTitle(tr("You will find help and description about the paramters in the following publication."));
    configDialog->addTitle(tr("This is also the correct citation you should give for scientific publications."));
    configDialog->addEmpty();
    configDialog->addTitle(tr("Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P."));
    configDialog->addTitle(tr("<em>SimpleTree —An Efficient Open Source Tool to Build Tree Models from TLS Clouds.</em>"));
    configDialog->addTitle(tr("Forests <b>2015</b>, 6, 4245-4294. "));
    configDialog->addEmpty();
}

void ST_StepSegmentation2::compute()
{








    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);

    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    while (itCpy_grp.hasNext() && !isStopped())
    {
//            CT_ResultGroupIterator itCpy_grp2(resCpy_res, this, DEFin_grp2);
//                    CT_StandardItemGroup* grpCpy_grp2 = (CT_StandardItemGroup*) itCpy_grp2.next();


        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        CT_AbstractItemDrawableWithPointCloud* itemCpy_source_in
                = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_source_in);
        CT_AbstractItemDrawableWithPointCloud* itemCpy_target_in
                = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_target_in);
        CT_PointsAttributesScalarTemplated<int>* itemCpy_id_in
                = (CT_PointsAttributesScalarTemplated<int>*)grpCpy_grp->firstItemByINModelName(this, DEFin_id_in);
        ConvertCTtoST ctst(itemCpy_source_in,16);
        ctst.convert();
        _cloud_in_source = ctst.get_cloud();

        ConvertCTtoST ctst2(itemCpy_target_in,16);
        ctst2.convert();
        _cloud_in_target = ctst2.get_cloud();

        pcl::KdTreeFLANN<PointS>::Ptr kdtree(new pcl::KdTreeFLANN<PointS>);
        kdtree->setInputCloud (_cloud_in_source);

        size_t size = _cloud_in_target->points.size();
        CT_StandardCloudStdVectorT<int> *id_target = new CT_StandardCloudStdVectorT<int>(size);

        for(size_t i = 0; i < _cloud_in_target->points.size(); i++)
        {
            PointS target = _cloud_in_target->points.at(i);
            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);
            kdtree->nearestKSearch (target, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
            size_t index_source = pointIdxNKNSearch[0];
            id_target->tAt(i) = itemCpy_id_in->valueAt(index_source);
        }

        CT_PointsAttributesScalarTemplated<int> * id =
                new CT_PointsAttributesScalarTemplated<int>(_cloud_out_id.completeName(), resCpy_res,itemCpy_target_in->getPointCloudIndexRegistered(),id_target);
        grpCpy_grp->addItemDrawable(id);
    }
}

//void ST_StepSegmentation2::add_cluster(int size, const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp)
//{



//}


//void ST_StepSegmentation2::add_cylinder_data(Tree tree, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp, QString string)
//{
//    QVector<QSharedPointer<Cylinder> > cylinders = tree.get_all_cylinders();

//    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);



//    while(it.hasNext()) {
//        QSharedPointer<Cylinder> cylin = it.next();
//        int branchID = cylin->get_segment()->get_branch_id();
//        int branchOrder = cylin->get_segment()->get_branch_order();
//        int segmentID = cylin->get_segment()->get_id();
//        int parentSegmentID = -1;
//        if (!(cylin->get_segment()->is_root()))
//        {
//            parentSegmentID = cylin->get_segment()->get_parent_segment()->get_id();
//        }
//        float growthVolume = tree.get_growth_volume(cylin);

//        int detection = cylin->get_detection();

//        int improvement = cylin->get_improvement();


//        CT_StandardItemGroup* cylinderGroup = new CT_StandardItemGroup(_outCylinderGroupModelName.completeName(), resCpy_res);
//        grpCpy_grp->addGroup(cylinderGroup);


//        QSharedPointer<PointS> center = cylin->get_center_ptr();
//        QSharedPointer<PointS> start = cylin->get_start_ptr();
//        QSharedPointer<PointS> stop = cylin->get_end_ptr();


//        CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
//                                                    Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
//                                                    cylin->get_radius(),
//                                                    cylin->get_length());



//        CT_Cylinder* cylinder = new CT_Cylinder(string, resCpy_res, data);
//        cylinderGroup->addItemDrawable(cylinder);
//    }
//}



void ST_StepSegmentation2::enrich_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp)
{
    const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();

    size_t size = index->size();

    CT_StandardCloudStdVectorT<float> *distance_cloud = new CT_StandardCloudStdVectorT<float>(size);

    for(size_t i =0; i < size; i ++)
    {
        PointS p = _cloud_in_source->points[i];
        float dist = p.distance;

        distance_cloud->tAt(i) = dist;

    }

    CT_PointsAttributesScalarTemplated<float> * distance =
            new CT_PointsAttributesScalarTemplated<float>(_cloud_out_dist.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(),distance_cloud);

    grpCpy_grp->addItemDrawable(distance);
}











