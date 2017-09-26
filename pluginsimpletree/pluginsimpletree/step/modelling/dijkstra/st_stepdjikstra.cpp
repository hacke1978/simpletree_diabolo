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
#include "st_stepdjikstra.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"

//#define DEF_resultIn_inputResult "inputResult"
//#define DEF_groupIn_inputScene "inputGroup"
//#define DEF_itemIn_scene "inputScene"


#define DEFin_normals_in "no_in"
#define DEFin_stem_in "stem_in"


//// Alias for indexing out models
#define DEF_resultOut_translated "extractedResult"
#define DEF_groupOut_pointCloud "extractedGroup"
#define DEF_itemOut_scene "extractedScene"



// Constructor : initialization of parameters
ST_StepDijkstra::ST_StepDijkstra(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{

}

ST_StepDijkstra::~ST_StepDijkstra()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepDijkstra::getStepDescription() const
{
    return tr("QSM method 2 .");
}

// Step detailled description
QString ST_StepDijkstra::getStepDetailledDescription() const
{
    return tr("QSM method 2 ." );
}

// Step URL
QString ST_StepDijkstra::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    // return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepDijkstra::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepDijkstra(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepDijkstra::createInResultModelListProtected()
{
      CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));

    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Isolated Tree"));

}

// Creation and affiliation of OUT models
void ST_StepDijkstra::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {
        res->addItemModel(DEFin_grp, _cloud_out_dist, new CT_PointsAttributesScalarTemplated<float>(),tr("distance"));
        res->addGroupModel(DEFin_grp, _outCylinderGroupModelName, new CT_StandardItemGroup(), tr("Cylinder group"));
        res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_improved_by_allometry, new CT_Cylinder(), tr("Cylinder_by_allometry"));



        res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _branchIDModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("branch_ID"));


        res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _branchOrderModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("branch_order"));


        res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _segmentIDModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("segment_ID"));


        res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _parentSegmentIDModelName,
                                          new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_ID),
                                          tr("parent_segment_ID"));

        res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _growthVolumeModelName,
                                          new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("growth_volume"));

        res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _tree_species,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_VALUE),
                                          tr("tree_species"));
        res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _tree_id,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_VALUE),
                                          tr("tree_id"));

        res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _detection_type,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("detection_method"));
        res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _improvement_type,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("improvement_method"));
    }

}

// Semi-automatic creation of step parameters DialogBox
void ST_StepDijkstra::createPostConfigurationDialog()
{
//    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
//    configDialog->addDouble( tr("clustering distance"), "m",  0.01, 0.2,2,  _distance, 1, "the clustering distance.");
}

void ST_StepDijkstra::compute()
{
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);

    // IN results browsing


    // COPIED results browsing
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    while (itCpy_grp.hasNext() && !isStopped())
    {

        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in
                = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);

        ConvertCTtoST ctst(itemCpy_cloud_in,16);
        ctst.convert();
        _cloud_in = ctst.get_cloud();
warning_geo_referenced(_cloud_in);
        DijkstraCoefficients coeff;

        ExtractLowestClusters elc(_cloud_in, 0.05f,0.05f);

        coeff.bin_width = 0.7f;
        coeff.cluster_for_bin = 0.6f;

        Dijkstra d(_cloud_in,elc.get_clusters(), coeff);

        enrich_cloud(itemCpy_cloud_in,resCpy_res,grpCpy_grp);

        GenerateSkeletonCloud gsc(_cloud_in,coeff);

        PointCloudS::Ptr skeleton_pts = gsc.get_skeleton();

        BuildTopology bt (skeleton_pts,coeff);

        QVector<pcl::ModelCoefficients> cyl_coeff = bt.get_skeleton_coeff();

        BuildTree builder(cyl_coeff);

        QSharedPointer<Tree> tree (new Tree(builder.getRoot_segment(), "tree"));

        add_cylinder_data(*tree, resCpy_res, grpCpy_grp, _outCylinderModelName_improved_by_allometry.completeName());




    }
}

void ST_StepDijkstra::add_cylinder_data(Tree tree, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp, QString string)
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


        CT_StandardItemGroup* cylinderGroup = new CT_StandardItemGroup(_outCylinderGroupModelName.completeName(), resCpy_res);
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
        if (string == _outCylinderModelName_improved_by_allometry.completeName())
        {
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, branchID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchOrderModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, branchOrder));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_segmentIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, segmentID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_parentSegmentIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, parentSegmentID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(_growthVolumeModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, growthVolume));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_detection_type.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, detection));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_improvement_type.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, improvement));
        }

    }

    _cloud_in.reset(new PointCloudS);

}



void ST_StepDijkstra::enrich_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp)
{
    const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();

    size_t size = index->size();

    CT_StandardCloudStdVectorT<float> *distance_cloud = new CT_StandardCloudStdVectorT<float>(size);

    for(size_t i =0; i < size; i ++)
    {
        PointS p = _cloud_in->points[i];
        float dist = p.distance;

        distance_cloud->tAt(i) = dist;

    }

    CT_PointsAttributesScalarTemplated<float> * distance =
            new CT_PointsAttributesScalarTemplated<float>(_cloud_out_dist.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(),distance_cloud);

    grpCpy_grp->addItemDrawable(distance);

}











