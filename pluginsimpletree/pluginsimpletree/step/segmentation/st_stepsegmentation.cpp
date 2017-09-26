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
#include "st_stepsegmentation.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_grp_out "grp_out"
#define DEFin_cloud_out "cloud_out"
#define DEFin_cloud_in "cloud_in"
#define DEFin_grp_in "grp_cluster_in"
#define DEFin_cluster_in "cluster_in"
#define DEFin_tree_in "tree_in"

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
ST_StepSegmentation::ST_StepSegmentation(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{

}

ST_StepSegmentation::~ST_StepSegmentation()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepSegmentation::getStepDescription() const
{
    return tr("Segmentation step 1 - Segments by seeds");
}

// Step detailled description
QString ST_StepSegmentation::getStepDetailledDescription() const
{
    return tr("Segments the cloud. The cloud should be downscaled to improve runtime performance. The ground has to be removed before. The step needs segmentation seeds aka detected tree roots."
              " You can produce those with euclidean clustering.");
}

// Step URL
QString ST_StepSegmentation::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    // return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepSegmentation::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepSegmentation(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepSegmentation::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));

    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Plot without ground"));
        resIn_res->addGroupModel(DEFin_grp, DEFin_grp_in, CT_AbstractItemGroup::staticGetType(), tr("cluster_in"));

    resIn_res->addItemModel(DEFin_grp_in, DEFin_cluster_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Clusters"));



}

// Creation and affiliation of OUT models
void ST_StepSegmentation::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {

        res->addItemModel(DEFin_grp, _cloud_out_dist, new CT_PointsAttributesScalarTemplated<float>(),tr("partial detected distance"));
        res->addItemModel(DEFin_grp, _cloud_out_id, new CT_PointsAttributesScalarTemplated<int>(),tr("partial detected tree ID"));



        //        res->addGroupModel(DEFin_grp, _grp_out, new CT_StandardItemGroup(), tr("Group for trees"));
        //        res->addItemModel(_grp_out, _tree, new CT_Scene(), tr("Partial detected Tree cloud"));


    }

}

// Semi-automatic creation of step parameters DialogBox
void ST_StepSegmentation::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addTitle(tr("You will find help and description about the paramters in the following publication."));
    configDialog->addTitle(tr("This is also the correct citation you should give for scientific publications."));
    configDialog->addEmpty();
    configDialog->addTitle(tr("Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P."));
    configDialog->addTitle(tr("<em>SimpleTree —An Efficient Open Source Tool to Build Tree Models from TLS Clouds.</em>"));
    configDialog->addTitle(tr("Forests <b>2015</b>, 6, 4245-4294. "));
    configDialog->addEmpty();
    configDialog->addText(tr("Should not be run on full defragmented cloud. Subset largest cluster of euclidean clustering (0.05,200,1) should be used."));

    //    configDialog->addDouble( tr("clustering distance"), "m",  0.01, 0.2,2,  _distance, 1, "the clustering distance.");
}

void ST_StepSegmentation::compute()
{
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);

    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    while (itCpy_grp.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in
                = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        ConvertCTtoST ctst(itemCpy_cloud_in,16);
        ctst.convert();
        _cloud_in = ctst.get_cloud();
        QVector<PointCloudS::Ptr> clstrs;
        CT_GroupIterator itGroupToCount(grpCpy_grp, this, DEFin_grp_in);
        while (itGroupToCount.hasNext() && (!isStopped()))
        {
                    CT_StandardItemGroup* grpCpy_grp2 = (CT_StandardItemGroup*) itGroupToCount.next();
                    CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in2
                            = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp2->firstItemByINModelName(this, DEFin_cluster_in);
                    ConvertCTtoST ctst(itemCpy_cloud_in2,13);
                    ctst.convert();
                    clstrs.push_back(ctst.get_cloud());
        }
        DijkstraCoefficients coeff;
//        coeff.search_range = 0.1;
//        coeff.punishment_power = 1;
        Dijkstra d(_cloud_in,clstrs, coeff);
        enrich_cloud(itemCpy_cloud_in,resCpy_res,grpCpy_grp);

    }
}

void ST_StepSegmentation::add_cluster(int size, const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp)
{
    QVector<CT_PointCloudIndexVector *> indices_vec;

    for(int i = 0; i < size; i++)
    {
        CT_PointCloudIndexVector *tree = new CT_PointCloudIndexVector();
        indices_vec.push_back(tree);
    }

    const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in->getPointCloudIndex();

    CT_PointIterator itP(pointCloudIndex);
    size_t i = 0;
    while (itP.hasNext() && !isStopped())
    {
        PointS p = _cloud_in->points.at(i);
        int id = p.treeID;


        itP.next();

        size_t index = itP.currentGlobalIndex();

        if(id>=0 && id < size )
        {
            indices_vec[id]->addIndex(index);
        }
        i++;
    }

    for(int i = 0; i < size; i++)
    {
        CT_StandardItemGroup* cloud_grp = new CT_StandardItemGroup(_grp_out.completeName(), resCpy_res);
        grpCpy_grp->addGroup(cloud_grp);
        CT_Scene* outScene_cluster
                = new CT_Scene(_tree.completeName(), resCpy_res, PS_REPOSITORY->registerPointCloudIndex(indices_vec[i])); // 3) create scene, registering the pointcloudindex
        outScene_cluster->updateBoundingBox(); // 4) don't forget to update the bounding box, to be fitted to filtered points

        cloud_grp->addItemDrawable(outScene_cluster);
    }
}


void ST_StepSegmentation::add_cylinder_data(Tree tree, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp, QString string)
{

}



void ST_StepSegmentation::enrich_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp)
{
    const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();

    size_t size = index->size();

    CT_StandardCloudStdVectorT<float> *distance_cloud = new CT_StandardCloudStdVectorT<float>(size);
    CT_StandardCloudStdVectorT<int> *id_cloud = new CT_StandardCloudStdVectorT<int>(size);

    for(size_t i =0; i < size; i ++)
    {
        PointS p = _cloud_in->points[i];
        float dist = p.distance;
        int id = p.treeID;
        distance_cloud->tAt(i) = dist;
        id_cloud->tAt(i) = id;

    }

    CT_PointsAttributesScalarTemplated<float> * distance =
            new CT_PointsAttributesScalarTemplated<float>(_cloud_out_dist.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(),distance_cloud);
    CT_PointsAttributesScalarTemplated<int> * id =
            new CT_PointsAttributesScalarTemplated<int>(_cloud_out_id.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(),id_cloud);
    grpCpy_grp->addItemDrawable(distance);
    grpCpy_grp->addItemDrawable(id);
}











