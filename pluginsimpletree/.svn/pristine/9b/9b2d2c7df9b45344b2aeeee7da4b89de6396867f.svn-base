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
#include "st_stepsegmentation3.h"

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
ST_StepSegmentationExtract::ST_StepSegmentationExtract(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{

}

ST_StepSegmentationExtract::~ST_StepSegmentationExtract()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepSegmentationExtract::getStepDescription() const
{
    return tr("Segmentation step 3 - Splits up the cloud into subclouds according to the segmentation.");
}

// Step detailled description
QString ST_StepSegmentationExtract::getStepDetailledDescription() const
{
    return tr("Segmentation step 3 - Splits up the cloud into subclouds according to the segmentation.");
}

// Step URL
QString ST_StepSegmentationExtract::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    // return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepSegmentationExtract::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepSegmentationExtract(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepSegmentationExtract::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resIn_res->addItemModel(DEFin_grp, DEFin_source_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Plot"));
    resIn_res->addItemModel(DEFin_grp, DEFin_id_in,  CT_PointsAttributesScalarTemplated<int>::staticGetType(),tr("Tree ID"));
}

// Creation and affiliation of OUT models
void ST_StepSegmentationExtract::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {
        res->addGroupModel(DEFin_grp, _grp_out, new CT_StandardItemGroup(),tr("Isolated Tree Grp"));
        res->addItemModel(_grp_out, _cloud_out, new CT_Scene(),tr("Isolated Tree"));
    }

}

// Semi-automatic creation of step parameters DialogBox
void ST_StepSegmentationExtract::createPostConfigurationDialog()
{
    //    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addTitle(tr("You will find help and description about the paramters in the following publication."));
    configDialog->addTitle(tr("This is also the correct citation you should give for scientific publications."));
    configDialog->addEmpty();
    configDialog->addTitle(tr("Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P."));
    configDialog->addTitle(tr("<em>SimpleTree —An Efficient Open Source Tool to Build Tree Models from TLS Clouds.</em>"));
    configDialog->addTitle(tr("Forests <b>2015</b>, 6, 4245-4294. "));
    //    configDialog->addDouble( tr("clustering distance"), "m",  0.01, 0.2,2,  _distance, 1, "the clustering distance.");
}

void ST_StepSegmentationExtract::compute()
{






    DijkstraCoefficients cf;

    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);

    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    while (itCpy_grp.hasNext() && !isStopped())
    {

        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in
                = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_source_in);

        CT_PointsAttributesScalarTemplated<int>* itemCpy_id_in
                = (CT_PointsAttributesScalarTemplated<int>*)grpCpy_grp->firstItemByINModelName(this, DEFin_id_in);

        const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in->getPointCloudIndex();
        int index  = 0;
        int max  = -1;
        CT_PointIterator itP2(pointCloudIndex);
        while (itP2.hasNext() && !isStopped())
        {
            itP2.next();
            int id = itemCpy_id_in->valueAt(index);
            if(id>max)
            {
                max = id;
            }


            index++;
        }
        max++;

        QVector<CT_PointCloudIndexVector *> indices_vec;

        for(int i = 0; i < max; i++)
        {
            CT_PointCloudIndexVector *tree = new CT_PointCloudIndexVector();
            indices_vec.push_back(tree);
        }
        int i = 0;
        CT_PointIterator itP(pointCloudIndex);
        while (itP.hasNext() && !isStopped())
        {
            int id = itemCpy_id_in->valueAt(i);


            itP.next();

            size_t index = itP.currentGlobalIndex();

            if(id>=0 && id < max )
            {
                indices_vec[id]->addIndex(index);
            }
            i++;
        }

        for(int i = 0; i < max; i++)
        {
            if(indices_vec[i]->size()>cf.TREE_CLUSTER_MIN_SIZE);
            {
                CT_StandardItemGroup* cloud_grp = new CT_StandardItemGroup(_grp_out.completeName(), resCpy_res);
                grpCpy_grp->addGroup(cloud_grp);
                CT_Scene* outScene_cluster
                        = new CT_Scene(_cloud_out.completeName(), resCpy_res, PS_REPOSITORY->registerPointCloudIndex(indices_vec[i])); // 3) create scene, registering the pointcloudindex
                outScene_cluster->updateBoundingBox(); // 4) don't forget to update the bounding box, to be fitted to filtered points

                cloud_grp->addItemDrawable(outScene_cluster);
            }
        }




    }
}

void ST_StepSegmentationExtract::add_cluster( const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp)
{  


}


void ST_StepSegmentationExtract::add_cylinder_data(Tree tree, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp, QString string)
{

}



void ST_StepSegmentationExtract::enrich_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp)
{

}











