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

#include "st_stepfilterstempointscircle.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"
#define DEFin_cloud_in_slice "cloud_in_sclice"

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
ST_StepFilterStemPointsCircle::ST_StepFilterStemPointsCircle(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
}

ST_StepFilterStemPointsCircle::~ST_StepFilterStemPointsCircle()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepFilterStemPointsCircle::getStepDescription() const
{
    return tr("Filters the stem with a cicular buffer.");
}

// Step detailled description
QString ST_StepFilterStemPointsCircle::getStepDetailledDescription() const
{
    return tr("Filters the stem with a cicular buffer." );
}

// Step URL
QString ST_StepFilterStemPointsCircle::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    // return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepFilterStemPointsCircle::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepFilterStemPointsCircle(dataInit);
}


// Creation and affiliation of IN models
void ST_StepFilterStemPointsCircle::createInResultModelListProtected()
{



    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("Result_In"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("Grp_In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);

    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Tree cloud"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in_slice, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Tree cloud slice"));
}

// Creation and affiliation of OUT models
void ST_StepFilterStemPointsCircle::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {

        res->addItemModel(DEFin_grp, _cloud_noise, new CT_Scene(), tr("Not classified stem"));
        res->addItemModel(DEFin_grp, _cloud_stem, new CT_Scene(), tr("Classified stem"));
    }
}

// Semi-automatic creation of step parameters DialogBox
void ST_StepFilterStemPointsCircle::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addTitle(tr("Select a radius around the estimated stem position to include or exclude points:"));
    configDialog->addDouble( tr("radius :"), "m", 0 , 35, 3, _distance, 1);
    dialog_simple_tree(configDialog);

}

void ST_StepFilterStemPointsCircle::compute()
{
    QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);

    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    while (itCpy_grp.hasNext() && !isStopped())
    {

        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in
                = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);

        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in_slice
                = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in_slice);

        if(itemCpy_cloud_in!= NULL && itemCpy_cloud_in_slice!=NULL)
        {
            const CT_AbstractPointCloudIndex *pointCloudIndex_slice = itemCpy_cloud_in_slice->getPointCloudIndex();
            const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in->getPointCloudIndex();

            if(pointCloudIndex->size()!=0 && pointCloudIndex_slice->size()!=0)
            {
                CT_PointIterator itP2(pointCloudIndex_slice);

                setProgress(1);

                float x_total = 0;
                float y_total = 0;
                int number = 0;
                while (itP2.hasNext() && !isStopped())
                {
                    itP2.next();
                    CT_Point ctp =  itP2.currentPoint();

                    x_total += ctp(0);
                    y_total += ctp(1);
                    number ++;
                }

                if(pointCloudIndex_slice->size()!=0)
                {
                    x_total = x_total / number;
                    y_total = y_total / number;
                }


                setProgress(4);


                CT_PointCloudIndexVector *extracted_cloud_stem = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
                CT_PointCloudIndexVector *extracted_cloud_no_stem = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index

                PointS center;
                center.x = x_total;
                center.y = y_total;
                center.z = 0;
                CT_PointIterator itP(pointCloudIndex);
                while (itP.hasNext() && !isStopped())
                {
                    itP.next();
                    size_t index = itP.currentGlobalIndex();
                    CT_Point ctp =  itP.currentPoint();
                    PointS p;
                    p.x = ctp(0);
                    p.y = ctp(1);
                    p.z = 0;

                    float dist = SimpleMath<float>::get_distance(p,center);


                    if(dist < _distance)
                    {
                        extracted_cloud_stem->addIndex(index);
                    } else {
                        extracted_cloud_no_stem->addIndex(index);
                    }



                }
                {
                    CT_Scene* outScene = new CT_Scene(_cloud_noise.completeName(),
                                                      resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extracted_cloud_no_stem));
                    outScene->updateBoundingBox();
                    grpCpy_grp->addItemDrawable(outScene);
                }

                {
                    CT_Scene* outScene = new CT_Scene(_cloud_stem.completeName(),
                                                      resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extracted_cloud_stem)); // 3) create scene, registering the pointcloudindex
                    outScene->updateBoundingBox(); // 4) don't forget to update the bounding box, to be fitted to filtered points

                    grpCpy_grp->addItemDrawable(outScene);
                }

                setProgress(99);
            }else {
                groupsToBeRemoved.push_back(grpCpy_grp);
            }



        } else {
            groupsToBeRemoved.push_back(grpCpy_grp);
        }


    }
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
}














