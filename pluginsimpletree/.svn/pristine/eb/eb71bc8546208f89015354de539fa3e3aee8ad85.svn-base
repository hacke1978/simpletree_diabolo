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


#include "st_stepsegmentedchm.h"

#ifdef USE_OPENCV
#include "ct_global/ct_context.h"

#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_result/model/outModel/ct_outresultmodelgroupcopy.h"
#include "ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h"

#include "ct_result/ct_resultgroup.h"

#include "ct_itemdrawable/ct_scene.h"
#include "ct_itemdrawable/ct_attributeslist.h"
#include "ct_itemdrawable/ct_image2d.h"
#include "ct_pointcloudindex/ct_pointcloudindexvector.h"
#include "ct_iterator/ct_pointiterator.h"
#include "ct_iterator/ct_groupiterator.h"
#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_itemdrawable/tools/image2dtools/ct_image2dnaturalneighboursinterpolator.h"


#include <math.h>
#include <stdlib.h>
#include <limits>

#define DEF_ResultScene   "resSc"
#define DEF_rootGroup   "rootGrp"
#define DEF_DTM   "DTM"
#define DEF_ground   "grnd"
#define DEF_GroupScene   "grpSc"
#define DEF_Scene   "Sc"


#define CANOPY_THICKNESS 2.0


ST_StepSegmentedCHM::ST_StepSegmentedCHM(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
}

QString ST_StepSegmentedCHM::getStepDescription() const
{
    return tr("Create Segmented CHM");
}

QString ST_StepSegmentedCHM::getStepDetailledDescription() const
{
    return tr("");
}

CT_VirtualAbstractStep* ST_StepSegmentedCHM::createNewInstance(CT_StepInitializeData &dataInit)
{
    // cree une copie de cette etape
    return new ST_StepSegmentedCHM(dataInit);
}

/////////////////////// PROTECTED ///////////////////////

void ST_StepSegmentedCHM::createInResultModelListProtected()
{  
    CT_InResultModelGroupToCopy *resultModel = createNewInResultModelForCopy(DEF_ResultScene);

    resultModel->setZeroOrMoreRootGroup();
    resultModel->addGroupModel("", DEF_rootGroup);
    resultModel->addItemModel(DEF_rootGroup, DEF_DTM, CT_Image2D<float>::staticGetType(), tr("DTM"));
    resultModel->addItemModel(DEF_rootGroup, DEF_ground, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Ground points"));

    resultModel->addGroupModel(DEF_rootGroup, DEF_GroupScene);
    resultModel->addItemModel(DEF_GroupScene, DEF_Scene, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Segmented scene"));
}

void ST_StepSegmentedCHM::createPostConfigurationDialog()
{
        //CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();

}

void ST_StepSegmentedCHM::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resultModel = createNewOutResultModelToCopy(DEF_ResultScene);
    if(resultModel != NULL) {
        resultModel->addItemModel(DEF_rootGroup, _outCHMModelName, new CT_Image2D<float>(), tr("CHM"));
        resultModel->addItemModel(DEF_rootGroup, _outTreeIDModelName, new CT_Image2D<int>(), tr("TreeID"));
        resultModel->addItemModel(DEF_rootGroup, _outGroundDensityModelName, new CT_Image2D<int>(), tr("Ground Density"));
        resultModel->addItemModel(DEF_rootGroup, _outCanopyDensityModelName, new CT_Image2D<int>(), tr("Canopy Density"));
        resultModel->addItemModel(DEF_GroupScene, _outSceneIDModelContainerName, new CT_AttributesList(), tr("TreeID"));
        resultModel->addItemAttributeModel(_outSceneIDModelContainerName, _outSceneIDModelName, new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_ID), tr("TreeID"));
    }
}

void ST_StepSegmentedCHM::compute()
{
    QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    const QList<CT_ResultGroup*> &outResList = getOutResultList();
    // récupération des modéles out
    CT_ResultGroup *outResult = outResList.at(0);

    CT_ResultGroupIterator it(outResult, this, DEF_rootGroup);
    while (!isStopped() && it.hasNext())
    {
        CT_StandardItemGroup* rootGroup = (CT_StandardItemGroup*) it.next();

        CT_Image2D<float>* dtm = (CT_Image2D<float>*)rootGroup->firstItemByINModelName(this, DEF_DTM);
        CT_AbstractItemDrawableWithPointCloud* groundPoints = (CT_AbstractItemDrawableWithPointCloud*)rootGroup->firstItemByINModelName(this, DEF_ground);

        if(dtm != NULL && groundPoints != NULL)
        {

            CT_Image2D<float>* dsm = new CT_Image2D<float>(NULL, NULL, dtm->minX(), dtm->minY(), dtm->colDim(), dtm->linDim(), dtm->resolution(), dtm->level(), dtm->NA(), -std::numeric_limits<float>::max());
            CT_Image2D<float>* chm = new CT_Image2D<float>(_outCHMModelName.completeName(), outResult, dtm->minX(), dtm->minY(), dtm->colDim(), dtm->linDim(), dtm->resolution(), dtm->level(), dtm->NA(), -1);
            CT_Image2D<int>* treeId = new CT_Image2D<int>(_outTreeIDModelName.completeName(), outResult, dtm->minX(), dtm->minY(), dtm->colDim(), dtm->linDim(), dtm->resolution(), dtm->level(), -1, -1);
            CT_Image2D<int>* groundDensity = new CT_Image2D<int>(_outGroundDensityModelName.completeName(), outResult, dtm->minX(), dtm->minY(), dtm->colDim(), dtm->linDim(), dtm->resolution(), dtm->level(), -1, 0);
            CT_Image2D<int>* canopyDensity = new CT_Image2D<int>(_outCanopyDensityModelName.completeName(), outResult, dtm->minX(), dtm->minY(), dtm->colDim(), dtm->linDim(), dtm->resolution(), dtm->level(), -1, 0);

            // 1- Create DSM and TreeID rasters
            int sceneCounter = 0;
            CT_GroupIterator itGrp(rootGroup, this, DEF_GroupScene);
            while (itGrp.hasNext())
            {
                CT_StandardItemGroup* group = (CT_StandardItemGroup*) itGrp.next();

                if (group != NULL)
                {
                    sceneCounter++;
                    CT_Scene *scene = (CT_Scene*)group->firstItemByINModelName(this, DEF_Scene);

                    if (scene != NULL)
                    {
                        CT_AttributesList* attList = new CT_AttributesList(_outSceneIDModelContainerName.completeName(), outResult);
                        attList->addItemAttribute(new CT_StdItemAttributeT<int>(_outSceneIDModelName.completeName(), CT_AbstractCategory::DATA_ID, outResult, sceneCounter));
                        group->addItemDrawable(attList);

                        const CT_AbstractPointCloudIndex *pointCloudIndex = scene->getPointCloudIndex();
                        CT_PointIterator itP(pointCloudIndex);
                        while(itP.hasNext() && !isStopped())
                        {
                            const CT_Point &point =itP.next().currentPoint();

                            if (dsm->setMaxValueAtCoords(point(0), point(1), point(2)))
                            {
                                treeId->setValueAtCoords(point(0), point(1), sceneCounter);
                            }
                        }
                    }
                }
            }

            // Create ground Points density raster
            const CT_AbstractPointCloudIndex *pointCloudIndexGround = groundPoints->getPointCloudIndex();
            CT_PointIterator itPGrd(pointCloudIndexGround);
            while(itPGrd.hasNext() && !isStopped())
            {
                const CT_Point &point = itPGrd.next().currentPoint();

                groundDensity->addValueAtCoords(point(0), point(1), 1);
            }

            // Create canopy density points density raster
            CT_GroupIterator itGrp2(rootGroup, this, DEF_GroupScene);
            while (itGrp2.hasNext())
            {
                CT_StandardItemGroup* group = (CT_StandardItemGroup*) itGrp2.next();

                if (group != NULL)
                {
                    const CT_Scene *scene = (CT_Scene*)group->firstItemByINModelName(this, DEF_Scene);

                    if (scene != NULL)
                    {
                        const CT_AbstractPointCloudIndex *pointCloudIndex = scene->getPointCloudIndex();
                        CT_PointIterator itP(pointCloudIndex);
                        while(itP.hasNext() && !isStopped())
                        {
                            const CT_Point &point =itP.next().currentPoint();

                            if ((dsm->valueAtCoords(point(0), point(1)) - point(2)) <= CANOPY_THICKNESS)
                            {
                                canopyDensity->addValueAtCoords(point(0), point(1), 1);
                            }
                        }
                    }
                }
            }

            // Create CHM
            for (size_t index = 0 ; index < dsm->nCells() ; index++)
            {
                if (dsm->valueAtIndex(index) == -std::numeric_limits<float>::max())
                {
                    chm->setValueAtIndex(index, 0);
                } else if (dtm->valueAtIndex(index) == dtm->NA())
                {
                    chm->setValueAtIndex(index, chm->NA());
                } else {
                    chm->setValueAtIndex(index, dsm->valueAtIndex(index) - dtm->valueAtIndex(index));
               }
            }

            chm->computeMinMax();
            treeId->computeMinMax();
            groundDensity->computeMinMax();
            canopyDensity->computeMinMax();

            rootGroup->addItemDrawable(chm);
            rootGroup->addItemDrawable(treeId);
            rootGroup->addItemDrawable(groundDensity);
            rootGroup->addItemDrawable(canopyDensity);

            delete dsm;
        }
    }
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }

}
#endif
