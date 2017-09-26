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


#include "st_stepextractsliceabovedtm.h"

//#ifdef USE_OPENCV

#include "ct_global/ct_context.h"

#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_result/model/inModel/ct_inresultmodelgroup.h"
#include "ct_result/model/outModel/ct_outresultmodelgroup.h"
#include "ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h"

#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_itemdrawable/ct_referencepoint.h"
#include "ct_itemdrawable/ct_image2d.h"
#include "ct_result/ct_resultgroup.h"
#include "ct_itemdrawable/abstract/ct_abstractitemgroup.h"
#include "ct_itemdrawable/ct_scene.h"

#include "ct_pointcloudindex/ct_pointcloudindexvector.h"
#include "ct_iterator/ct_pointiterator.h"
#include "ct_view/ct_stepconfigurabledialog.h"

#include "qdebug.h"
#include <limits>


#define DEF_SearchInResult  "ires"
#define DEF_SearchInGroup   "igrp"
#define DEF_SearchInScene   "isc"

#define DEF_SearchInMNTResult   "mntres"
#define DEF_SearchInMNTGroup    "mntgrp"
#define DEF_SearchInMNT         "mntitem"

#define DEF_SearchOutMNTResult       "rmnt"

ST_StepExtactSliceAboveDTM::ST_StepExtactSliceAboveDTM(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _zmin = 1.0;
    _zmax = 1.6;
}

QString ST_StepExtactSliceAboveDTM::getStepDescription() const
{
    return tr("Smooth extraction of a point slice parallel to the DTM.");
}

QString ST_StepExtactSliceAboveDTM::getStepDetailledDescription() const
{
    return tr("Smooth extraction of a point slice parallel to the DTM.");
}

CT_VirtualAbstractStep* ST_StepExtactSliceAboveDTM::createNewInstance(CT_StepInitializeData &dataInit)
{
    // cree une copie de cette etape
    return new ST_StepExtactSliceAboveDTM(dataInit);
}

//////////////////// PROTECTED //////////////////

void ST_StepExtactSliceAboveDTM::createInResultModelListProtected()
{
    CT_InResultModelGroup *resultMNTModel = createNewInResultModel(DEF_SearchInMNTResult, tr("MNT (Raster)"), "", true);
    resultMNTModel->setZeroOrMoreRootGroup();
    resultMNTModel->addGroupModel("", DEF_SearchInMNTGroup,CT_AbstractItemGroup::staticGetType(), tr("Grp_DTM"),"", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resultMNTModel->addItemModel(DEF_SearchInMNTGroup, DEF_SearchInMNT, CT_Image2D<float>::staticGetType(), tr("DTM"));

    CT_InResultModelGroupToCopy *resultModel = createNewInResultModelForCopy(DEF_SearchInResult, tr("Scène(s)"));
    resultModel->setZeroOrMoreRootGroup();
    resultModel->addGroupModel("", DEF_SearchInGroup,CT_AbstractItemGroup::staticGetType(), tr("Grp_Veg"),"", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resultModel->addItemModel(DEF_SearchInGroup, DEF_SearchInScene, CT_Scene::staticGetType(), tr("Vegetation cloud"));
}

void ST_StepExtactSliceAboveDTM::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();

    configDialog->addDouble(tr("H minimum :"), "m", -100, 100 ,  2, _zmin);
    configDialog->addDouble(tr("H maximum :"), "m", -100, 100 ,  2, _zmax);
    dialog_simple_tree(configDialog);
}

void ST_StepExtactSliceAboveDTM::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEF_SearchInResult);

    if(res != NULL)
        res->addItemModel(DEF_SearchInGroup, _outSceneModel, new CT_Scene(), tr("Slice above DTM"));
}

void ST_StepExtactSliceAboveDTM::compute()
{
    QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    // Récupération du résultat de sortie
    CT_ResultGroup* inMNTResult = getInputResults().at(0);
    CT_ResultGroup *outResult = getOutResultList().at(0);

    CT_ResultItemIterator it(inMNTResult, this, DEF_SearchInMNT);
    if(it.hasNext())
    {
        CT_Image2D<float>* mnt = (CT_Image2D<float>*) it.next();



        if (mnt != NULL)
        {

            double na = mnt->NA();

            PointCloudS::Ptr dtm_cloud ( new PointCloudS);
            int number_cells = mnt->nCells();
            dtm_cloud->points.resize(number_cells);
            for(size_t i = 0; i < number_cells; i++)
            {
                Eigen::Vector3d center;
                mnt->getCellCenterCoordinates(i,center);
                double x = center(0);
                double z =  mnt->valueAtIndex(i);
                double y = center(1);


                if(z == na)
                {
                    x = 30000000;
                    y = 30000000;
                }
                PointS p (x,y,0) ;
                dtm_cloud->points[i] = p;
            }
            pcl::KdTreeFLANN<PointS> kdtree;

            kdtree.setInputCloud (dtm_cloud);



            CT_ResultGroupIterator itsc(outResult, this, DEF_SearchInGroup);
            while(!isStopped() && itsc.hasNext())
            {
                CT_StandardItemGroup *group = (CT_StandardItemGroup*)itsc.next();
                const CT_Scene *in_scene = (const CT_Scene*)group->firstItemByINModelName(this, DEF_SearchInScene);

                if(in_scene != NULL)
                {
                    const CT_AbstractPointCloudIndex *pointCloudIndex = in_scene->getPointCloudIndex();
                    if(pointCloudIndex->size()!=0)
                    {
                        size_t n_points = pointCloudIndex->size();

                        PS_LOG->addMessage(LogInterface::info, LogInterface::step, QString(tr("La scène d'entrée comporte %1 points.")).arg(n_points));

                        CT_PointCloudIndexVector *resPointCloudIndex = new CT_PointCloudIndexVector();

                        // Extraction des points de la placette
                        size_t i = 0;

                        double xmin = std::numeric_limits<double>::max();
                        double ymin = std::numeric_limits<double>::max();
                        double zmin = std::numeric_limits<double>::max();

                        double xmax = -std::numeric_limits<double>::max();
                        double ymax = -std::numeric_limits<double>::max();
                        double zmax = -std::numeric_limits<double>::max();

                        CT_PointIterator itP(pointCloudIndex);
                        while(itP.hasNext() && !isStopped())
                        {
                            const CT_Point &point = itP.next().currentPoint();

                            PointS query (point(0),point(1),0);
                            int K = 3;
                            std::vector<int> pointIdxNKNSearch(K);
                            std::vector<float> pointNKNSquaredDistance(K);

                            kdtree.nearestKSearch (query, K, pointIdxNKNSearch, pointNKNSquaredDistance);

                            float dist1 = 1/std::sqrt(pointNKNSquaredDistance[0]);
                            float dist2 = 1/std::sqrt(pointNKNSquaredDistance[1]);
                            float dist3 = 1/std::sqrt(pointNKNSquaredDistance[2]);

                            float dist_sum = dist1+dist2+dist3;

                            float hauteur1 = mnt->valueAtIndex(pointIdxNKNSearch[0]);
                            float hauteur2 = mnt->valueAtIndex(pointIdxNKNSearch[1]);
                            float hauteur3 = mnt->valueAtIndex(pointIdxNKNSearch[2]);

                            float height = hauteur1*dist1/dist_sum + hauteur2*dist2/dist_sum + hauteur3*dist3/dist_sum;





                            size_t index = itP.currentGlobalIndex();

                            double hauteur;
                            //double zMNT = mnt->valueAtCoords(point(0), point(1));
                            double zMNT = height;

                            if (zMNT != na) {
                                hauteur = point(2) - zMNT;
                            } else {
                                hauteur = -std::numeric_limits<double>::max();
                                qDebug() << "st_stepextractsliuceabonvedtm NA";
                            }

                            if (hauteur >= _zmin && hauteur <= _zmax) {

                                resPointCloudIndex->addIndex(index);

                                if (point(0)<xmin) {xmin = point(0);}
                                if (point(0)>xmax) {xmax = point(0);}
                                if (point(1)<ymin) {ymin = point(1);}
                                if (point(1)>ymax) {ymax = point(1);}
                                if (point(2)<zmin) {zmin = point(2);}
                                if (point(2)>zmax) {zmax = point(2);}
                            }

                            // progres de 0 à 100
                            setProgress(100.0*i/n_points);
                            ++i;
                        }

                        if (resPointCloudIndex->size() > 0)
                        {
                            // creation et ajout de la scene
                            CT_Scene *outScene = new CT_Scene(_outSceneModel.completeName(), outResult);

                            outScene->setPointCloudIndexRegistered(PS_REPOSITORY->registerPointCloudIndex(resPointCloudIndex));
                            outScene->setBoundingBox(xmin,ymin,zmin,xmax,ymax,zmax);
                            group->addItemDrawable(outScene);

                            PS_LOG->addMessage(LogInterface::info, LogInterface::step, QString(tr("La scène extraite comporte %1 points.")).arg(outScene->getPointCloudIndex()->size()));

                        } else {
                            delete resPointCloudIndex;
                            PS_LOG->addMessage(LogInterface::info, LogInterface::step, tr("Aucun point n'est dans l'emprise choisie"));

                        }
                    }else {
                        groupsToBeRemoved.push_back(group);
                    }

                } else {
                    groupsToBeRemoved.push_back(group);
                }
            }
        }
    }
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
}
//#endif
