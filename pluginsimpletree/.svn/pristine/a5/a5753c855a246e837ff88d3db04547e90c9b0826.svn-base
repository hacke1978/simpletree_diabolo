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

#include "st_extractmajorbranches.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"


ST_StepExtractMajorBranches::ST_StepExtractMajorBranches(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{

    _e1_min = 0;
    _e1_max = 0.15;
    _e2_min = 0.25;
    _e2_max = 1;
    _e3_min = 0;
    _e3_max = 0.8;
    _number_clusters = 3;
}

ST_StepExtractMajorBranches::~ST_StepExtractMajorBranches()
{
}

// Step description (tooltip of contextual menu)
QString ST_StepExtractMajorBranches::getStepDescription() const
{
    return tr("Extracts the major branch points.");
}

// Step detailled description
QString ST_StepExtractMajorBranches::getStepDetailledDescription() const
{
    return tr("The stem points are detected in a semi automatic manner. The user has to give min and max for the Eigenvalues after a principal component analysis"
              "is applied on each points' neighborhood." );
}

// Step URL
QString ST_StepExtractMajorBranches::getStepURL() const
{
    //return tr("http://www.simpletree.uni-freiburg.de/news.html");
    return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepExtractMajorBranches::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepExtractMajorBranches(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepExtractMajorBranches::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("Result_In"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("Grp_In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Tree_Cloud"));
}

// Creation and affiliation of OUT models
void ST_StepExtractMajorBranches::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);
    if(resCpy_res != NULL)
    {
        resCpy_res->addItemModel(DEFin_grp, _outScene_stem, new CT_Scene(), tr("Extracted_Stem"));
        resCpy_res->addItemModel(DEFin_grp, _outScene_twigs, new CT_Scene(), tr("Extracted_Twigs"));
    }
}

// Semi-automatic creation of step parameters DialogBox
void ST_StepExtractMajorBranches::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addText("A PCA is performed on each input points neighbourhood of the input cloud."
                          "If the eigenvalues E1, E2 and E3 fulfil user-given thresholds, the point is accepted as a stem point.");
    configDialog->addEmpty();
    configDialog->addDouble( tr("E1 min"), "",  0, 1,2,  _e1_min, 1, "E1 min.");
    configDialog->addDouble( tr("E1 max"), "",  0, 1,2,  _e1_max, 1, "E1 max.");
    configDialog->addDouble( tr("E2 min"), "",  0, 1,2,  _e2_min, 1, "E2 min.");
    configDialog->addDouble( tr("E2 max"), "",  0, 1,2,  _e2_max, 1, "E2 max.");
    configDialog->addDouble( tr("E3 min"), "",  0, 1,2,  _e3_min, 1, "E3 min.");
    configDialog->addDouble( tr("E3 max"), "",  0, 1,2,  _e3_max, 1, "E3 max.");
    configDialog->addInt(tr("number of clusters" ),"",1,1000, _number_clusters, "The maximum number of clusters");
    dialog_simple_tree(configDialog);
}

void ST_StepExtractMajorBranches::compute()
{
    QList<CT_AbstractItemGroup*> groupsToBeRemoved;

    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    while (itCpy_grp.hasNext() && !isStopped())
    {
        setProgress(5);
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in =
                (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        if(itemCpy_cloud_in!=NULL)
        {
            setProgress(10);
            PointCloudS::Ptr cloud_in = pcl_CT_to_PCL_cloud(itemCpy_cloud_in, this,16, false, false);
            if(cloud_in->points.size()!=0)
            {
                setProgress(15);
                EnrichCloud (cloud_in, 16, 0, true);
                warning_geo_referenced(cloud_in);
                setProgress(20);
                StemPointDetection stempts (_e1_min,_e1_max,_e2_min,_e2_max,_e3_min,_e3_max,0.035,cloud_in,  _number_clusters) ;
                stempts.compute();
                setProgress(35);
                const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in->getPointCloudIndex();
                size_t nbPoints = pointCloudIndex->size();
                CT_PointCloudIndexVector *extractedCloud = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
                CT_PointCloudIndexVector *extractedCloud2 = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
                CT_PointIterator itP(pointCloudIndex);
                size_t i = 0;
                while (itP.hasNext() && !isStopped())
                {
                    itP.next();
                    size_t index = itP.currentGlobalIndex();
                    const float stem_flag = cloud_in->points.at(i).is_stem;
                    if ( stem_flag == 1)
                    {
                        extractedCloud->addIndex(index);
                    } else {
                        extractedCloud2->addIndex(index);
                    }
                    setProgress( 99.0*i++ /nbPoints );
                }
                setProgress(50);
                if (extractedCloud->size() > 0)
                {
                    CT_Scene* outScene = new CT_Scene(_outScene_stem.completeName(),
                                                      resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud));
                    outScene->updateBoundingBox();
                    grpCpy_grp->addItemDrawable(outScene);
                }
                if(extractedCloud2->size() > 0)
                {
                    CT_Scene* outScene = new CT_Scene(_outScene_twigs.completeName(),
                                                      resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud2));
                    outScene->updateBoundingBox();
                    grpCpy_grp->addItemDrawable(outScene);
                }
            } else {
                groupsToBeRemoved.push_back(grpCpy_grp);
            }
        } else {
            groupsToBeRemoved.push_back(grpCpy_grp);
        }
    }
    setProgress(99);
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
}




