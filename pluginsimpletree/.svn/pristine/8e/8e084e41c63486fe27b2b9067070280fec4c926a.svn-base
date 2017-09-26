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

#include "st_stepmergeclouds.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"
//#define DEFin_cloud_in2 "cloud_in2"



//// Alias for indexing out models
#define DEF_resultOut_translated "extractedResult"
#define DEF_groupOut_pointCloud "extractedGroup"
#define DEF_itemOut_scene "extractedScene"



// Constructor : initialization of parameters
ST_StepMergeClouds::ST_StepMergeClouds(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{

}

ST_StepMergeClouds::~ST_StepMergeClouds()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepMergeClouds::getStepDescription() const
{
    return tr("Merges multiple clouds.");
}

// Step detailled description
QString ST_StepMergeClouds::getStepDetailledDescription() const
{
    return tr("Merges multiple clouds belonging to the same Result." );
}

// Step URL
QString ST_StepMergeClouds::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
}

// Step copy method
CT_VirtualAbstractStep* ST_StepMergeClouds::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepMergeClouds(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepMergeClouds::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("Result_In"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("Grp_In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Cluster"),"",CT_InAbstractModel::C_ChooseMultipleIfMultiple);

}

// Creation and affiliation of OUT models
void ST_StepMergeClouds::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {
        res->addItemModel(DEFin_grp, _outScene_merge, new CT_Scene(), tr("Merged_Cloud"));
    }

}

// Semi-automatic creation of step parameters DialogBox
void ST_StepMergeClouds::createPostConfigurationDialog()
{    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
     configDialog->addTitle(tr("This step merges selected clouds within one group."));
      configDialog->addEmpty();
       dialog_simple_tree(configDialog);
}

void ST_StepMergeClouds::compute()
{
    QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);

    // IN results browsing

    // COPIED results browsing
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    while (itCpy_grp.hasNext() && !isStopped())
    {

        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        CT_PointCloudIndexVector *merged_cloud = new CT_PointCloudIndexVector();
        merged_cloud->setSortType(CT_AbstractCloudIndex::NotSorted);

        CT_ItemIterator it (grpCpy_grp,this,DEFin_cloud_in);
        std::vector<size_t> index_vec;

        while(it.hasNext())
        {
            CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in = (CT_AbstractItemDrawableWithPointCloud*)it.next();
            if(itemCpy_cloud_in!=NULL)
            {                const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in->getPointCloudIndex();
                CT_PointIterator itP(pointCloudIndex);
                while (itP.hasNext() && !isStopped())
                {
                    itP.next();
                    size_t index = itP.currentGlobalIndex();
                    index_vec.push_back(index);
//                    merged_cloud->addIndex(index);
                }
            } else {
                groupsToBeRemoved.push_back(grpCpy_grp);
            }


        }
        std::sort(index_vec.begin(),index_vec.end());
        for(size_t i = 0; i < index_vec.size(); i++)
        {
            size_t ind = index_vec.at(i);
            merged_cloud->addIndex(ind);
        }
        if (merged_cloud->size() > 0)
        {
            merged_cloud->setSortType(CT_AbstractCloudIndex::SortedInAscendingOrder);
            CT_Scene* outScene_cluster
                    = new CT_Scene(_outScene_merge.completeName(), resCpy_res, PS_REPOSITORY->registerPointCloudIndex(merged_cloud));
            outScene_cluster->updateBoundingBox();

            grpCpy_grp->addItemDrawable(outScene_cluster);
        }
    }
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
}















