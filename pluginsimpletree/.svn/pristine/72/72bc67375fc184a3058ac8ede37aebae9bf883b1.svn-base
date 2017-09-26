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

#include "st_stepfilterclusters.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"

//#define DEF_resultIn_inputResult "inputResult"
//#define DEF_groupIn_inputScene "inputGroup"
//#define DEF_itemIn_scene "inputScene"


#define DEFin_normals_in "no_in"
#define DEFin_stem_in "stem_in"
#define DEF_grp_out "grp_out"
#define DEF_cloud_out "grp_out"
#define DEFin_cluster_in "cluster_in"
#define DEFin_grp_in "grp_cluster_in"


//// Alias for indexing out models
#define DEF_resultOut_translated "extractedResult"
#define DEF_groupOut_pointCloud "extractedGroup"
#define DEF_itemOut_scene "extractedScene"



// Constructor : initialization of parameters
ST_StepFilterClusters::ST_StepFilterClusters(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _minX = 0;
    _maxX = 200;

    _minY = 0;
    _maxY = 200;

    _minZ = 3;
    _maxZ = 200;
}

ST_StepFilterClusters::~ST_StepFilterClusters()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepFilterClusters::getStepDescription() const
{
    return tr("Filters clusters by their bounding box.");
}

// Step detailled description
QString ST_StepFilterClusters::getStepDetailledDescription() const
{
    return tr("Filters clusters by their bounding box." );
}

// Step URL
QString ST_StepFilterClusters::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
}

// Step copy method
CT_VirtualAbstractStep* ST_StepFilterClusters::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepFilterClusters(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepFilterClusters::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy * resultModel = createNewInResultModelForCopy(DEFin_res, tr("Clusters"));
    resultModel->setZeroOrMoreRootGroup();
    resultModel->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("Grp_In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resultModel->addItemModel(DEFin_grp, DEFin_cluster_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Clusters"));

}

// Creation and affiliation of OUT models
void ST_StepFilterClusters::createOutResultModelListProtected()
{
    createNewOutResultModelToCopy(DEFin_res);

}

// Semi-automatic creation of step parameters DialogBox
void ST_StepFilterClusters::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addText("Filters segmented clouds. The user has to give min and or max extension of a segmented clouds bounding box. Only the clousd fullfilling this threshold are taken.");
    configDialog->addEmpty();
    configDialog->addDouble( tr("min x Extension "), " m",  0, 200,1,  _minX, 1, "");
    configDialog->addDouble( tr("max x Extension "), " m",  0, 200,1,  _maxX, 1, "");
    configDialog->addDouble( tr("min y Extension "), " m",  0, 200,1,  _minY, 1, "");
    configDialog->addDouble( tr("max y Extension "), " m",  0, 200,1,  _maxY, 1, "");
    configDialog->addDouble( tr("min z Extension "), " m",  0, 200,1,  _minZ, 1, "");
    configDialog->addDouble( tr("max z Extension "), " m",  0, 200,1,  _maxZ, 1, "");
      dialog_simple_tree(configDialog);
}

void ST_StepFilterClusters::compute()
{
        QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    CT_ResultGroup *outRes = getOutResultList().first();

    size_t totalNumberOfClusters = 0;


    CT_ResultGroupIterator it(outRes, this, DEFin_grp);
    while (it.hasNext() && (!isStopped()))
    {
        CT_AbstractItemGroup *group = (CT_AbstractItemGroup*) it.next();
        const CT_AbstractItemDrawableWithPointCloud *item = (const CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(this, DEFin_cluster_in);

        if(item != NULL)
        {
            ++totalNumberOfClusters;
            Eigen::Vector3d min, max;
            item->getBoundingBox(min,max);
            if(!((std::abs(min(0)-max(0)) > _minX && std::abs(min(0)-max(0)) < _maxX)
                 &&(std::abs(min(1)-max(1)) > _minY && std::abs(min(1)-max(1) < _maxY))
                 &&(std::abs(min(2)-max(2)) > _minZ && std::abs(min(2)-max(2)) < _maxZ)))

                groupsToBeRemoved.append(group);
        }
    }

    size_t numberOfRemovedClusters = groupsToBeRemoved.size();
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }

    PS_LOG->addMessage(LogInterface::info, LogInterface::step, QString(tr("Number of clouds before filtering : %1")).arg(totalNumberOfClusters));
    PS_LOG->addMessage(LogInterface::info, LogInterface::step, QString(tr("Number of clouds rejected : %1")).arg(numberOfRemovedClusters));

    setProgress( 100 );
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
}










void ST_StepFilterClusters::recursiveRemoveGroupIfEmpty(CT_AbstractItemGroup *parent, CT_AbstractItemGroup *group) const
{
    if(parent != NULL)
    {
        parent->removeGroup(group);

        if(parent->isEmpty())
            recursiveRemoveGroupIfEmpty(parent->parentGroup(), parent);
    }
    else
    {
        ((CT_ResultGroup*)group->result())->removeGroupSomethingInStructure(group);
    }
}




