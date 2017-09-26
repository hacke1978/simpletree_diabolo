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
#include "st_stepvoxelgridfilter.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"



//// Alias for indexing out models
#define DEF_resultOut_translated "translatedResult"
#define DEF_groupOut_pointCloud "translatedGroup"
#define DEF_itemOut_scene "translatedScene"



// Constructor : initialization of parameters
ST_StepVoxelGridFilter::ST_StepVoxelGridFilter(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _voxel_grid_size = 0.01;
}

ST_StepVoxelGridFilter::~ST_StepVoxelGridFilter()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepVoxelGridFilter::getStepDescription() const
{
    return tr("VoxelGrid filter.");
}

// Step detailled description
QString ST_StepVoxelGridFilter::getStepDetailledDescription() const
{
    return tr("VoxelGrid filter." );
}

// Step URL
QString ST_StepVoxelGridFilter::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
}

// Step copy method
CT_VirtualAbstractStep* ST_StepVoxelGridFilter::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepVoxelGridFilter(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepVoxelGridFilter::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("Result_In"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("Grp_In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Tree_Cloud"));
}

// Creation and affiliation of OUT models
void ST_StepVoxelGridFilter::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {
        res->addItemModel(DEFin_grp, _outScene_ModelName, new CT_Scene(), tr("Downscaled_Cloud"));
    }
}

// Semi-automatic creation of step parameters DialogBox
void ST_StepVoxelGridFilter::createPostConfigurationDialog()
{

    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addText("This method is an information loss reduced down-sampling approach. Over the point cloud a");
    configDialog->addText(" three-dimensional voxel grid is generated. The cell size <b>size</b> of the voxels equals cellsize. If a voxel contains");
    configDialog->addText(" at least one point, for the output cloud a point is generated. The output point is the centroid of all");
    configDialog->addText(" contained points, which minimizes the information loss of the down-sampling procedure compared to");
    configDialog->addText("using the voxel center point.");
    configDialog->addDouble("The size of the voxel cells.","size in m",0.001,0.1,3,_voxel_grid_size);
    dialog_simple_tree(configDialog);


}

void ST_StepVoxelGridFilter::compute()
{
        QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    while (itCpy_grp.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        if(itemCpy_cloud_in!=NULL)
        {
            create_simple_tree_cloud(itemCpy_cloud_in);
            if(_cloud_out->points.size()!=0)
            {

                CT_AbstractUndefinedSizePointCloud* mpcir = PS_REPOSITORY->createNewUndefinedSizePointCloud();
                size_t size = _cloud_out->points.size();
                warning_geo_referenced(_cloud_out);
                for(size_t i = 0; i < size; i++)
                {
                    PointS p = _cloud_out->points.at(i);


                    CT_Point point;
                    point[0] = p.x;
                    point[1] = p.y;
                    point[2] = p.z;
                    mpcir->addPoint(point);

                }
                CT_NMPCIR pcir = PS_REPOSITORY->registerUndefinedSizePointCloud(mpcir);
                CT_Scene* scene = new CT_Scene(_outScene_ModelName.completeName(), resCpy_res, pcir);
                scene->updateBoundingBox();
                grpCpy_grp->addItemDrawable(scene);
            }else {
                groupsToBeRemoved.push_back(grpCpy_grp);
            }


        } else {
            groupsToBeRemoved.push_back(grpCpy_grp);
        }


    }
    _cloud_out.reset(new PointCloudS);
    _cloud_in.reset(new PointCloudS);
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
}



void ST_StepVoxelGridFilter::create_simple_tree_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in)
{
    if (itemCpy_cloud_in != NULL)
    {
        const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();
        int knn = 16;
        size_t size = 0;
        size = index->size();
        _cloud_in.reset(new PointCloudS);
        _cloud_in->width = size;
        _cloud_in->height = 1;
        if(size > 0)
        {
            _cloud_in->points.resize(size);
            size_t i = 0;
            CT_PointIterator it (index);
            while(it.hasNext())
            {
                const CT_Point &internalPoint = it.next().currentPoint();
                PointS p (internalPoint(0),internalPoint(1),internalPoint(2));
                _cloud_in->points[i] = p;
                ++i;
            }
        }

        _cloud_out = pcl_voxel_grid_filter(_cloud_in, this, _voxel_grid_size);
    }
}



