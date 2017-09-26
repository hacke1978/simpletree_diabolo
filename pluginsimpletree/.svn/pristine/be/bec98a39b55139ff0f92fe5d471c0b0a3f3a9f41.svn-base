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

#include "st_stepbuffering.h"


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
ST_StepBuffering::ST_StepBuffering(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _range = 0.05;
}

ST_StepBuffering::~ST_StepBuffering()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepBuffering::getStepDescription() const
{
    return tr("Buffers two clouds.");
}

// Step detailled description
QString ST_StepBuffering::getStepDetailledDescription() const
{
    return tr("Each point of the target cloud near a user given range of the source cloud is added to the output. Can be used in combination of VoxelGrid Filter to improve clustering runtime.");
}

// Step URL
QString ST_StepBuffering::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
}

// Step copy method
CT_VirtualAbstractStep* ST_StepBuffering::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepBuffering(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepBuffering::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resIn_res->addItemModel(DEFin_grp, DEFin_source_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("source cloud"));
    resIn_res->addItemModel(DEFin_grp, DEFin_target_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("target cloud"));

}

// Creation and affiliation of OUT models
void ST_StepBuffering::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {

        res->addItemModel(DEFin_grp, _cloud_out_target, new CT_Scene(),tr("bufferd cloud"));
    }

}

// Semi-automatic creation of step parameters DialogBox
void ST_StepBuffering::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();

    configDialog->addDouble( tr("buffer distance"), "m",  0.01, 0.2,2,  _range, 1, "the buffer distance.");

    dialog_simple_tree(configDialog);
}

void ST_StepBuffering::compute()
{
    QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    while (itCpy_grp.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        CT_AbstractItemDrawableWithPointCloud* itemCpy_source_in
                = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_source_in);
        CT_AbstractItemDrawableWithPointCloud* itemCpy_target_in
                = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_target_in);
        if(itemCpy_source_in!=NULL && itemCpy_target_in!=NULL)
        {   ConvertCTtoST ctst(itemCpy_source_in,16);
            ctst.convert();
            _cloud_in_source = ctst.get_cloud();
            warning_geo_referenced(_cloud_in_source);
            ConvertCTtoST ctst2(itemCpy_target_in,16);
            ctst2.convert();
            _cloud_in_target = ctst2.get_cloud();

            if(_cloud_in_source->points.size()!=0 && _cloud_in_target->points.size()!=0)
            {    std::vector<bool> is_cluster;
                for(size_t i = 0; i < _cloud_in_target->points.size(); i++)
                {
                    is_cluster.push_back(false);
                }

                QSharedPointer<pcl::octree::OctreePointCloudSearch<PointS> > octree (new pcl::octree::OctreePointCloudSearch<PointS> (SimpleMath<float>::_OCTREE_RESOLUTION));
                octree->setInputCloud (_cloud_in_target);
                octree->addPointsFromInputCloud ();

                for(size_t i = 0; i < _cloud_in_source->points.size(); i++)
                {
                    PointS query = _cloud_in_source->points.at(i);
                    std::vector<int> pointIdxRadiusSearch;
                    std::vector<float> pointRadiusSquaredDistance;
                    octree->radiusSearch(query, _range, pointIdxRadiusSearch,
                                         pointRadiusSquaredDistance);
                    for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++)
                    {
                        is_cluster[pointIdxRadiusSearch[j]] = true;
                    }

                }

                const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_target_in->getPointCloudIndex();
                CT_PointCloudIndexVector *buffered_cloud = new CT_PointCloudIndexVector();
                CT_PointIterator itP(pointCloudIndex);
                size_t k = 0;
                while (itP.hasNext() && !isStopped())
                {
                    itP.next();
                    size_t index = itP.currentGlobalIndex();
                    if(is_cluster[k])
                    {
                        buffered_cloud->addIndex(index);
                    }
                    k++;
                }

                if (buffered_cloud->size() > 0)
                {
                    CT_Scene* outScene_cluster
                            = new CT_Scene(_cloud_out_target.completeName(), resCpy_res, PS_REPOSITORY->registerPointCloudIndex(buffered_cloud)); // 3) create scene, registering the pointcloudindex
                    outScene_cluster->updateBoundingBox(); // 4) don't forget to update the bounding box, to be fitted to filtered points

                    grpCpy_grp->addItemDrawable(outScene_cluster);
                }

            }else
            {
                groupsToBeRemoved.push_back(grpCpy_grp);
            }
        }else
        {
            groupsToBeRemoved.push_back(grpCpy_grp);
        }


    }
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
}


