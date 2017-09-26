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

#include "st_stepextractlargestcluster.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"

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
ST_StepExtractLargestCluster::ST_StepExtractLargestCluster(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{

}

ST_StepExtractLargestCluster::~ST_StepExtractLargestCluster()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepExtractLargestCluster::getStepDescription() const
{
    return tr("Euclidean clustering denoising.");
}

// Step detailled description
QString ST_StepExtractLargestCluster::getStepDetailledDescription() const
{
    return tr("An euclidean clustering operation. The routine produces two output clouds, one containing all large clusters, one all small." );
}

// Step URL
QString ST_StepExtractLargestCluster::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
}

// Step copy method
CT_VirtualAbstractStep* ST_StepExtractLargestCluster::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepExtractLargestCluster(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepExtractLargestCluster::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("Result_In"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("Grp_In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);

    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Point Cloud"));

}

// Creation and affiliation of OUT models
void ST_StepExtractLargestCluster::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {
        res->addItemModel(DEFin_grp, _outScene_rest, new CT_Scene(), tr("Smallest_Clusters"));
        res->addItemModel(DEFin_grp, _outScene_cluster, new CT_Scene(), tr("Largest_Clusters"));
    }

}

// Semi-automatic creation of step parameters DialogBox
void ST_StepExtractLargestCluster::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();

    configDialog->addText("A simple but fast spatial clustering operation is applied. The output cloud will contain the <b>numCluster</b> largest clusters containing at least <b>minPts</b> points. If only a smaller number of clusters",
                          "     exists, <b>numCluster</b> is set automatically to this number. Points are separated into two clusters, if the distance between the closest point pair exceeds <b>clusterTolerance</b>.");
    configDialog->addEmpty();
    configDialog->addDouble( tr("clustering distance "), "clusterTolerance m",  0.01, 0.2,2,  _distance, 1, "the clustering distance.");
    configDialog->addInt( "The number of clusters  ","numCluster",1,10000,_number_clusters);
    configDialog->addInt("The number of points a cluster needs to contain ","minPts",1,10000,_minPts);
    configDialog->addDouble("The number of points a cluster needs to contain in percentage of total size","min percentage",0 , 100,1,_min_perc);
    dialog_simple_tree(configDialog);
}

void ST_StepExtractLargestCluster::compute()
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
        if(itemCpy_cloud_in!=NULL)
        {
            PointCloudS::Ptr _cloud_in = pcl_CT_to_PCL_cloud(itemCpy_cloud_in,this,16,false,false);
            if(_cloud_in->points.size()!=0)
            {
                warning_geo_referenced(_cloud_in);
                PointCloudS::Ptr _cloud_in_downscaled = pcl_voxel_grid_filter( _cloud_in,this, _distance/4,false);
                setProgress(5);
                pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS>);
                tree->setInputCloud (_cloud_in_downscaled);
                std::vector<pcl::PointIndices> _cluster_indices;
                pcl::EuclideanClusterExtraction<PointS> ec;
                ec.setClusterTolerance (_distance); // 2cm
                size_t size = 1;
                if(itemCpy_cloud_in->getPointCloudIndexSize()!=0)
                    size = itemCpy_cloud_in->getPointCloudIndexSize();
                float minsize =  std::max<float>( _minPts*_cloud_in_downscaled->points.size()/size,_min_perc*0.01*_cloud_in_downscaled->points.size());
                ec.setMinClusterSize (minsize);
                ec.setSearchMethod (tree);
                ec.setInputCloud (_cloud_in_downscaled);
                ec.extract (_cluster_indices);
                for(size_t i = 0; i < _cloud_in_downscaled->points.size(); i++)
                {
                    _cloud_in_downscaled->points[i].treeID = 0;
                }
                CT_PointCloudIndexVector *extracted_cluster = new CT_PointCloudIndexVector();
                CT_PointCloudIndexVector *extracted_rest = new CT_PointCloudIndexVector();

                if(_cluster_indices.size()>0)
                {
                    int k = 0;
                    while(  (k < _number_clusters)  && ( k < _cluster_indices.size()) )
                    {
                        pcl::PointIndices c_ind = _cluster_indices.at(k);
                        for(size_t i = 0; i < c_ind.indices.size(); i++)
                        {
                            int index = c_ind.indices.at(i);
                            _cloud_in_downscaled->points[index].treeID = 1;
                        }
                        k++;
                    }
                    {
                        const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in->getPointCloudIndex();
                        CT_PointIterator itP(pointCloudIndex);
                        pcl::KdTreeFLANN<PointS> kdtree;
                        kdtree.setInputCloud (_cloud_in_downscaled);
                        setProgress(30);
                        while (itP.hasNext() && !isStopped())
                        {
                            itP.next();
                            const CT_Point &point = itP.currentPoint();
                            size_t index = itP.currentGlobalIndex();
                            PointS query(point[0],point[1],point[2]);

                            std::vector<int> pointIdxNKNSearch(1);
                            std::vector<float> pointNKNSquaredDistance(1);
                            kdtree.nearestKSearch (query, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
                            PointS nn = _cloud_in_downscaled->points[pointIdxNKNSearch.at(0)];

                            int is_valid = _cloud_in_downscaled->points[pointIdxNKNSearch.at(0)].treeID;
                            if(is_valid == 0)
                            {
                                extracted_rest->addIndex(index);
                            }
                            else
                            {
                                extracted_cluster->addIndex(index);
                            }
                        }
                    }
                    setProgress(45);
                }
                CT_Scene* outScene_cluster
                        = new CT_Scene(_outScene_cluster.completeName(), resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extracted_cluster)); // 3) create scene, registering the pointcloudindex
                outScene_cluster->updateBoundingBox();
                grpCpy_grp->addItemDrawable(outScene_cluster);

                CT_Scene* outScene_rest
                        = new CT_Scene(_outScene_rest.completeName(), resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extracted_rest)); // 3) create scene, registering the pointcloudindex
                outScene_rest->updateBoundingBox();
                grpCpy_grp->addItemDrawable(outScene_rest);
            } else {
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















