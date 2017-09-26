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

#include "st_stepeuclideanclustering.h"

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


//// Alias for indexing out models
#define DEF_resultOut_translated "extractedResult"
#define DEF_groupOut_pointCloud "extractedGroup"
#define DEF_itemOut_scene "extractedScene"



// Constructor : initialization of parameters
ST_StepEuclideanClustering::ST_StepEuclideanClustering(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
}

ST_StepEuclideanClustering::~ST_StepEuclideanClustering()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepEuclideanClustering::getStepDescription() const
{
    return tr("An euclidean clustering operation. Produces sub clouds of the clusters.");
}

// Step detailled description
QString ST_StepEuclideanClustering::getStepDetailledDescription() const
{
    return tr("An euclidean clustering operation. Produces sub clouds of the clusters." );
}

// Step URL
QString ST_StepEuclideanClustering::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
}

// Step copy method
CT_VirtualAbstractStep* ST_StepEuclideanClustering::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepEuclideanClustering(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepEuclideanClustering::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("Result_In"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("Grp_In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);

    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Tree_Cloud"));

}

// Creation and affiliation of OUT models
void ST_StepEuclideanClustering::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {
        res->addGroupModel(DEFin_grp, _outScene_grp, new CT_StandardItemGroup(), tr ("Grp out") );
        res->addItemModel(_outScene_grp, _outScene_cluster, new CT_Scene(), tr("Clusters"));
    }

}

// Semi-automatic creation of step parameters DialogBox
void ST_StepEuclideanClustering::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();

    configDialog->addEmpty();
    configDialog->addText("A simple but fast spatial clustering operation is applied. The output cloud will contain the <b>numCluster</b> largest clusters containing at least <b>minPts</b> points. If only a smaller number of clusters",
                          "     exists, <b>numCluster</b> is set automatically to this number. Points are separated into two clusters, if the distance between the closest point pair exceeds <b>clusterTolerance</b>.");
    configDialog->addEmpty();
    configDialog->addDouble( tr("clustering distance "), "clusterTolerance m",  0.01, 0.2,2,  _distance, 1, "the clustering distance.");
    // configDialog->addInt( "The number of clusters  ","numCluster",1,10000,_number_clusters);
    configDialog->addInt("The number of points a cluster needs to contain ","minPts",1,10000,_minPts);
    dialog_simple_tree(configDialog);
}

void ST_StepEuclideanClustering::compute()
{
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    while (itCpy_grp.hasNext() && !isStopped())
    {

        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in
                = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        if(itemCpy_cloud_in!=NULL)
        {
            if(itemCpy_cloud_in->getPointCloudIndex()->size() == 0)
            {
                groupsToBeRemoved.append(grpCpy_grp);
            } else
            {

                PointCloudS::Ptr cloud = pcl_CT_to_PCL_cloud(itemCpy_cloud_in,this, 9,false,false);
                PointCloudS::Ptr cloud_downscaled = pcl_voxel_grid_filter(cloud,this,_distance/3,false);

                qDebug() << cloud_downscaled->points.size();
                warning_geo_referenced(cloud_downscaled);

                pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS>);
                tree->setInputCloud (cloud_downscaled);

                std::vector<pcl::PointIndices> _cluster_indices;
                pcl::EuclideanClusterExtraction<PointS> ec;
                ec.setClusterTolerance (_distance);
                ec.setSearchMethod (tree);
                ec.setInputCloud (cloud_downscaled);
                ec.extract (_cluster_indices);


                QVector<CT_PointCloudIndexVector *> index_vectors;
                for(size_t i = 0; i <  _cluster_indices.size(); i++)
                {
                    pcl::PointIndices pi = _cluster_indices.at(i);
                    for(size_t j = 0; j < pi.indices.size(); j++)
                    {
                        int index = pi.indices.at(j);
                        cloud_downscaled->points[index].ID = i;
                    }
                    CT_PointCloudIndexVector *extracted_cluster = new CT_PointCloudIndexVector();
                    index_vectors.push_back(extracted_cluster);
                }


                const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in->getPointCloudIndex();
                CT_PointIterator itP(pointCloudIndex);

                while (itP.hasNext() && !isStopped())
                {
                    itP.next();
                    const CT_Point &point = itP.currentPoint();
                    size_t index = itP.currentGlobalIndex();
                    PointS query(point[0],point[1],point[2]);
                    std::vector<int> pointIdxNKNSearch(1);
                    std::vector<float> pointNKNSquaredDistance(1);
                    tree->nearestKSearch (query, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
                    PointS nn = cloud_downscaled->points[pointIdxNKNSearch.at(0)];
                    index_vectors[nn.ID]->addIndex(index);
                }

                for(size_t i = 0; i <  index_vectors.size(); i++)
                {
                    CT_PointCloudIndexVector *extracted_cluster = index_vectors.at(i);
                    if(extracted_cluster->size()>=_minPts)
                    {
                        CT_StandardItemGroup* cloud_grp = new CT_StandardItemGroup(_outScene_grp.completeName(), resCpy_res);
                        grpCpy_grp->addGroup(cloud_grp);
                        CT_Scene* outScene_cluster
                                = new CT_Scene(_outScene_cluster.completeName(), resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extracted_cluster));
                        outScene_cluster->updateBoundingBox();
                        cloud_grp->addItemDrawable(outScene_cluster);
                    }
                    else
                    {
                        delete extracted_cluster;
                    }
                }
                index_vectors.clear();

            }


        } else {
            groupsToBeRemoved.append(grpCpy_grp);
        }

    }
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }

}

//void ST_StepEuclideanClustering::recursiveRemoveGroupIfEmpty(CT_AbstractItemGroup *parent, CT_AbstractItemGroup *group) const
//{
//    if(parent != NULL)
//    {
//        parent->removeGroup(group);

//        if(parent->isEmpty())
//            recursiveRemoveGroupIfEmpty(parent->parentGroup(), parent);
//    }
//    else
//    {
//        ((CT_ResultGroup*)group->result())->removeGroupSomethingInStructure(group);
//    }
//}


















