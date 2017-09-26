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
#include "st_stepsegmentationall.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_grp_out "grp_out"
#define DEFin_cloud_out "cloud_out"
#define DEFin_cloud_in "cloud_in"
#define DEFin_grp_in "grp_cluster_in"
#define DEFin_cluster_in "cluster_in"
#define DEFin_tree_in "tree_in"

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
ST_StepSegmentationAll::ST_StepSegmentationAll(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _speed = 4;
    _range = 0.05;
    _low = false;
}

ST_StepSegmentationAll::~ST_StepSegmentationAll()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepSegmentationAll::getStepDescription() const
{
    return tr("Segmentation into tree clouds");
}

// Step detailled description
QString ST_StepSegmentationAll::getStepDetailledDescription() const
{
    return tr("Segments the cloud. The cloud should be downscaled to improve runtime performance. The ground has to be removed before. The step needs segmentation seeds aka detected tree roots."
              " You can produce those with euclidean clustering.");
}

// Step URL
QString ST_StepSegmentationAll::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    // return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepSegmentationAll::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepSegmentationAll(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepSegmentationAll::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Vegetation"));
    resIn_res->addGroupModel(DEFin_grp, DEFin_grp_in, CT_AbstractItemGroup::staticGetType(), tr("Vegetation seeds grp"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resIn_res->addItemModel(DEFin_grp_in, DEFin_cluster_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Vegetation seeds"));
}

// Creation and affiliation of OUT models
void ST_StepSegmentationAll::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);
    if(res != NULL)
    {
        res->addGroupModel(DEFin_grp, _grp_out, new CT_StandardItemGroup(),tr("Isolated Tree Grp"));
        res->addItemModel(_grp_out, _cloud_out_segmented, new CT_Scene(),tr("Isolated Tree"));
    }
}

// Semi-automatic creation of step parameters DialogBox
void ST_StepSegmentationAll::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addInt(tr("Speed of computation"),"",0,5,_speed,"effects accuracy - 5 is most accurate and slowest, 0 fast and less accurate, 1 or 2 recommended to start");
    configDialog->addDouble(tr("range"),"m",0.01,0.2,2,_range,1,"a range parameter for preliminary clustering, 0.05 cm is recommended, increase for low quality clouds");
    configDialog->addBool(tr("low quality"), "","",_low,"check only if you have a low density cloud () Might cost a lot of computation time for high resoluted clouds.");

    configDialog->addBool(tr("another slow switch"), "","",_slow,"Check in every case if you have time. My test depict 2 minute vs 3 hours runtime for NFI plots. Should be more accurate though.");
    dialog_simple_tree(configDialog);
}

void ST_StepSegmentationAll::compute()
{
    QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    while (itCpy_grp.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        setProgress(1);
        int counter = 0;
        {
            CT_GroupIterator itGroupToCount(grpCpy_grp, this, DEFin_grp_in);
            while (itGroupToCount.hasNext() && (!isStopped()))
            {
                itGroupToCount.next();
                counter++;
            }
        }
        setProgress(5);
        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in
                = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        PointCloudS::Ptr cloud = pcl_CT_to_PCL_cloud(itemCpy_cloud_in,this,16,false,false);
        warning_geo_referenced(cloud);
        setProgress(10);
        PointCloudS::Ptr cloud_downscaled = pcl_voxel_grid_filter(cloud,this,(_range*3.0/5.0),false);
        setProgress(20);
        PointCloudS::Ptr cloud_downscaled_largest_clusters (new PointCloudS);
        {
            pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS>);
            tree->setInputCloud (cloud_downscaled);
            std::vector<pcl::PointIndices> _cluster_indices;
            pcl::EuclideanClusterExtraction<PointS> ec;
            ec.setClusterTolerance (_range);
            if(!_low)
                ec.setMinClusterSize (_minsize*cloud_downscaled->points.size()/cloud->points.size());
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud_downscaled);
            ec.extract (_cluster_indices);
            int _number_clusters = counter*std::pow(2, _speed);
            {
                int k = 0;
                while( (k < _number_clusters) && ( k < _cluster_indices.size()) )
                {
                    pcl::PointIndices c_ind = _cluster_indices.at(k);
                    for(size_t i = 0; i < c_ind.indices.size(); i++)
                    {
                        int index = c_ind.indices.at(i);
                        cloud_downscaled_largest_clusters->points.push_back(cloud_downscaled->points[index]);
                    }
                    k++;
                }
            }
        }
        setProgress(45);

        PointCloudS::Ptr cloud_height_scaled = pcl_scale_along_z_axis(cloud);
        setProgress(55);
        PointCloudS::Ptr cloud_downscaled_largest_clusters_height_scaled = pcl_scale_along_z_axis(cloud_downscaled_largest_clusters);
        setProgress(60);
        QVector<PointCloudS::Ptr> seeds_height_scaled;
        CT_GroupIterator itGroupToCount(grpCpy_grp, this, DEFin_grp_in);
        int seedsize = 0;
        while (itGroupToCount.hasNext() && (!isStopped()))
        {
            CT_StandardItemGroup* grpCpy_grp2 = (CT_StandardItemGroup*) itGroupToCount.next();
            CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in2
                    = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp2->firstItemByINModelName(this, DEFin_cluster_in);
            PointCloudS::Ptr cloud2 = pcl_CT_to_PCL_cloud(itemCpy_cloud_in2,this,16,false,false);

            PointCloudS::Ptr cloud_downscaled2 = pcl_voxel_grid_filter(cloud2,this,(_range*3.0/5.0),false);
            PointCloudS::Ptr cloud2_height_scaled2 = pcl_scale_along_z_axis(cloud_downscaled2);
            seedsize += cloud2_height_scaled2->points.size();
            seeds_height_scaled.push_back(cloud2_height_scaled2);
        }

        setProgress(65);
        DijkstraCoefficients coeff;
        coeff.search_range = _range;
        if(_slow)
        {
            Dijkstra(cloud_downscaled_largest_clusters_height_scaled,seeds_height_scaled, coeff);
        } else
        {
            DijkstraFast(cloud_downscaled_largest_clusters_height_scaled,seeds_height_scaled, coeff);
        }

        setProgress(80);
        {
            pcl::KdTreeFLANN<PointS>::Ptr kdtree(new pcl::KdTreeFLANN<PointS>);
            kdtree->setInputCloud (cloud_downscaled_largest_clusters_height_scaled);
            for(size_t i = 0; i < cloud_height_scaled->points.size(); i++)
            {
                PointS target = cloud_height_scaled->points.at(i);
                std::vector<int> pointIdxNKNSearch(1);
                std::vector<float> pointNKNSquaredDistance(1);
                kdtree->nearestKSearch (target, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
                cloud_height_scaled->points[i].treeID = cloud_downscaled_largest_clusters_height_scaled->points[pointIdxNKNSearch.at(0)].treeID;
            }
        }
        int max  = -1;
        for(size_t i = 0; i < cloud_height_scaled->points.size(); i++)
        {
            PointS p =  cloud_height_scaled->points.at(i);
            if(p.treeID > max)
                max = p.treeID;
        }
        QVector<QVector<int> > indices_int; //added and the llop
        for(int i = -1; i < max; i++)
        {
            QVector<int> indices;
            indices_int.push_back(indices);
        }

        QVector<CT_PointCloudIndexVector *> indices_vec;
        for(int i = -1; i < max; i++)
        {
            CT_PointCloudIndexVector *tree = new CT_PointCloudIndexVector();
            indices_vec.push_back(tree);
        }
        int i = 0;
        const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in->getPointCloudIndex();
        CT_PointIterator itP(pointCloudIndex);
        while (itP.hasNext() && !isStopped())
        {
            int id = cloud_height_scaled->points.at(i).treeID;
            itP.next();
            size_t index = itP.currentGlobalIndex();
            if(id>=0 && id < max + 1 )
            {
                indices_int[id].push_back(index);
//                indices_vec[id]->addIndex(index);
            }
            i++;
        }



        for(int i = 0; i < max + 1; i++)
        {
            QVector<int> indices = indices_int.at(i);
            qStableSort(indices.begin(), indices.end());
            for(int j = 0; j < indices.size(); j++)
            {
                size_t inde = indices.at(j);
                 indices_vec[i]->addIndex(inde);
            }
        }
        setProgress(95);
qDebug()  << "asd" << indices_vec.size();
        for(int i = 0; i < max + 1; i++)
        {
            if(indices_vec[i]->size()>coeff.TREE_CLUSTER_MIN_SIZE)
            {
                CT_StandardItemGroup* cloud_grp = new CT_StandardItemGroup(_grp_out.completeName(), resCpy_res);
                grpCpy_grp->addGroup(cloud_grp);
                CT_Scene* outScene_cluster
                        = new CT_Scene(_cloud_out_segmented.completeName(), resCpy_res, PS_REPOSITORY->registerPointCloudIndex(indices_vec[i])); // 3) create scene, registering the pointcloudindex
                outScene_cluster->updateBoundingBox(); // 4) don't forget to update the bounding box, to be fitted to filtered points
                cloud_grp->addItemDrawable(outScene_cluster);
            }
        }
        setProgress(99);

    }
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
}

PointCloudS::Ptr  ST_StepSegmentationAll::pcl_scale_along_z_axis(PointCloudS::Ptr cloud_in,float factor, bool inverse)
{
    PointCloudS::Ptr cloud_out (new PointCloudS);

    for(size_t i = 0; i < cloud_in->points.size(); i++)
    {
        PointS p = cloud_in->points.at(i);
        if(inverse)
        {
            PointS n (p.x,p.y,p.z*factor);
            n.ID = p.ID;
            cloud_out->points.push_back(n);
        } else {
            PointS n (p.x,p.y,p.z/factor);
            n.ID = p.ID;
            cloud_out->points.push_back(n);
        }

    }
    return cloud_out;
}










