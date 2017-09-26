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

#include "st_stepfiltergroundpoints.h"

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
ST_StepFilterGroundPoints::ST_StepFilterGroundPoints(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
}

ST_StepFilterGroundPoints::~ST_StepFilterGroundPoints()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepFilterGroundPoints::getStepDescription() const
{
    return tr("Removes noise from detected ground points.");
}

// Step detailled description
QString ST_StepFilterGroundPoints::getStepDetailledDescription() const
{
    return tr("Removes noise from detected ground points." );
}

// Step URL
QString ST_StepFilterGroundPoints::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    // return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepFilterGroundPoints::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepFilterGroundPoints(dataInit);
}


// Creation and affiliation of IN models
void ST_StepFilterGroundPoints::createInResultModelListProtected()
{



    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("Result_In"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("Grp_In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);

    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Ground cloud"));
}

// Creation and affiliation of OUT models
void ST_StepFilterGroundPoints::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {

        res->addItemModel(DEFin_grp, _ground_noise, new CT_Scene(), tr("Noise within ground points"));
        res->addItemModel(DEFin_grp, _ground_filtered, new CT_Scene(), tr("Denoised ground points"));
    }
}

// Semi-automatic creation of step parameters DialogBox
void ST_StepFilterGroundPoints::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addTitle(tr("Select an angle for the maximum point orientation deviation for a ground point:"));
    // configDialog->addInt( "The number of clusters  ","numCluster",1,10000,_number_clusters);
    configDialog->addInt("angle ","(in Degree)",0,90,_angle);
      dialog_simple_tree(configDialog);

}

void ST_StepFilterGroundPoints::compute()
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
            setProgress(1);
            _cloud_in = pcl_CT_to_PCL_cloud(itemCpy_cloud_in,this,16,false,true);
            if(_cloud_in->points.size()!=0)
            {
                warning_geo_referenced(_cloud_in);
               setProgress(4);
                _cloud_in_downscaled_for_ground = pcl_voxel_grid_filter(_cloud_in,this,0.25f,true);
                setProgress(10);
                _cloud_in_downscaled = pcl_voxel_grid_filter(_cloud_in,this, 0.025f,true);
                setProgress(16);
                _ground_normal =  compute_ground_normal(_cloud_in_downscaled_for_ground);
                setProgress(20);

               {
                   QString str;
                   str.append("The normal orientation of the ground points is (");
                   QString a = tofQString(_ground_normal[0],2);
                   QString b = tofQString(_ground_normal[1],2);
                   QString c = tofQString(_ground_normal[2],2);
                   str.append(a).append(";").append(b).append(";").append(c).append(").");
                   PS_LOG->addInfoMessage(this, str);
               }
               pcl_compute_normals(_cloud_in_downscaled, 90);
               setProgress(40);
               pcl_transfer_attribute(_cloud_in_downscaled,_cloud_in, PointAtrributeType::NORMAL);
               setProgress(50);
               QVector<float> sin_vec;

               for(size_t i = 0; i < _cloud_in->points.size(); i++)
               {
                   PointS p = _cloud_in->points.at(i);
                   Eigen::Vector3f point_normal;
                   point_normal[0] = p.normal_x;
                   point_normal[1] = p.normal_y;
                   point_normal[2] = p.normal_z;
                   float angle_in_rad = SimpleMath<float>::angle_between_in_rad(point_normal,_ground_normal);
                   float sine = std::sin(angle_in_rad);
                   sin_vec.push_back(sine);
               }
               setProgress(60);


               CT_PointCloudIndexVector *extractedCloud1 = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
               CT_PointCloudIndexVector *extractedCloud2 = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index


               const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in->getPointCloudIndex();
               CT_PointIterator itP(pointCloudIndex);
               int ind = 0;
               double pi = std::acos(-1);
               double threshold = std::sin((_angle*pi)/180);

               while (itP.hasNext() && !isStopped())
               {
                   itP.next();
                   size_t index = itP.currentGlobalIndex();
                   float sine = sin_vec.at(ind++);
                   if(sine>threshold)
                   {
                       extractedCloud1->addIndex(index);
                   } else {
                       extractedCloud2->addIndex(index);
                   }
               }
               {
                   CT_Scene* outScene = new CT_Scene(_ground_noise.completeName(),
                                                     resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud1));
                   outScene->updateBoundingBox();
                   grpCpy_grp->addItemDrawable(outScene);
               }

               {
                   CT_Scene* outScene = new CT_Scene(_ground_filtered.completeName(),
                                                     resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud2)); // 3) create scene, registering the pointcloudindex
                   outScene->updateBoundingBox(); // 4) don't forget to update the bounding box, to be fitted to filtered points

                   grpCpy_grp->addItemDrawable(outScene);
               }

               setProgress(99);
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
    _cloud_in.reset(new PointCloudS);
    _cloud_in_downscaled.reset(new PointCloudS);
    _cloud_in_downscaled_for_ground.reset(new PointCloudS);
}

Eigen::Vector3f ST_StepFilterGroundPoints::compute_ground_normal(PointCloudS::Ptr ground_points)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<PointS> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.5);

    seg.setInputCloud (ground_points);
    seg.segment (*inliers, *coefficients);
    Eigen::Vector3f normal;
    normal[0]  = coefficients->values[0];
    normal[1]  = coefficients->values[1];
    normal[2]  = coefficients->values[2];
    return normal;

}















