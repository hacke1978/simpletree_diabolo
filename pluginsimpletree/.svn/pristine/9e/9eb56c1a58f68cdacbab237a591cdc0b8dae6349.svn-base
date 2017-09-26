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

#include "st_stepfilterstempoints.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_grp2 "grp2"
#define DEFin_cloud_in "ground cloud"
#define DEFin_cloud_in2 "slice cloud"

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
ST_StepFilterStems::ST_StepFilterStems(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
}

ST_StepFilterStems::~ST_StepFilterStems()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepFilterStems::getStepDescription() const
{
    return tr("Removes noise from the stem  slice.");
}

// Step detailled description
QString ST_StepFilterStems::getStepDetailledDescription() const
{
    return tr("Removes noise from the stem  slice." );
}

// Step URL
QString ST_StepFilterStems::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    // return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepFilterStems::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepFilterStems(dataInit);
}


// Creation and affiliation of IN models
void ST_StepFilterStems::createInResultModelListProtected()
{



    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("Result_In"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp2, CT_AbstractItemGroup::staticGetType(), tr("Grp stem"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resIn_res->addItemModel(DEFin_grp2, DEFin_cloud_in2, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Stem slice cloud"));


   // resIn_res->addItemModel(DEFin_grp2, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Ground cloud"));
}

// Creation and affiliation of OUT models
void ST_StepFilterStems::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {
        res->addItemModel(DEFin_grp2, _stem_noise, new CT_Scene(), tr("Noise in  Stem slice "));
        res->addItemModel(DEFin_grp2, _stem_denoise, new CT_Scene(), tr("Denoised Stem slice "));
    }
}

// Semi-automatic creation of step parameters DialogBox
void ST_StepFilterStems::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addTitle(tr("Select an angle for the maximum deviation of the dominant direction to the z axis:"));
    //configDialog->addBool("compute ground" , "" , "compute",_compute_ground);
    configDialog->addInt("angle ","(in Degree)",0,90,_angle);
    dialog_simple_tree(configDialog);

}

void ST_StepFilterStems::compute()
{
    QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res_stem = outResultList.at(0);
            _ground_normal[0] = 0;
            _ground_normal[1] = 0;
            _ground_normal[2] = 1;
//    CT_ResultGroupIterator itCpy_grp_ground(resCpy_res_stem, this, DEFin_grp2);
//    while (itCpy_grp_ground.hasNext() && !isStopped())
//    {

//        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp_ground.next();
//        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in
//                = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);

////        if(_compute_ground)
////        {
////            setProgress(1);
////            _cloud_in = pcl_CT_to_PCL_cloud(itemCpy_cloud_in,this,16,false,true);
////            warning_geo_referenced(_cloud_in);
////            setProgress(4);
////            _cloud_in_downscaled_for_ground = pcl_voxel_grid_filter(_cloud_in,this,0.25f,true);
////            setProgress(10);
////            _cloud_in_downscaled = pcl_voxel_grid_filter(_cloud_in,this, 0.025f,true);
////            setProgress(16);
////            _ground_normal =  compute_ground_normal(_cloud_in_downscaled_for_ground);
////        } else {
////            _ground_normal[0] = 0;
////            _ground_normal[1] = 0;
////            _ground_normal[2] = 1;
////        }

//        _ground_normal[0] = 0;
//        _ground_normal[1] = 0;
//        _ground_normal[2] = 1;
//        setProgress(20);

//        {
//            QString str;
//            str.append("The normal orientation of the ground points is (");
//            QString a = tofQString(_ground_normal[0],2);
//            QString b = tofQString(_ground_normal[1],2);
//            QString c = tofQString(_ground_normal[2],2);
//            str.append(a).append(";").append(b).append(";").append(c).append(").");
//            PS_LOG->addInfoMessage(this, str);
//        }
//    }

    CT_ResultGroupIterator itCpy_grp_stem(resCpy_res_stem, this, DEFin_grp2);
    while (itCpy_grp_stem.hasNext() && !isStopped())
    {

        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp_stem.next();
        PointCloudS::Ptr _cloud_in2 (new PointCloudS);
        PointCloudS::Ptr _cloud_in_downscaled2 (new PointCloudS);

        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in2
                = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in2);
        if(itemCpy_cloud_in2!=NULL)
        {
            setProgress(25);
            _cloud_in2 = pcl_CT_to_PCL_cloud(itemCpy_cloud_in2,this,16,false,true);
            if(_cloud_in2->points.size()!=0)
            {
                setProgress(45);
                _cloud_in_downscaled2 = pcl_voxel_grid_filter(_cloud_in2,this, 0.025f,true);

                setProgress(60);

                pcl::NormalEstimation<PointS, PointS> ne;
                ne.setInputCloud (_cloud_in_downscaled2);

                // Create an empty kdtree representation, and pass it to the normal estimation object.
                // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
                pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS> ());
                ne.setSearchMethod (tree);

                // Output datasets
                pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

                // Use all neighbors in a sphere of radius 3cm
                ne.setRadiusSearch (0.06f);

                // Compute the features
                ne.compute (*_cloud_in_downscaled2);


                setProgress(70);
                pcl_transfer_attribute(_cloud_in_downscaled2,_cloud_in2, PointAtrributeType::NORMAL);
                setProgress(80);
                QVector<float> cos_vec;

                double pi = std::acos(-1);
                for(size_t i = 0; i < _cloud_in2->points.size(); i++)
                {
                    PointS p = _cloud_in2->points.at(i);
                    Eigen::Vector3f point_normal;
                    point_normal[0] = p.normal_x;
                    point_normal[1] = p.normal_y;
                    point_normal[2] = p.normal_z;
                    float angle_in_rad = SimpleMath<float>::angle_between_in_rad(point_normal,_ground_normal);
                    cos_vec.push_back(angle_in_rad);
                }
                setProgress(85);


                CT_PointCloudIndexVector *extractedCloud_denoise = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
                CT_PointCloudIndexVector *extractedCloud_noise = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index

                setProgress(86);
                const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in2->getPointCloudIndex();
                CT_PointIterator itP(pointCloudIndex);
                int ind = 0;
                double threshold1 = ((_angle*pi)/180);
                double threshold2 = (((180-_angle)*pi)/180);
                setProgress(87);
                while (itP.hasNext() && !isStopped())
                {
                    itP.next();
                    size_t index = itP.currentGlobalIndex();
                    float cos = cos_vec.at(ind++);
                    if(cos < threshold1 || cos > threshold2)
                    {
                        extractedCloud_noise->addIndex(index);
                    } else {
                        extractedCloud_denoise->addIndex(index);
                    }
                }
                setProgress(94);
                {
                    CT_Scene* outScene = new CT_Scene(_stem_noise.completeName(),
                                                      resCpy_res_stem, PS_REPOSITORY->registerPointCloudIndex(extractedCloud_noise));
                    outScene->updateBoundingBox();

                    grpCpy_grp->addItemDrawable(outScene);
                }
                {
                    CT_Scene* outScene = new CT_Scene(_stem_denoise.completeName(),
                                                      resCpy_res_stem, PS_REPOSITORY->registerPointCloudIndex(extractedCloud_denoise));
                    outScene->updateBoundingBox();
                    grpCpy_grp->addItemDrawable(outScene);
                }
                setProgress(99);
            }else {
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

Eigen::Vector3f ST_StepFilterStems::compute_ground_normal(PointCloudS::Ptr ground_points)
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















