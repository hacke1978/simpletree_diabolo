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
#include "st_stepfitsinglecylinder.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"


ST_StepFitSingleCylinder::ST_StepFitSingleCylinder(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{


     _nmb_inliers = 100;
     _iterations = 1000;
     _max_dist = 0.1;
}

ST_StepFitSingleCylinder::~ST_StepFitSingleCylinder()
{
}

// Step description (tooltip of contextual menu)
QString ST_StepFitSingleCylinder::getStepDescription() const
{
    return tr("Detects the Stem points.");
}

// Step detailled description
QString ST_StepFitSingleCylinder::getStepDetailledDescription() const
{
    return tr("The stem points are detected." );
}

// Step URL
QString ST_StepFitSingleCylinder::getStepURL() const
{
    //return tr("http://www.simpletree.uni-freiburg.de/news.html");
    return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepFitSingleCylinder::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepFitSingleCylinder(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepFitSingleCylinder::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("Result_In"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("Grp_In"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Tree_Cloud"));



}

// Creation and affiliation of OUT models
void ST_StepFitSingleCylinder::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addGroupModel(DEFin_grp, _outCylinderGroupModelName, new CT_StandardItemGroup(), tr("Cylinder group"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName, new CT_Cylinder(), tr("Fitted Cylinder"));
    }
}

// Semi-automatic creation of step parameters DialogBox
void ST_StepFitSingleCylinder::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addTitle(tr("This method fits a cylinder by MLESAC fitting routine."));

    configDialog->addDouble( tr("distance to cylinder"), "",  0, 1,3,  _max_dist, 1, "distance to cylinder.");
    //    configDialog->addInt(tr("number of inliers" ),"",1,1000, _nmb_inliers, "The number of inliers");
    configDialog->addInt(tr("number of _iterations" ),"",1,1000, _iterations, "The number of _iterations");
}

void ST_StepFitSingleCylinder::compute()
{
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);

    while (itCpy_grp.hasNext() && !isStopped())
    {
//        qDebug() << "01";
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        CT_Scene* itemIn_scene = (CT_Scene*) grpCpy_grp;
        const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in =
                (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        create_simple_tree_cloud(itemCpy_cloud_in);
        enrich_cloud(itemCpy_cloud_in,resCpy_res,grpCpy_grp);
//        qDebug() <<_cloud_in->points.size();
        pcl::PointIndices::Ptr  inliers_cylinder (new pcl::PointIndices);
        pcl::ModelCoefficients coefficients_cylinder;
        pcl::SACSegmentationFromNormals<PointS, PointS> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_CYLINDER);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (_iterations);
        seg.setDistanceThreshold (_max_dist);
        seg.setInputCloud (_cloud_in);
        seg.setInputNormals (_cloud_in);
        seg.setDistanceThreshold (0.05);
        seg.setRadiusLimits (0, 2);
        seg.segment (*inliers_cylinder, coefficients_cylinder);
//        qDebug() << "02";
        PointS s;
        PointS e;
        float min = std::numeric_limits<float>::max();
        float max = std::numeric_limits<float>::lowest();
        for(size_t i = 0; i < _cloud_in->points.size(); i++)
        {
            PointS p = _cloud_in->points.at(i);
            if(p.z < min)
            {
                s = p;
                min = p.z;
            }
            if(p.z > max)
            {
                e = p;
                max = p.z;
            }
        }
//        qDebug() << s.x;
//        qDebug() << e.x;
//        qDebug() << coefficients_cylinder.values.size();
//        qDebug() << coefficients_cylinder.values[6];

//        qDebug() << "03";
        QSharedPointer<Cylinder> ransac (new Cylinder(coefficients_cylinder));

        QSharedPointer<PointS> start (new PointS(s.x,s.y,s.z));
        QSharedPointer<PointS> end   (new PointS(e.x,e.y,e.z));

        QSharedPointer<PointS> start2 =  ransac->projection_on_axis_ptr(start);
        QSharedPointer<PointS> end2 =  ransac->projection_on_axis_ptr(end);

        ransac->set_start_end(start2,end2);

//        qDebug() << "04";

        CT_StandardItemGroup* cylinderGroup = new CT_StandardItemGroup(_outCylinderGroupModelName.completeName(), resCpy_res);
        grpCpy_grp->addGroup(cylinderGroup);


        QSharedPointer<PointS> center = ransac->get_center_ptr();
        QSharedPointer<PointS> start3 = ransac->get_start_ptr();
        QSharedPointer<PointS> stop = ransac->get_end_ptr();


        CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                    Eigen::Vector3d(stop->x-start3->x, stop->y-start3->y, stop->z-start3->z),
                                                    ransac->get_radius(),
                                                    ransac->get_length());



        CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName.completeName(), resCpy_res, data);
        cylinderGroup->addItemDrawable(cylinder);
//        qDebug() << "05";
    }
}

void ST_StepFitSingleCylinder::enrich_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp)
{
    const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();
    size_t size = index->size();
    CT_NormalCloudStdVector *normalCloud = new CT_NormalCloudStdVector( size );
    CT_StandardCloudStdVectorT<int> *stemCloud = new CT_StandardCloudStdVectorT<int>(size);

    for(size_t i =0; i < size; i ++)
    {
        PointS p = _cloud_in->points[i];
        float n1 = p.normal_x;
        float n2 = p.normal_y;
        float n3 = p.normal_z;
        float stem = p.is_stem;
        CT_Normal &ctNormal = normalCloud->normalAt(i);
        ctNormal.x() = n1;
        ctNormal.y() = n2;
        ctNormal.z() = n3;
        ctNormal.w() = p.curvature;
        stemCloud->tAt(i) = stem;
    }


}

void ST_StepFitSingleCylinder::create_simple_tree_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in)
{
    if (itemCpy_cloud_in != NULL)
    {
        const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();

        size_t size = 0;
        size = index->size();
        _cloud_in.reset(new PointCloudS);
        _cloud_in->width = size;
        _cloud_in->height = 1;
        if(size > 0) {
            _cloud_in->points.resize(size);
            size_t i = 0;
            CT_PointIterator it (index);
            while(it.hasNext())
            {
                const CT_PointData &internalPoint = it.next().currentConstInternalPoint();
                PointS p (internalPoint[0],internalPoint[1],internalPoint[2]);
                _cloud_in->points[i] = p;
                ++i;
            }
        }
        EnrichCloud enrich(_cloud_in, 45, 0, true);

    }
}



