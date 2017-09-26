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
#include "st_stepfitmultipleDBH.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"


ST_StepFITMultipleDBH::ST_StepFITMultipleDBH(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _nmb_inliers = 100;
    _iterations = 1000;
    _max_dist = 0.1;
}

ST_StepFITMultipleDBH::~ST_StepFITMultipleDBH()
{
}

// Step description (tooltip of contextual menu)
QString ST_StepFITMultipleDBH::getStepDescription() const
{
    return tr("Fits cylinders into multiple slices.");
}

// Step detailled description
QString ST_StepFITMultipleDBH::getStepDetailledDescription() const
{
    return tr("The stem points are detected." );
}

// Step URL
QString ST_StepFITMultipleDBH::getStepURL() const
{
    //return tr("http://www.simpletree.uni-freiburg.de/news.html");
    return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepFITMultipleDBH::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepFITMultipleDBH(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepFITMultipleDBH::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("Result_In"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("Grp_In"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Tree_Cloud"));



}

// Creation and affiliation of OUT models
void ST_StepFITMultipleDBH::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addGroupModel(DEFin_grp, _outCylinderGroupModelName, new CT_StandardItemGroup(), tr("Cylinder group"));
        resCpy_res->addGroupModel(DEFin_grp, _outCylinderGroupModelName2, new CT_StandardItemGroup(), tr("Cylinder group_all"));
        resCpy_res->addItemModel(_outCylinderGroupModelName2, _outCylinderModelName, new CT_Cylinder(), tr("Fitted Cylinder"));

        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_r5,  new CT_Cylinder(), tr("Fitted Cylinder r 5"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_r10, new CT_Cylinder(), tr("Fitted Cylinder r 10"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_r15, new CT_Cylinder(), tr("Fitted Cylinder r 15"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_r20, new CT_Cylinder(), tr("Fitted Cylinder r 20"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_r25, new CT_Cylinder(), tr("Fitted Cylinder r 25"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_r30, new CT_Cylinder(), tr("Fitted Cylinder r 30" ));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_r35, new CT_Cylinder(), tr("Fitted Cylinder r 35"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_r40, new CT_Cylinder(), tr("Fitted Cylinder r 40 "));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_r45, new CT_Cylinder(), tr("Fitted Cylinder r 45"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_r50, new CT_Cylinder(), tr("Fitted Cylinder r 50"));

        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_m5, new CT_Cylinder(), tr("Fitted Cylinder   m 5"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_m10, new CT_Cylinder(), tr("Fitted Cylinder  m 15"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_m15, new CT_Cylinder(), tr("Fitted Cylinder  m 20"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_m20, new CT_Cylinder(), tr("Fitted Cylinder  m 25"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_m25, new CT_Cylinder(), tr("Fitted Cylinder  m 30"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_m30, new CT_Cylinder(), tr("Fitted Cylinder  m 35"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_m35, new CT_Cylinder(), tr("Fitted Cylinder  m 40"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_m40, new CT_Cylinder(), tr("Fitted Cylinder  m 45"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_m45, new CT_Cylinder(), tr("Fitted Cylinder  m 50"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_m50, new CT_Cylinder(), tr("Fitted Cylinder  m 50"));




    }
}

// Semi-automatic creation of step parameters DialogBox
void ST_StepFITMultipleDBH::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addTitle(tr("This method fits a cylinder by various routines into various thick slices."));
    configDialog->addDouble( tr("height of vegetation cloud"), "",  0, 1,3,  _min_z_height, 1, "");
    configDialog->addDouble( tr("center height of the slize"), "",  0, 5,1,  _cut_height, 1, "");

    configDialog->addDouble( tr("distance to cylinder"), "",  0, 1,3,  _max_dist, 1, "distance to cylinder.");
    //    configDialog->addInt(tr("number of inliers" ),"",1,1000, _nmb_inliers, "The number of inliers");
    configDialog->addInt(tr("number of _iterations" ),"",1,1000, _iterations, "The number of _iterations");
}

void ST_StepFITMultipleDBH::compute()
{
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    QString header = "5_R,10_R,15_R,20_R,25_R,30_R,35_R,40_R,45_R,50_R,5_M,10_M,15_M,20_M,25_M,30_M,35_M,40_M,45_M,50_M";
    QString values = "";
    PS_LOG->addInfoMessage(this, header);
    while (itCpy_grp.hasNext() && !isStopped())
    {
        //        qDebug() << "01";
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        CT_Scene* itemIn_scene = (CT_Scene*) grpCpy_grp;
        const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in =
                (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        create_simple_tree_cloud(itemCpy_cloud_in);
        enrich_cloud(itemCpy_cloud_in,resCpy_res,grpCpy_grp);
        QVector<float> length{0.05f,0.1f,0.15f,0.2f,0.25f,0.3f,0.35f,0.4f,0.45f,0.5f};
        QVector<int> type{pcl::SAC_RANSAC,pcl::SAC_MLESAC};
        float min_z = _get_min_z();
        qDebug() << min_z<< "minZ";

        for(size_t i = 0; i < length.size(); i++)
        {

            for(size_t j = 0; j < type.size(); j++)
            {
                int typ = type.at(j);

                float l = length.at(i);
                qDebug () << l <<"the length";
                qDebug() << "01";
                PointCloudS::Ptr slice = extract_slice(l,min_z);
                qDebug() << "02" <<slice ->points.size();

                pcl::PointIndices::Ptr  inliers_cylinder (new pcl::PointIndices);
                pcl::ModelCoefficients coefficients_cylinder;
                pcl::SACSegmentationFromNormals<PointS, PointS> seg;
                seg.setOptimizeCoefficients (true);
                seg.setModelType (pcl::SACMODEL_CYLINDER);
                seg.setMethodType (typ);
                seg.setMaxIterations (_iterations);
                seg.setDistanceThreshold (_max_dist);
                seg.setInputCloud (slice);
                seg.setInputNormals (slice);
                seg.setRadiusLimits (0, 2);
                seg.segment (*inliers_cylinder, coefficients_cylinder);
                float rad = -1;
                if(coefficients_cylinder.values.size()==7)
                {
                    rad = std::abs(coefficients_cylinder.values.at(6));


                    PointS s;
                    PointS e;
                    float min = std::numeric_limits<float>::max();
                    float max = std::numeric_limits<float>::lowest();
                    for(size_t i = 0; i < slice->points.size(); i++)
                    {
                        PointS p = slice->points.at(i);
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
                    QSharedPointer<Cylinder> ransac (new Cylinder(coefficients_cylinder));

                    QSharedPointer<PointS> start (new PointS(s.x,s.y,s.z));
                    QSharedPointer<PointS> end   (new PointS(e.x,e.y,e.z));

                    QSharedPointer<PointS> start2 =  ransac->projection_on_axis_ptr(start);
                    QSharedPointer<PointS> end2 =  ransac->projection_on_axis_ptr(end);

                    ransac->set_start_end(start2,end2);



                    CT_StandardItemGroup* cylinderGroup = new CT_StandardItemGroup(_outCylinderGroupModelName.completeName(), resCpy_res);
                    grpCpy_grp->addGroup(cylinderGroup);

                    CT_StandardItemGroup* cylinderGroup2 = new CT_StandardItemGroup(_outCylinderGroupModelName2.completeName(), resCpy_res);
                    grpCpy_grp->addGroup(cylinderGroup2);


                    QSharedPointer<PointS> center = ransac->get_center_ptr();
                    QSharedPointer<PointS> start3 = ransac->get_start_ptr();
                    QSharedPointer<PointS> stop = ransac->get_end_ptr();


                    CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                                Eigen::Vector3d(stop->x-start3->x, stop->y-start3->y, stop->z-start3->z),
                                                                ransac->get_radius(),
                                                                ransac->get_length());


                    QString test = _outCylinderModelName.completeName();
                    test.append(QString::number(i)).append(QString::number(j));
                                   qDebug() << "i " << i << "j" << j;

                    switch (i) {
                    case 0:
                        if(j == 0)
                        {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_r5.completeName(), resCpy_res, data);
                            cylinderGroup->addItemDrawable(cylinder);}
                        if(j == 1)
                        {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_m5.completeName(), resCpy_res, data);
                            cylinderGroup->addItemDrawable(cylinder);}
                        break;
                    case 1:
                        if(j == 0)
                        {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_r10.completeName(), resCpy_res, data);
                            cylinderGroup->addItemDrawable(cylinder);}
                        if(j == 1)
                        {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_m10.completeName(), resCpy_res, data);
                            cylinderGroup->addItemDrawable(cylinder);}
                        break;
                    case 2:
                            if(j == 0)
                            {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_r15.completeName(), resCpy_res, data);
                                cylinderGroup->addItemDrawable(cylinder);}
                            if(j == 1)
                            {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_m15.completeName(), resCpy_res, data);
                                cylinderGroup->addItemDrawable(cylinder);}
                        break;
                    case 3:
                        if(j == 0)
                        {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_r20.completeName(), resCpy_res, data);
                            cylinderGroup->addItemDrawable(cylinder);}
                        if(j == 1)
                        {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_m20.completeName(), resCpy_res, data);
                            cylinderGroup->addItemDrawable(cylinder);}
                        break;
                    case 4:
                        if(j == 0)
                        {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_r25.completeName(), resCpy_res, data);
                            cylinderGroup->addItemDrawable(cylinder);}
                        if(j == 1)
                        {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_m25.completeName(), resCpy_res, data);
                            cylinderGroup->addItemDrawable(cylinder);}
                        break;
                    case 5:
                        if(j == 0)
                        {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_r30.completeName(), resCpy_res, data);
                            cylinderGroup->addItemDrawable(cylinder);}
                        if(j == 1)
                        {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_m30.completeName(), resCpy_res, data);
                            cylinderGroup->addItemDrawable(cylinder);}
                        break;
                    case 6:
                            if(j == 0)
                            {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_r35.completeName(), resCpy_res, data);
                                cylinderGroup->addItemDrawable(cylinder);}
                            if(j == 1)
                            {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_m35.completeName(), resCpy_res, data);
                                cylinderGroup->addItemDrawable(cylinder);}
                    case 7:
                        if(j == 0)
                        {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_r40.completeName(), resCpy_res, data);
                            cylinderGroup->addItemDrawable(cylinder);}
                        if(j == 1)
                        {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_m40.completeName(), resCpy_res, data);
                            cylinderGroup->addItemDrawable(cylinder);}
                        break;
                    case 8:
                        if(j == 0)
                        {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_r45.completeName(), resCpy_res, data);
                            cylinderGroup->addItemDrawable(cylinder);}
                        if(j == 1)
                        {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_m45.completeName(), resCpy_res, data);
                            cylinderGroup->addItemDrawable(cylinder);}
                        break;
                    case 9:
                        if(j == 0)
                        {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_r50.completeName(), resCpy_res, data);
                            cylinderGroup->addItemDrawable(cylinder);}
                        if(j == 1)
                        {CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_m50.completeName(), resCpy_res, data);
                            cylinderGroup->addItemDrawable(cylinder);}
                        break;

                    default:
                        break;
                    }
                    CT_Cylinder* cylinder2 = new CT_Cylinder(_outCylinderModelName.completeName(), resCpy_res, data);
                                                cylinderGroup2->addItemDrawable(cylinder2);

                } else
                {
                    qDebug() << "not fitted";
                }
                values.append(QString::number(rad)).append(",");


            }
        }

    PS_LOG->addInfoMessage(this, values);


    }
}

float ST_StepFITMultipleDBH::_get_min_z()
{
    float z_min = std::numeric_limits<float>::max();
    for(size_t i = 0; i< _cloud_in->points.size(); i++)
    {
        PointS p = _cloud_in->points.at(i);
        if(z_min>p.z)
        {
            z_min = p.z;
        }
    }
    return z_min;
}

PointCloudS::Ptr ST_StepFITMultipleDBH::extract_slice(float length, float min_z)
{

    PointCloudS::Ptr cloud_filtered (new PointCloudS);
    float min = _cut_height-(length/2);
    float max = _cut_height+(length/2);
    qDebug() << "minmax" << min << max;
    min += min_z - _min_z_height;
    max += min_z - _min_z_height;
    qDebug() << "minmax2" << min << max;

    pcl::PassThrough<PointS> pass;
    pass.setInputCloud (_cloud_in);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (min, max);
    pass.filter (*cloud_filtered);
    return cloud_filtered;
}

void ST_StepFITMultipleDBH::enrich_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp)
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

void ST_StepFITMultipleDBH::create_simple_tree_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in)
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



