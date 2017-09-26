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
#include "st_stepdetectmisfits.h"






ST_StepDetectMisfits::ST_StepDetectMisfits(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{

    // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

ST_StepDetectMisfits::~ST_StepDetectMisfits()
{
}


// Step description (tooltip of contextual menu)
QString ST_StepDetectMisfits::getStepDescription() const
{
    return tr("QSM spherefollowing method - bad fit detection.");
}

// Step detailled description
QString ST_StepDetectMisfits::getStepDetailledDescription() const
{
    return tr("This step detect badly fitted cylinders.");
}

// Step URL
QString ST_StepDetectMisfits::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    //return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
ST_StepDetectMisfits* ST_StepDetectMisfits::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepDetectMisfits(dataInit);
}





void ST_StepDetectMisfits::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();

    configDialog->addText(tr("This steps detects misfitted cylinders."));
      dialog_simple_tree(configDialog);
    //        configDialog->addDouble(tr("Cut height :"), "m",0 , 10, 2, _cut_height);

}


// Creation and affiliation of IN models
void ST_StepDetectMisfits::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Isolated Tree cloud"));
    resIn_res->addItemModel(DEFin_grp, DEFin_coeff_in, ST_Coefficients::staticGetType(), tr ("parameter set"));
    resIn_res->addItemModel(DEFin_grp, DEFin_tree_in, ST_Tree::staticGetType(), tr("tree model"));
}

// Creation and affiliation of OUT models
void ST_StepDetectMisfits::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addGroupModel(DEFin_grp, _cylinder_group, new CT_StandardItemGroup(), tr("cylinder_grp_out"));
        resCpy_res->addItemModel(_cylinder_group, _cylinders, new CT_Cylinder(), tr("Cylinder"));
        resCpy_res->addGroupModel(DEFin_grp, _cylinder_group_good, new CT_StandardItemGroup(), tr("cylinder_grp_out_good"));
        resCpy_res->addItemModel(_cylinder_group_good, _cylinders_good, new CT_Cylinder(), tr("Cylinders good"));
        resCpy_res->addGroupModel(DEFin_grp, _cylinder_group_bad, new CT_StandardItemGroup(), tr("cylinder_grp_out_bad"));
        resCpy_res->addItemModel(_cylinder_group_bad, _cylinders_bad, new CT_Cylinder(), tr("Cylinders bad"));

        resCpy_res->addItemAttributeModel(_cylinders, _number_pts,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("number of points"));
        resCpy_res->addItemAttributeModel(_cylinders, _average_distance,
                                          new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("distance"));
        resCpy_res->addItemAttributeModel(_cylinders, _average_sqrd_distance,
                                          new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("squared distance"));
        resCpy_res->addItemAttributeModel(_cylinders, _average_sqrd_distance_angle,
                                          new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("squared distance angle"));
    }
}






void ST_StepDetectMisfits::compute()
{
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp_temp(resCpy_res, this, DEFin_grp);
    float number_clouds = 0;
    float processed_clouds = 0;
    while (itCpy_grp_temp.hasNext() && !isStopped())
    {
        itCpy_grp_temp.next();
        number_clouds++;
    }


    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);

    while (itCpy_grp.hasNext() && !isStopped())
    {
        processed_clouds++;
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();

        ST_Coefficients * coeff_in = (ST_Coefficients*) grpCpy_grp->firstItemByINModelName(this, DEFin_coeff_in);
        ST_Tree * tree_in = (ST_Tree*) grpCpy_grp->firstItemByINModelName(this, DEFin_tree_in);
        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in =
                (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        if(tree_in !=0)
        {
            QSharedPointer<Tree> tree = tree_in->getTree()->clone();
            PointCloudS::Ptr cloud = pcl_CT_to_PCL_cloud(itemCpy_cloud_in,this,16,false,false);
            pcl_compute_normals(cloud,16);
            DetectFalseCylinders(cloud,tree);

            float max_mean_distance = 0;
            float max_mean_distance_sqrd = 0;
            float max_mean_distance_sqrd_angle = 0;

            QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();

            QVector<float> good_distances;
            QVector<float> good_distances_sqrd;
            QVector<float> good_distances_sqrd_angle;

            QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
            while(it.hasNext())
            {
                QSharedPointer<Cylinder> cylinder = it.next();
                if(cylinder->get_detection()== DetectionType::SPHEREFOLLOWING && cylinder->getNumber_pts() >= MIN_PTS)
                {
                    if(cylinder->getMean_distance()> max_mean_distance)
                    {
                        max_mean_distance = cylinder->getMean_distance();
                        good_distances.push_back(cylinder->getMean_distance());
                    }
                    if(cylinder->getMean_distance_sqrd()> max_mean_distance_sqrd)
                    {
                        max_mean_distance_sqrd = cylinder->getMean_distance_sqrd();
                        good_distances_sqrd.push_back(cylinder->getMean_distance_sqrd());
                    }
                    if(cylinder->getMean_distance_sqrd_angle()> max_mean_distance_sqrd_angle)
                    {
                        max_mean_distance_sqrd_angle = cylinder->getMean_distance_sqrd_angle();
                        good_distances_sqrd_angle.push_back(cylinder->getMean_distance_sqrd_angle());
                    }
                }
            }
            _mean = SimpleMath<float>::get_mean(good_distances);
            _sd   = SimpleMath<float>::get_standard_deviation(good_distances);

            float threshold = _mean+(_sd_mult*_sd);
            if(threshold<=0)
                threshold = _mean;

            QVectorIterator<QSharedPointer<Cylinder> > pit(cylinders);
            while(pit.hasNext()) {
                QSharedPointer<Cylinder> cylin = pit.next();

                int number = cylin->getNumber_pts();
                if(cylin->getMean_distance()<threshold&&number >= MIN_PTS)
                {
                    cylin->setWell_fitted(true);
                } else {
                    cylin->setWell_fitted(false);
                }
            }
            MethodCoefficients cf = coeff_in->get_coeff();

            Stem_Taper taper(tree,cf);









            QVectorIterator<QSharedPointer<Cylinder> > git(cylinders);
            while(git.hasNext()) {
                QSharedPointer<Cylinder> cylin = git.next();
                if(cylin->get_detection()==DetectionType::SPHEREFOLLOWING)
                {
                    int number = cylin->getNumber_pts();
                    float distance = cylin->getMean_distance() /max_mean_distance;
                    float distance_sqrd = cylin->getMean_distance_sqrd()/max_mean_distance_sqrd;
                    float distance_sqrd_angle = cylin->getMean_distance_sqrd_angle()/max_mean_distance_sqrd_angle;
                    if(number >= MIN_PTS)
                    {
                        CT_StandardItemGroup* cylinderGroup = new CT_StandardItemGroup(_cylinder_group.completeName(), resCpy_res);
                        grpCpy_grp->addGroup(cylinderGroup);
                        QSharedPointer<PointS> center = cylin->get_center_ptr();
                        QSharedPointer<PointS> start = cylin->get_start_ptr();
                        QSharedPointer<PointS> stop = cylin->get_end_ptr();
                        CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                                    Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
                                                                    cylin->get_radius(),
                                                                    cylin->get_length());
                        CT_Cylinder* cylinder = new CT_Cylinder(_cylinders.completeName(), resCpy_res, data);
                        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_number_pts.completeName(),
                                                                                 CT_AbstractCategory::DATA_NUMBER, resCpy_res, number));
                        cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(_average_distance.completeName(),
                                                                                   CT_AbstractCategory::DATA_NUMBER, resCpy_res,distance));
                        cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(_average_sqrd_distance.completeName(),
                                                                                   CT_AbstractCategory::DATA_NUMBER, resCpy_res, distance_sqrd));
                        cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(_average_sqrd_distance_angle.completeName(),
                                                                                   CT_AbstractCategory::DATA_NUMBER, resCpy_res, distance_sqrd_angle));
                        cylinderGroup->addItemDrawable(cylinder);
                    }
                }
            }


            QVectorIterator<QSharedPointer<Cylinder> > tit(cylinders);
            while(tit.hasNext()) {
                QSharedPointer<Cylinder> cylin = tit.next();
                int number = cylin->getNumber_pts();
                if(cylin->getMean_distance()<threshold&&number >= MIN_PTS)
                {

                    CT_StandardItemGroup* cylinderGroup = new CT_StandardItemGroup(_cylinder_group_good.completeName(), resCpy_res);
                    grpCpy_grp->addGroup(cylinderGroup);
                    QSharedPointer<PointS> center = cylin->get_center_ptr();
                    QSharedPointer<PointS> start = cylin->get_start_ptr();
                    QSharedPointer<PointS> stop = cylin->get_end_ptr();
                    CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                                Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
                                                                cylin->get_radius(),
                                                                cylin->get_length());
                    CT_Cylinder* cylinder = new CT_Cylinder(_cylinders_good.completeName(), resCpy_res, data);
                    cylinderGroup->addItemDrawable(cylinder);
                } else {
                    CT_StandardItemGroup* cylinderGroup = new CT_StandardItemGroup(_cylinder_group_bad.completeName(), resCpy_res);
                    grpCpy_grp->addGroup(cylinderGroup);
                    QSharedPointer<PointS> center = cylin->get_center_ptr();
                    QSharedPointer<PointS> start = cylin->get_start_ptr();
                    QSharedPointer<PointS> stop = cylin->get_end_ptr();
                    CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                                Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
                                                                cylin->get_radius(),
                                                                cylin->get_length());
                    CT_Cylinder* cylinder = new CT_Cylinder(_cylinders_bad.completeName(), resCpy_res, data);
                    cylinderGroup->addItemDrawable(cylinder);
                }
            }
        }
        int percentage = (((float)processed_clouds)/((float)number_clouds)*100.0f);
        setProgress(percentage);
    }
}

QVector<float> ST_StepDetectMisfits::normalize(const QVector<float> vec, float max)
{
    if(max == 0)
    {
        return vec;
    }
    QVector<float> result;
    QVectorIterator<float> it (vec);
    while(it.hasNext())
    {
        float number = it.next();
        float number_norm = number/max;
        result.push_back(number_norm);
    }
    return result;
}


