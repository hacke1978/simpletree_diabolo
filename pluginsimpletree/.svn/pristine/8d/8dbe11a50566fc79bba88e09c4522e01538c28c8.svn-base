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
#include "st_stepallometrycorrected_qsm.h"






ST_StepAllometry_Corrected_QSM::ST_StepAllometry_Corrected_QSM(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _min_rad = 0.035f;
}

ST_StepAllometry_Corrected_QSM::~ST_StepAllometry_Corrected_QSM()
{
}


// Step description (tooltip of contextual menu)
QString ST_StepAllometry_Corrected_QSM::getStepDescription() const
{
    return tr("QSM spherefollowing method - extrapolates everything smaller than a certain diameter.");
}

// Step detailled description
QString ST_StepAllometry_Corrected_QSM::getStepDetailledDescription() const
{
    return tr("QSM spherefollowing method - extrapolates everything smaller than a certain diameter.");
}

// Step URL
QString ST_StepAllometry_Corrected_QSM::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    //return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
ST_StepAllometry_Corrected_QSM* ST_StepAllometry_Corrected_QSM::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepAllometry_Corrected_QSM(dataInit);
}





void ST_StepAllometry_Corrected_QSM::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();


    configDialog->addText(tr("This step crops considers the QSM stable from the root."));
    configDialog->addBool(tr("up to the tips up to a certain diameter"),"","",_redo_pipe);
    configDialog->addText(tr("The rest is corrected with an growth volume allometric approach."));
    configDialog->addDouble(tr("A minimum radius up to which the tree model is considered good modelled."),"",0,4, 4,_min_rad);
    dialog_simple_tree(configDialog);

}


// Creation and affiliation of IN models
void ST_StepAllometry_Corrected_QSM::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resIn_res->addItemModel(DEFin_grp, DEFin_coeff_in, ST_Coefficients::staticGetType(), tr ("allometric corrected QSM"));
    resIn_res->addItemModel(DEFin_grp, DEFin_tree_in, ST_Tree::staticGetType(), tr("allometric corrected QSM "));
}

// Creation and affiliation of OUT models
void ST_StepAllometry_Corrected_QSM::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addItemModel(DEFin_grp, _tree_out, new ST_Tree(), tr("tree - modelled with growth volume allometry"));
        resCpy_res->addItemModel(DEFin_grp, _coeff_out, new ST_Coefficients(), tr("coefficients of tree modelled with  with growth volume allometry - by cropping"));
        resCpy_res->addGroupModel(DEFin_grp, _outCylinderGroupModelName, new CT_StandardItemGroup(), tr("Cylinder group  with growth volume  allometry - by cropping"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_improved_by_allometry, new CT_Cylinder(), tr("cylinders good with growth volume allometry"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _branchIDModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("branch_ID"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _branchOrderModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("branch_order"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _segmentIDModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("segment_ID"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _parentSegmentIDModelName,
                                          new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_ID),
                                          tr("parent_segment_ID"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _growthVolumeModelName,
                                          new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("growth_volume"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _detection_type,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("detection_method"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _improvement_type,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("improvement_method"));





        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_improved_by_allometry_bad, new CT_Cylinder(), tr("cylinders unsave - with allometry (advanced)"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_bad, _branchIDModelName_bad,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("branch_ID"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_bad, _branchOrderModelName_bad,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("branch_order"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_bad, _segmentIDModelName_bad,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("segment_ID"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_bad, _parentSegmentIDModelName_bad,
                                          new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_ID),
                                          tr("parent_segment_ID"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_bad, _growthVolumeModelName_bad,
                                          new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("growth_volume"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_bad, _detection_type_bad,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("detection_method"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_bad, _improvement_type_bad,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("improvement_method"));

        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_improved_by_allometry_stem, new CT_Cylinder(), tr("cylinders stem save - with allometry (advanced)"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_stem, _branchIDModelName_stem,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("branch_ID"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_stem, _branchOrderModelName_stem,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("branch_order"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_stem, _segmentIDModelName_stem,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("segment_ID"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_stem, _parentSegmentIDModelName_stem,
                                          new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_ID),
                                          tr("parent_segment_ID"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_stem, _growthVolumeModelName_stem,
                                          new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("growth_volume"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_stem, _detection_type_stem,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("detection_method"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_stem, _improvement_type_stem,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("improvement_method"));



    }
}



void ST_StepAllometry_Corrected_QSM::add_cylinder_data(QSharedPointer<Tree> tree, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp)
{
    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
    bool add_stem = true;
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext()) {
        QSharedPointer<Cylinder> cylin = it.next();
        int branchID = cylin->get_segment()->get_branch_id();
        int branchOrder = cylin->get_segment()->get_branch_order();
        int segmentID = cylin->get_segment()->get_id();
        int parentSegmentID = -1;
        if (!(cylin->get_segment()->is_root()))
        {
            parentSegmentID = cylin->get_segment()->get_parent_segment()->get_id();
        }
        float growthVolume = tree->get_growth_volume(cylin);

        int detection = cylin->get_detection();

        int improvement = cylin->get_improvement();


        CT_StandardItemGroup* cylinderGroup = new CT_StandardItemGroup(_outCylinderGroupModelName.completeName(), resCpy_res);
        grpCpy_grp->addGroup(cylinderGroup);


        QSharedPointer<PointS> center = cylin->get_center_ptr();
        QSharedPointer<PointS> start = cylin->get_start_ptr();
        QSharedPointer<PointS> stop = cylin->get_end_ptr();


        CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                    Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
                                                    cylin->get_radius(),
                                                    cylin->get_length());




        if( (cylin->get_detection()== DetectionType::SPHEREFOLLOWING) && (cylin->get_allometry_improvement() == AllometryImproved::NOALLOM ))
        {
            CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_improved_by_allometry.completeName(), resCpy_res, data);
            cylinderGroup->addItemDrawable(cylinder);

            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, branchID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchOrderModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, branchOrder));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_segmentIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, segmentID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_parentSegmentIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, parentSegmentID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(_growthVolumeModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, growthVolume));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_detection_type.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, detection));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_improvement_type.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, improvement));
        } else {

            CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_improved_by_allometry_bad.completeName(), resCpy_res, data);
            cylinderGroup->addItemDrawable(cylinder);

            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchIDModelName_bad.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, branchID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchOrderModelName_bad.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, branchOrder));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_segmentIDModelName_bad.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, segmentID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_parentSegmentIDModelName_bad.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, parentSegmentID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(_growthVolumeModelName_bad.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, growthVolume));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_detection_type_bad.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, detection));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_improvement_type_bad.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, improvement));
        }




    }

    cylinders = tree->get_stem_cylinders_save();

    QVectorIterator<QSharedPointer<Cylinder> > git(cylinders);
    while(git.hasNext()) {
        QSharedPointer<Cylinder> cylin = git.next();
        int branchID = cylin->get_segment()->get_branch_id();
        int branchOrder = cylin->get_segment()->get_branch_order();
        int segmentID = cylin->get_segment()->get_id();
        int parentSegmentID = -1;
        if (!(cylin->get_segment()->is_root()))
        {
            parentSegmentID = cylin->get_segment()->get_parent_segment()->get_id();
        }
        float growthVolume = tree->get_growth_volume(cylin);

        int detection = cylin->get_detection();

        int improvement = cylin->get_improvement();


        CT_StandardItemGroup* cylinderGroup = new CT_StandardItemGroup(_outCylinderGroupModelName.completeName(), resCpy_res);
        grpCpy_grp->addGroup(cylinderGroup);


        QSharedPointer<PointS> center = cylin->get_center_ptr();
        QSharedPointer<PointS> start = cylin->get_start_ptr();
        QSharedPointer<PointS> stop = cylin->get_end_ptr();


        CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                    Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
                                                    cylin->get_radius(),
                                                    cylin->get_length());



        CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_improved_by_allometry_stem.completeName(), resCpy_res, data);
        cylinderGroup->addItemDrawable(cylinder);

        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchIDModelName_stem.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, branchID));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchOrderModelName_stem.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, branchOrder));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_segmentIDModelName_stem.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, segmentID));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_parentSegmentIDModelName_stem.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, parentSegmentID));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(_growthVolumeModelName_stem.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, growthVolume));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_detection_type_stem.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, detection));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_improvement_type_stem.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, improvement));

    }
}


void ST_StepAllometry_Corrected_QSM::compute()
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

    QVector<float> radii;
    radii.push_back(0.035);

    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);

    while (itCpy_grp.hasNext() && !isStopped())
    {
        processed_clouds++;
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        ST_Coefficients* coeff_in = (ST_Coefficients*) grpCpy_grp->firstItemByINModelName(this,DEFin_coeff_in );
        ST_Tree * tree_in = (ST_Tree*) grpCpy_grp->firstItemByINModelName(this, DEFin_tree_in);

        if(tree_in !=0)
        {
            QSharedPointer<Tree> tree_old = tree_in->getTree();
            if(tree_old!=0)
            {
                MethodCoefficients coeff = coeff_in->get_coeff();
                QString id = coeff.id;

                QStringList file_list = id.split(".");
                QString file_id = file_list.at(0);
                float radius = _min_rad;
                QString extension = QString::number(radius * 1000);
                coeff.id = file_id;
                coeff.minRad = radius;
                QSharedPointer<Tree> tree = tree_old->clone();
                QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
                QVectorIterator<QSharedPointer<Cylinder> > tit(cylinders);
                while(tit.hasNext())
                {
                    QSharedPointer<Cylinder> cyl = tit.next();
                    cyl->set_allometry_improvement(AllometryImproved::NOALLOM);
                }
                QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
                while(it.hasNext())
                {
                    QSharedPointer<Cylinder> cyl = it.next();
                    if(cyl->get_radius()<radius)
                    {
                        QSharedPointer<Segment> segment = cyl->get_segment();
                        QVector<QSharedPointer<Segment> > children = segment->get_child_segments_recursively();
                        QVectorIterator<QSharedPointer<Segment> > pit (children);
                        while(pit.hasNext())
                        {
                            QSharedPointer<Segment> child = pit.next();
                            QVector<QSharedPointer<Cylinder> > seg_cylinders = child->get_cylinders();
                            QVectorIterator<QSharedPointer<Cylinder> > rit (seg_cylinders);
                            while(rit.hasNext())
                            {
                                QSharedPointer<Cylinder> cylinder = rit.next();
                                cylinder->set_allometry_improvement(AllometryImproved::ALLOM);
                            }
                        }
                    }
                }
                ComputeAllometry ca2(tree,true);
                coeff.a = ca2.get_a();
                coeff.b = ca2.get_b();
                ImproveByAllometry(tree,coeff,coeff.a,coeff.b,2.0f, false,true);
                {
                    QVector<QSharedPointer<Cylinder> > cylinders2 = tree->get_all_cylinders();
                    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders2);
                    while(it.hasNext())
                    {
                        QSharedPointer<Cylinder> cylinder = it.next();
                        if(cylinder->get_detection()==DetectionType::ATTRACTOR)
                        {
                            cylinder->set_radius(0);
                        }
                    }
                    ImproveByPipeModel pype(tree,false,0.01);
                }


                QSharedPointer<Tree> tree_clone = tree->clone();
                ST_Tree * st_tree = new ST_Tree(_tree_out.completeName(),resCpy_res,tree_clone);
                grpCpy_grp->addItemDrawable(st_tree);
                ST_Coefficients *  st_coefficients = new ST_Coefficients( _coeff_out.completeName(), resCpy_res,coeff);
                grpCpy_grp->addItemDrawable(st_coefficients);
            }
        }
        int percentage = (((float)processed_clouds)/((float)number_clouds)*100.0f);
        setProgress(percentage);
    }


}
