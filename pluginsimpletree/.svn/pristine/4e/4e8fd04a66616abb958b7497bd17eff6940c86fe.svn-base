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
#include "st_step_adjust_branch_order.h"






ST_StepAdjustBranchOrder::ST_StepAdjustBranchOrder(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
}

ST_StepAdjustBranchOrder::~ST_StepAdjustBranchOrder()
{
}


// Step description (tooltip of contextual menu)
QString ST_StepAdjustBranchOrder::getStepDescription() const
{
    return tr("QSM correct the branch order.");
}

// Step detailled description
QString ST_StepAdjustBranchOrder::getStepDetailledDescription() const
{
    return tr("You can select the way the branch order is choosen.");
}

// Step URL
QString ST_StepAdjustBranchOrder::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    //return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

QString ST_StepAdjustBranchOrder::getStepName() const
{
    return "ST_StepAdjustBranchOrder";
}

// Step copy method
ST_StepAdjustBranchOrder* ST_StepAdjustBranchOrder::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepAdjustBranchOrder(dataInit);
}





void ST_StepAdjustBranchOrder::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();

    configDialog->addText(tr("You can choose between 3 approaches to detect the branch order changes."));
    configDialog->addText(tr("The stem detection will rely on this selection as well."));
    QStringList choices;
    choices.append(dia);
    choices.append(vol);
    choices.append(growth_vol);
    choices.append(direction);

    configDialog->addStringChoice( "Please select:","",choices,_selected,"");
    configDialog->addText(tr("If diameter is selected, the branch order will remain the same after a branch junction,"));
    configDialog->addText(tr("for the branch which has the largest diameter after the branch junction(tradtional definition)"));

    configDialog->addText(tr("If volume is selected, the branch order will remain the same after a branch junction,"));
    configDialog->addText(tr("for the branch which has the largest volume up to the tip after the branch junction"));


    configDialog->addText(tr("If growth_volume is selected, the branch order will remain the same after a branch junction,"));
    configDialog->addText(tr("for the branch which has the largest cumulative volume up to all its tips after the branch junction"));

}


// Creation and affiliation of IN models
void ST_StepAdjustBranchOrder::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Isolated Tree cloud"));
    resIn_res->addItemModel(DEFin_grp, DEFin_coeff_in, ST_Coefficients::staticGetType(), tr ("parameter set"));
    resIn_res->addItemModel(DEFin_grp, DEFin_tree_in, ST_Tree::staticGetType(), tr("tree model"));
}

// Creation and affiliation of OUT models
void ST_StepAdjustBranchOrder::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addItemModel(DEFin_grp, _tree_out, new ST_Tree(), tr("tree modelled with Growth_Length allometry"));
        resCpy_res->addItemModel(DEFin_grp, _coeff_out, new ST_Coefficients(), tr("coefficients of tree modelled with Growth_Length  allometry"));
        resCpy_res->addGroupModel(DEFin_grp, _outCylinderGroupModelName, new CT_StandardItemGroup(), tr("Cylinder group modelled with Growth_Length allometry" ));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_improved_by_allometry, new CT_Cylinder(), tr("cylinders good with Growth_Length allometry"));

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



    }
}



void ST_StepAdjustBranchOrder::add_cylinder_data(QSharedPointer<Tree> tree, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp)
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

        if( (cylin->get_segment()->get_branch_order()==0)&& add_stem)
        {
            if(cylin->get_length()>=0.7)
            {
                add_stem = false;
            } else if (cylin->get_detection()!= DetectionType::SPHEREFOLLOWING)
            {
                add_stem = false;
            } else

            {

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
    }
}


void ST_StepAdjustBranchOrder::compute()
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
        ST_Coefficients* coeff_in = (ST_Coefficients*) grpCpy_grp->firstItemByINModelName(this,DEFin_coeff_in );
        ST_Tree * tree_in = (ST_Tree*) grpCpy_grp->firstItemByINModelName(this, DEFin_tree_in);

        if(tree_in !=0)
        {
            MethodCoefficients coeff = coeff_in->get_coeff();
            coeff.minRad = _min_rad;
            QSharedPointer<Tree> tree_old = tree_in->getTree();
            if(tree_old!=0)
            {
                QSharedPointer<Tree> tree = tree_old->clone();

                if(_selected == dia)
                {
                    ReorderTree reorder(tree,0);
                }

                if(_selected == vol)
                {
                    ReorderTree reorder(tree,1);
                }

                if(_selected == growth_vol)
                {
                    ReorderTree reorder(tree,2);
                }


                if(_selected == direction)
                {
                    ReorderTree reorder(tree,3);
                }



//                if(coeff.tree_height < 10)
//                {
//                    // qDebug() << "small";
//                    ComputeAllometry ca2(tree);
//                    coeff.a = ca2.get_a();
//                    coeff.b = ca2.get_b();
//                    ImproveByAllometry(tree,coeff,coeff.a,coeff.b);
//                    ComputeAllometry ca3(tree,true); //
//                    coeff.a = ca3.get_a();
//                    coeff.b = ca3.get_b();
//                    ImproveByAllometry(tree,coeff,coeff.a,coeff.b);
//                    ComputeAllometry ca4(tree,true);//,true
//                    coeff.a = ca4.get_a();
//                    coeff.b = ca4.get_b();
//                    ImproveByAllometry(tree,coeff,coeff.a,coeff.b,1.3f);
//                } else {
//                    ComputeAllometry ca2(tree,true);
//                    coeff.a = ca2.get_a();
//                    coeff.b = ca2.get_b();
//                    ImproveByAllometry(tree,coeff,coeff.a,coeff.b);
//                }
//                ComputeAllometryLength ca(tree);
//                coeff.length_a = ca.get_a();
//                coeff.length_b = ca.get_b();

//                ImproveByAllometryLength(tree,coeff,coeff.a,coeff.b,coeff.length_a, coeff.length_b);
//                if(_redo_pipe)
//                {
//                    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
//                    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
//                    while(it.hasNext())
//                    {
//                        QSharedPointer<Cylinder> cylinder = it.next();
//                        if(cylinder->get_detection()==DetectionType::ATTRACTOR)
//                        {
//                            cylinder->set_radius(0);
//                        }
//                    }
//                    ImproveByPipeModel pype(tree);
//                }


                add_cylinder_data(tree, resCpy_res, grpCpy_grp);
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
