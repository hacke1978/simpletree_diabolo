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
#include "st_step_detect_wrong_fits_in_qsm.h"






ST_StepDetectWrongFitsInQSM::ST_StepDetectWrongFitsInQSM(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _min_rad = 0.035f;
}

ST_StepDetectWrongFitsInQSM::~ST_StepDetectWrongFitsInQSM()
{
}


// Step description (tooltip of contextual menu)
QString ST_StepDetectWrongFitsInQSM::getStepDescription() const
{
    return tr("QSM spherefollowing method - detects and corrects by pype model theory.");
}

// Step detailled description
QString ST_StepDetectWrongFitsInQSM::getStepDetailledDescription() const
{
    return tr("QSM spherefollowing method - detects and corrects by pype model theory.");
}

// Step URL
QString ST_StepDetectWrongFitsInQSM::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    //return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
ST_StepDetectWrongFitsInQSM* ST_StepDetectWrongFitsInQSM::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepDetectWrongFitsInQSM(dataInit);
}





void ST_StepDetectWrongFitsInQSM::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();

    configDialog->addText(tr("You can choose between 3 approaches to correct errors in the QSM."));
    QStringList choices;
    choices.append(pype);
//    choices.append(growth_len);
    choices.append(growth_vol);
    configDialog->addStringChoice( "Please select:","",choices,_selected,"");
    configDialog->addText(tr("An approach was published where from a parent cylinders radius the child cylinders radius"));
    configDialog->addText(tr("can be predicted. (Xu 2007) - pype model theory related"));
    configDialog->addText(tr("The formula is applied to detect wrong cylinder fits"));
    configDialog->addEmpty();
    configDialog->addText(tr("Pype"));
    configDialog->addText(tr("If an inconsistency between the two cylinders radii is given, the child is corrected by the formula (Xu 2007) "));

    configDialog->addEmpty();
    configDialog->addText(tr("GrowthVolume"));
    configDialog->addText(tr("All corrected cylinders are a second time corrected with the GrowthVolume approach published in the SimpleTree paper"));
    configDialog->addEmpty();

    configDialog->addText(tr("GrowthLength"));
    configDialog->addText(tr("The same as growth_volume, but with length instead of volume, a lot less stable"));
    configDialog->addEmpty();
    configDialog->addDouble(tr("The rel error range (0-1) allowed between a modelled and a predicted radius. "),"", 0,  0.5,3,_percentage);

    configDialog->addEmpty();
    configDialog->addText("Xu, H.; Gosset, N.; Chen, B.. Knowledge and Heuristic-Based Modeling of Laser-Scanned Trees");
    configDialog->addText("ACM Trans. Graph. 26, 4, Article 19 (October 2007). - see Equation 4 and 5");

}


// Creation and affiliation of IN models
void ST_StepDetectWrongFitsInQSM::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resIn_res->addItemModel(DEFin_grp, DEFin_coeff_in, ST_Coefficients::staticGetType(), tr ("allometric corrected QSM"));
    resIn_res->addItemModel(DEFin_grp, DEFin_tree_in, ST_Tree::staticGetType(), tr("allometric corrected QSM "));
}

// Creation and affiliation of OUT models
void ST_StepDetectWrongFitsInQSM::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addItemModel(DEFin_grp, _tree_out, new ST_Tree(), tr("tree - modelled with growth volume allometry"));
        resCpy_res->addItemModel(DEFin_grp, _coeff_out, new ST_Coefficients(), tr("coefficients of tree modelled with  with growth volume allometry - by cropping"));
        resCpy_res->addGroupModel(DEFin_grp, _outCylinderGroupModelName, new CT_StandardItemGroup(), tr("Cylinder group  with growth volume  allometry - by cropping"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_improved_by_allometry, new CT_Cylinder(), tr("cylinders uncorrected"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _branchIDModelName,
                                          new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_ID),
                                          tr("branch_ID"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _branchOrderModelName,
                                          new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("branch_order"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _segmentIDModelName,
                                          new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_ID),
                                          tr("segment_ID"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _parentSegmentIDModelName,
                                          new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_ID),
                                          tr("parent_segment_ID"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _growthVolumeModelName,
                                          new CT_StdItemAttributeT<float>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("growth_volume"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _detection_type,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("detection_method"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _improvement_type,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("improvement_method"));





        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_improved_by_allometry_bad, new CT_Cylinder(), tr("cylinders corrected"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_bad, _branchIDModelName_bad,
                                          new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_ID),
                                          tr("branch_ID"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_bad, _branchOrderModelName_bad,
                                          new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("branch_order"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_bad, _segmentIDModelName_bad,
                                          new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_ID),
                                          tr("segment_ID"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_bad, _parentSegmentIDModelName_bad,
                                          new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_ID),
                                          tr("parent_segment_ID"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_bad, _growthVolumeModelName_bad,
                                          new CT_StdItemAttributeT<float>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("growth_volume"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_bad, _detection_type_bad,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("detection_method"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry_bad, _improvement_type_bad,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("improvement_method"));



        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_bo_1, new CT_Cylinder(), tr("Up to Branch Order 0"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_bo_2, new CT_Cylinder(), tr("Up to Branch Order 1"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_bo_3, new CT_Cylinder(), tr("Up to Branch Order 2"));

    }
}



void ST_StepDetectWrongFitsInQSM::add_cylinder_data(QSharedPointer<Tree> tree, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp)
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


            if(cylin->get_segment()->get_branch_order() == 0)
            {
                CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_bo_1.completeName(), resCpy_res, data);
                cylinderGroup->addItemDrawable(cylinder);
            }


            if(cylin->get_segment()->get_branch_order() == 1)
            {
                CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_bo_2.completeName(), resCpy_res, data);
                cylinderGroup->addItemDrawable(cylinder);
            }

            if(cylin->get_segment()->get_branch_order() >= 2)
            {
                CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_bo_3.completeName(), resCpy_res, data);
                cylinderGroup->addItemDrawable(cylinder);
            }

//            if(cylin->get_segment()->get_branch_order() <= 3)
//            {
//                CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_bo_4.completeName(), resCpy_res, data);
//                cylinderGroup->addItemDrawable(cylinder);
//            }


//            if(cylin->get_segment()->get_branch_order() <= 4)
//            {
//                CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_bo_5.completeName(), resCpy_res, data);
//                cylinderGroup->addItemDrawable(cylinder);
//            }




//            if(tree->get_branch_order_from_leave(cylin) == 0)
//            {
//                CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_reverse_bo_1.completeName(), resCpy_res, data);
//                cylinderGroup->addItemDrawable(cylinder);
//            }


//            if(tree->get_branch_order_from_leave(cylin) <= 1)
//            {
//                CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_reverse_bo_2.completeName(), resCpy_res, data);
//                cylinderGroup->addItemDrawable(cylinder);
//            }


//            if(tree->get_branch_order_from_leave(cylin) <= 2)
//            {
//                CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_reverse_bo_3.completeName(), resCpy_res, data);
//                cylinderGroup->addItemDrawable(cylinder);
//            }


//            if(tree->get_branch_order_from_leave(cylin) <= 3)
//            {
//                CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_reverse_bo_4.completeName(), resCpy_res, data);
//                cylinderGroup->addItemDrawable(cylinder);
//            }


//            if(tree->get_branch_order_from_leave(cylin) <= 4)
//            {
//                CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_reverse_bo_5.completeName(), resCpy_res, data);
//                cylinderGroup->addItemDrawable(cylinder);
//            }

//            if(cylin->get_detection() == DetectionType::SPHEREFOLLOWING)
//            {
//                CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_spherefollowing.completeName(), resCpy_res, data);
//                cylinderGroup->addItemDrawable(cylinder);
//            }
//            if(cylin->get_detection() == DetectionType::ATTRACTOR)
//            {
//                CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_attractor.completeName(), resCpy_res, data);
//                cylinderGroup->addItemDrawable(cylinder);
//            }
//            if(cylin->get_segment()->get_branch_order() == 0)
//            {
//                if(cylin->get_detection() == DetectionType::SPHEREFOLLOWING)
//                {
//                    CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_stem_spherefollowing.completeName(), resCpy_res, data);
//                    cylinderGroup->addItemDrawable(cylinder);
//                }
//            }
//            if(cylin->get_segment()->get_branch_order() == 0)
//            {
//                if(cylin->get_detection() == DetectionType::ATTRACTOR)
//                {
//                    CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_stem_attractor.completeName(), resCpy_res, data);
//                    cylinderGroup->addItemDrawable(cylinder);
//                }
//            }
//            if(cylin->get_segment()->get_branch_order() != 0)
//            {
//                if(cylin->get_detection() == DetectionType::SPHEREFOLLOWING)
//                {
//                    CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_branch_spherefollowing.completeName(), resCpy_res, data);
//                    cylinderGroup->addItemDrawable(cylinder);
//                }
//            }
//            if(cylin->get_segment()->get_branch_order() != 0)
//            {
//                if(cylin->get_detection() == DetectionType::ATTRACTOR)
//                {
//                    CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_branch_attractor.completeName(), resCpy_res, data);
//                    cylinderGroup->addItemDrawable(cylinder);
//                }
//            }
//            if(cylin->get_allometry_improvement() != AllometryImproved::ALLOM)
//            {
//                CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_spherefollowing_allom.completeName(), resCpy_res, data);
//                cylinderGroup->addItemDrawable(cylinder);

//            }
//            if(cylin->get_allometry_improvement() != AllometryImproved::NOALLOM)
//            {

//                CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModel_spherefollowing_no_allom.completeName(), resCpy_res, data);
//                cylinderGroup->addItemDrawable(cylinder);

//            }





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

}


void ST_StepDetectWrongFitsInQSM::compute()
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

                coeff.id = file_id;
                coeff.minRad = radius;
                QSharedPointer<Tree> tree = tree_old->clone();
                QSharedPointer<Tree> tree_save = tree_old->clone();
                ImproveByPipeModel pype(tree, true, _percentage);
                if(_selected == "growth_length" )
                {
                    ComputeAllometrySave ca2( tree,false);
                    coeff.a = ca2.get_a();
                    coeff.b = ca2.get_b();
                    coeff.length_a = ca2.get_a();
                    coeff.length_b = ca2.get_b();
                }
                if(_selected == "growth_volume" )
                {
                    ComputeAllometrySave ca2( tree,true);
                    coeff.a = ca2.get_a();
                    coeff.b = ca2.get_b();
                    coeff.length_a = ca2.get_a();
                    coeff.length_b = ca2.get_b();
                }
                float volume_before = tree_save->get_volume();
                float volume_after  = tree->get_volume();


                QSharedPointer<Tree> tree_clone = tree->clone();
                if(volume_after>(volume_before*2.5 ))
                {
                    tree_clone = tree_save;
                }
                 ReorderTree reorder4(tree_clone);

                ST_Tree * st_tree = new ST_Tree(_tree_out.completeName(),resCpy_res,tree_clone);
                grpCpy_grp->addItemDrawable(st_tree);
                ST_Coefficients *  st_coefficients = new ST_Coefficients( _coeff_out.completeName(), resCpy_res,coeff);
                grpCpy_grp->addItemDrawable(st_coefficients);
                add_cylinder_data(tree_clone, resCpy_res, grpCpy_grp);

            }
        }
        int percentage = (((float)processed_clouds)/((float)number_clouds)*100.0f);
        setProgress(percentage);
    }


}
