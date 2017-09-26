#include "st_stepcropbranchorder.h"






ST_StepCropBranchOrder::ST_StepCropBranchOrder(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{

    // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

ST_StepCropBranchOrder::~ST_StepCropBranchOrder()
{
}


// Step description (tooltip of contextual menu)
QString ST_StepCropBranchOrder::getStepDescription() const
{
    return tr("QSM spherefollowing method - simulation branch order crop.");
}

// Step detailled description
QString ST_StepCropBranchOrder::getStepDetailledDescription() const
{
    return tr("This step peforms a - simulation with branch order crop.");
}

// Step URL
QString ST_StepCropBranchOrder::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    //return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
ST_StepCropBranchOrder* ST_StepCropBranchOrder::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepCropBranchOrder(dataInit);
}





void ST_StepCropBranchOrder::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();


    configDialog->addText(tr("This step performs an allometric correction of the model."));
    configDialog->addBool(tr("standard deactivated, check visually. On plot might be good"),"","",_redo_pipe);
    configDialog->addInt( tr("the minimum reverse branch order to be croped. "),"",0,100,_min_bo);
    configDialog->addInt(tr("the maximum reverse branch order minus this value to be croped. "),"",0,100,_max_bo);



    configDialog->addFileChoice( tr("Select a folder to write the output files."), CT_FileChoiceButton::OneExistingFolder, "",
                                 _file_name_list);
    dialog_simple_tree(configDialog);

}


// Creation and affiliation of IN models
void ST_StepCropBranchOrder::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Isolated Tree cloud"));
    resIn_res->addItemModel(DEFin_grp, DEFin_coeff_in, ST_Coefficients::staticGetType(), tr ("parameter set"));
    resIn_res->addItemModel(DEFin_grp, DEFin_tree_in, ST_Tree::staticGetType(), tr("tree model"));


}

// Creation and affiliation of OUT models
void ST_StepCropBranchOrder::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addItemModel(DEFin_grp, _tree_out, new ST_Tree(), tr("tree - modelled with growth volume allometry"));
        resCpy_res->addItemModel(DEFin_grp, _coeff_out, new ST_Coefficients(), tr("coefficients of tree modelled with  with growth volume allometry"));
        resCpy_res->addGroupModel(DEFin_grp, _outCylinderGroupModelName, new CT_StandardItemGroup(), tr("Cylinder group  with growth volume  allometry"));
    }
}



QVector<int> ST_StepCropBranchOrder::generate_crop_bo(QSharedPointer<Tree> tree)
{
    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    int max_bo = 0;
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        if(tree->get_branch_order_from_leave(cylinder) > max_bo)
        {
            max_bo = tree->get_branch_order_from_leave(cylinder);
        }
    }
    max_bo = max_bo - _max_bo;
    max_bo = std::max(max_bo,0);
    QVector<int> bos;
    if(_min_bo <= max_bo)
    {
        int current = _min_bo;
        while(current <= max_bo)
        {
            bos.push_back(current);
            current += 1;
        }
    }
    return bos;
}

QSharedPointer<Tree> ST_StepCropBranchOrder::crop_tree(QSharedPointer<Tree> tree, int bo)
{
    QSharedPointer<Tree> clone = tree->clone();
    QVector<QSharedPointer<Cylinder> > cylinders_orginal = tree->get_all_cylinders();
    QVector<QSharedPointer<Cylinder> > cylinders_clone   = clone->get_all_cylinders();
    qDebug() << "A";
    for(size_t i = 0; i < cylinders_clone.size(); i++)
    {
        QSharedPointer<Cylinder> cylinder_original = cylinders_orginal.at(i);
        QSharedPointer<Cylinder> cylinder_clone = cylinders_clone.at(i);
        int branchorder = tree->get_branch_order_from_leave(cylinder_original);
        qDebug ()  << branchorder << "bo" << bo;
        if(branchorder<=bo)
        {
            QSharedPointer<Segment> segment = cylinder_clone->get_segment();
            segment->remove();
        }
    }
    qDebug() << "B";
    return clone;
}




void ST_StepCropBranchOrder::compute()
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
        QString path;
        if(!_file_name_list.empty())
        {
            path = _file_name_list.at(0);
        }
        if(tree_in !=0)
        {
            MethodCoefficients coeff = coeff_in->get_coeff();
            coeff.minRad = _min_bo;

//            qDebug() << coeff.id;
            QString id = coeff.id;
            QSharedPointer<Tree> tree_old = tree_in->getTree();
            QVector<int> bos = generate_crop_bo(tree_old);
            for(int i = 0; i < bos.size(); i++)
            {
                int radius = bos.at(i);
                qDebug() << radius;
                QSharedPointer<Tree> tree_new = crop_tree(tree_old, radius);
                                qDebug() << radius << ";";
                QString id_new = id;
                coeff.id = id_new;
                id_new.append("_");
                int millimeter = radius ;
                id_new.append(QString::number(millimeter));
                for(int j = 0; j < 3; j++)
                {
                    QString id_with_allom = id_new;
                    id_with_allom.append("_");
                    switch (j) {
                    case 0:
                        id_with_allom.append("no_allom");
                        break;
                    case 1:
                        id_with_allom.append("vol_allom");
                        break;
                    case 2:
                        id_with_allom.append("len_allom");
                        break;
                    default:
                        break;
                    }
                    QSharedPointer<Tree> tree_clone = tree_new->clone();

                    switch (j) {
                    case 0:
                    {
                        Export::export_tree_detail(tree_clone,path, id_with_allom,coeff);
                        break;
                    }
                    case 1:
                    {
                        ComputeAllometry ca2(tree_clone,true);
                        coeff.a = ca2.get_a();
                        coeff.b = ca2.get_b();
                        ImproveByAllometry(tree_clone,coeff,coeff.a,coeff.b);

//                        ComputeAllometry ca6(tree_clone,true);
//                        coeff.a = ca6.get_a();
//                        coeff.b = ca6.get_b();
//                        ImproveByAllometry(tree_clone,coeff,coeff.a,coeff.b,2.0,true);

                        if(_redo_pipe)
                        {
                            QVector<QSharedPointer<Cylinder> > cylinders = tree_clone->get_all_cylinders();
                            QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
                            while(it.hasNext())
                            {
                                QSharedPointer<Cylinder> cylinder = it.next();
                                if(cylinder->get_detection()==DetectionType::ATTRACTOR)
                                {
                                    cylinder->set_radius(0);
                                }
                            }
                            ImproveByPipeModel pype(tree_clone,false,0.01);
                        }
                        Export::export_tree_detail(tree_clone,path, id_with_allom,coeff);
                        break;
                    }
                    case 2:
                    {
                        ComputeAllometry ca3(tree_clone,true);
                        coeff.a = ca3.get_a();
                        coeff.b = ca3.get_b();
                        ImproveByAllometry(tree_clone,coeff,coeff.a,coeff.b);

                        ComputeAllometryLength ca4(tree_clone);
                        coeff.length_a = ca4.get_a();
                        coeff.length_b = ca4.get_b();

                        ImproveByAllometryLength(tree_clone,coeff,coeff.a,coeff.b,coeff.length_a, coeff.length_b);
                        if(_redo_pipe)
                        {
                            QVector<QSharedPointer<Cylinder> > cylinders = tree_clone->get_all_cylinders();
                            QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
                            while(it.hasNext())
                            {
                                QSharedPointer<Cylinder> cylinder = it.next();
                                if(cylinder->get_detection()==DetectionType::ATTRACTOR)
                                {
                                    cylinder->set_radius(0);
                                }
                            }
                            ImproveByPipeModel pype(tree_clone,false,0.01);
                        }

                        Export::export_tree_detail(tree_clone,path, id_with_allom,coeff);
                        break;
                    }
                    default:
                    {
                        break;
                    }
                    }
                }
            }
        }
        int percentage = (((float)processed_clouds)/((float)number_clouds)*100.0f);
        setProgress(percentage);
    }
}
