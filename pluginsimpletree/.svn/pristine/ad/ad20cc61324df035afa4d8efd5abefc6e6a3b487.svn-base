#include "st_stepincreasemindiameter.h"






ST_StepIncreaseDiameter::ST_StepIncreaseDiameter(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{

    // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

ST_StepIncreaseDiameter::~ST_StepIncreaseDiameter()
{
}


// Step description (tooltip of contextual menu)
QString ST_StepIncreaseDiameter::getStepDescription() const
{
    return tr("QSM spherefollowing method - simulation diameter increase.");
}

// Step detailled description
QString ST_StepIncreaseDiameter::getStepDetailledDescription() const
{
    return tr("This step peforms a - simulation with diameter increase.");
}

// Step URL
QString ST_StepIncreaseDiameter::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    //return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
ST_StepIncreaseDiameter* ST_StepIncreaseDiameter::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepIncreaseDiameter(dataInit);
}





void ST_StepIncreaseDiameter::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();


    configDialog->addText(tr("This step performs an allometric correction of the model."));
    configDialog->addBool(tr("standard deactivated, check visually. On plot might be good"),"","",_redo_pipe);
    configDialog->addDouble(tr("A minimum radius for the corrected cylinders. "),"",0,4,4,_min_rad);
    configDialog->addDouble(tr("A maximum radius for the corrected cylinders. "),"",0,4,4,_max_rad);



    configDialog->addFileChoice( tr("Select a folder to write the output files."), CT_FileChoiceButton::OneExistingFolder, "",
                                 _file_name_list);
    dialog_simple_tree(configDialog);

}


// Creation and affiliation of IN models
void ST_StepIncreaseDiameter::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Isolated Tree cloud"));
    resIn_res->addItemModel(DEFin_grp, DEFin_coeff_in, ST_Coefficients::staticGetType(), tr ("parameter set"));
    resIn_res->addItemModel(DEFin_grp, DEFin_tree_in, ST_Tree::staticGetType(), tr("tree model"));


}

// Creation and affiliation of OUT models
void ST_StepIncreaseDiameter::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addItemModel(DEFin_grp, _tree_out, new ST_Tree(), tr("tree - modelled with growth volume allometry"));
        resCpy_res->addItemModel(DEFin_grp, _coeff_out, new ST_Coefficients(), tr("coefficients of tree modelled with  with growth volume allometry"));
        resCpy_res->addGroupModel(DEFin_grp, _outCylinderGroupModelName, new CT_StandardItemGroup(), tr("Cylinder group  with growth volume  allometry"));
    }
}



QVector<double> ST_StepIncreaseDiameter::generate_min_radii()
{
    QVector<double> radii;
    if(_min_rad <= _max_rad)
    {
        double current = _min_rad;
        while(current <= _max_rad)
        {
            radii.push_back(current);
            current += _increment;
        }
    }
    return radii;
}

QSharedPointer<Tree> ST_StepIncreaseDiameter::set_min_radius(QSharedPointer<Tree> tree, double radius)
{
    QSharedPointer<Tree> clone = tree->clone();
    QVector<QSharedPointer<Cylinder> > cylinders = clone->get_all_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        if(cylinder->get_radius() < radius)
        {
            cylinder->set_radius(radius);
        }
    }
    return clone;
}


void ST_StepIncreaseDiameter::compute()
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
            coeff.minRad = _min_rad;
            QString id= coeff.id.split(".",QString::SkipEmptyParts).at(0);
            QSharedPointer<Tree> tree_old = tree_in->getTree();
            QVector<double> radii = generate_min_radii();
            for(int i = 0; i < radii.size(); i++)
            {
                double radius = radii.at(i);
                QSharedPointer<Tree> tree_new = set_min_radius(tree_old, radius);
                QString id_new = id;
                coeff.id = id_new;
                id_new.append("_");
                int millimeter = radius * 1000;
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

//                        ComputeAllometry ca5(tree_clone,true);
//                        coeff.a = ca5.get_a();
//                        coeff.b = ca5.get_b();
//                        ImproveByAllometry(tree_clone,coeff,coeff.a,coeff.b,2.0,true);

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
