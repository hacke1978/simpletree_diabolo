#include "st_steploadsimpletreemodel.h"

ST_StepLoadSimpleTreeModel::ST_StepLoadSimpleTreeModel(CT_StepInitializeData &dataInit):ST_StepAbstractModellingMT(dataInit)
{

}

ST_StepLoadSimpleTreeModel::~ST_StepLoadSimpleTreeModel()
{
    _sceneList->clear();
    delete _sceneList;
}

//// Step description (tooltip of contextual menu)
QString ST_StepLoadSimpleTreeModel::getStepDescription() const
{
    return tr("Loads a SimpleTree output csv file.");
}

//// Step detailled description
QString ST_StepLoadSimpleTreeModel::getStepDetailledDescription() const
{
    return tr("See for now SimpleTree homepage." );
}

//// Step URL
QString ST_StepLoadSimpleTreeModel::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    //    return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

//Step copy method
CT_VirtualAbstractStep* ST_StepLoadSimpleTreeModel::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepLoadSimpleTreeModel(dataInit);
}


void ST_StepLoadSimpleTreeModel::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addTitle(tr("You will find help and description about the paramters in the following publication."));
    configDialog->addTitle(tr("This is also the correct citation you should give for scientific publications."));
    configDialog->addEmpty();
    configDialog->addTitle(tr("Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P."));
    configDialog->addTitle(tr("<em>SimpleTree —An Efficient Open Source Tool to Build Tree Models from TLS Clouds.</em>"));
    configDialog->addTitle(tr("Forests <b>2015</b>, 6, 4245-4294. "));
    configDialog->addEmpty();
    QString des = "You need to select a CSV file here, with a model of SimpleTree method.";
    configDialog->addFileChoice( tr("Select a File containing the SimpleTree model."), CT_FileChoiceButton::OneExistingFile, "Extension (*.csv)",
                                 _file_name_list, des);
    configDialog->addEmpty();
    configDialog->addEmpty();
    configDialog->addEmpty();
}

void ST_StepLoadSimpleTreeModel::compute()
{
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);

    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    while (itCpy_grp.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        for (QStringList::iterator it = _file_name_list.begin();
             it != _file_name_list.end(); ++it)
        {
            QString path = *it;
            ReadTree read_tree(path);
            QVector<QSharedPointer<Cylinder> > cylinders = read_tree.get_cylinders();
            QVectorIterator<QSharedPointer<Cylinder> > git(cylinders);
            while(git.hasNext()) {
                QSharedPointer<Cylinder> cylin = git.next();

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



                CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName_improved_by_allometry.completeName(), resCpy_res, data);
                cylinderGroup->addItemDrawable(cylinder);

                cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_detection_type.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, detection));
                cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_improvement_type.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, improvement));


            }
        }
    }
}




