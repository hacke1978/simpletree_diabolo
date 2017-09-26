#include "st_stepcompletemodellingfromabstract.h"






// Constructor : initialization of parameters
ST_StepCompleteModellingFromAbstract::ST_StepCompleteModellingFromAbstract(CT_StepInitializeData &dataInit) : ST_StepAbstractModelling(dataInit)
{
    _species = "unknown";
    _id      = "file_001";
    _leave_removed = false;
    _is_high_quality = true;

}

ST_StepCompleteModellingFromAbstract::~ST_StepCompleteModellingFromAbstract()
{
    _sceneList->clear();
    delete _sceneList;
}

// Step description (tooltip of contextual menu)
QString ST_StepCompleteModellingFromAbstract::getStepDescription() const
{
    return tr("Complete Modelling of a tree - slow, but accurate.");
}

// Step detailled description
QString ST_StepCompleteModellingFromAbstract::getStepDetailledDescription() const
{
    return tr("Uses heavy optimiztation and should give best results. Can be slow." );
}

// Step URL
QString ST_StepCompleteModellingFromAbstract::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/index.html");
    //return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepCompleteModellingFromAbstract::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepCompleteModellingFromAbstract(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////





// Semi-automatic creation of step parameters DialogBox
void ST_StepCompleteModellingFromAbstract::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();


    configDialog->addString(tr("tree species"), "",_species,"");
    configDialog->addString(tr("tree ID"), "",_id,"");
    configDialog->addBool(tr("twigs have been removed"),tr(""),tr(""),_leave_removed);
    configDialog->addBool(tr("is the cloud off high quality - not much noise in branches left"),tr(""),tr(""),_is_high_quality);
    QString des = "Select a folder to write the output. If non choosen the output will be written to Desktop. edit Desktop not working now.";
    configDialog->addFileChoice( tr("Select Folder for writing an output file."), CT_FileChoiceButton::OneExistingFolder, "",
    _file_name_list, des);
    configDialog->addEmpty();


}

void ST_StepCompleteModellingFromAbstract::compute()
{
    // DONT'T FORGET TO ADD THIS STEP TO THE PLUGINMANAGER !!!!!

    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);

    // IN results browsing


    // COPIED results browsing
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);

    while (itCpy_grp.hasNext() && !isStopped())
    {
        //qDebug() <<  "start";
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        // CT_Scene* itemIn_scene = (CT_Scene*) grpCpy_grp;
        if(_leave_removed&&_is_high_quality)
        {
            _use_allom = "FALSE";
        }
        else
        {
            _use_allom = "TRUE";
        }

        if(!_file_name_list.empty())
        {
            _out_put_path = _file_name_list.at(0);
        }
        else
        {
            _out_put_path = QStandardPaths::displayName(QStandardPaths::DesktopLocation);

        }

        const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        _knn =16;
        create_simple_tree_cloud(itemCpy_cloud_in);
        enrich_cloud(itemCpy_cloud_in,resCpy_res,grpCpy_grp);
        build_tree_model(resCpy_res,grpCpy_grp);
       // qDebug() <<  "fini";
    }
}




