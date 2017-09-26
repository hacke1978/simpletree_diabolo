#include "ST_stepmodellingfolderfromabstract.h"







// Constructor : initialization of parameters
ST_StepModellingFolderFromAbstract::ST_StepModellingFolderFromAbstract(CT_StepInitializeData &dataInit) : ST_StepAbstractModelling(dataInit)
{
//    _knn = 4;
//    _species = "unknown";
//    _id      = "unknown_001";
//    _sceneList = new QList<CT_Scene*>();

//    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

ST_StepModellingFolderFromAbstract::~ST_StepModellingFolderFromAbstract()
{
    _sceneList->clear();
    delete _sceneList;
}

//// Step description (tooltip of contextual menu)
QString ST_StepModellingFolderFromAbstract::getStepDescription() const
{
    return tr("Complete Modelling for one folder (abstract).");
}

//// Step detailled description
QString ST_StepModellingFolderFromAbstract::getStepDetailledDescription() const
{
    return tr("See for now SimpleTree homepage." );
}

//// Step URL
QString ST_StepModellingFolderFromAbstract::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
//    return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

 //Step copy method
CT_VirtualAbstractStep* ST_StepModellingFolderFromAbstract::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepModellingFolderFromAbstract(dataInit);
}




void ST_StepModellingFolderFromAbstract::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();

    configDialog->addEmpty();
    configDialog->addEmpty();
    configDialog->addEmpty();
    QString des = "You need to select a CSV file here, see documentation for the right format of this file.";
    configDialog->addFileChoice( tr("Select File for generating the Map."), CT_FileChoiceButton::OneExistingFile, "Extension (*.csv)",
    _file_name_list, des);
    configDialog->addEmpty();
    configDialog->addEmpty();
    configDialog->addEmpty();
}

void ST_StepModellingFolderFromAbstract::compute()
{
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);

    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    QMap<QString, FileCoefficients> map = get_map();
    while (itCpy_grp.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();


        const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        const CT_FileHeader * itemCpy_header = (CT_FileHeader*)grpCpy_grp->firstItemByINModelName(this, DEFin_header);
        QString cloud_name = itemCpy_header->getFileName();

        FileCoefficients coeff_file = map.value(cloud_name);
        _species = coeff_file.species;
        _id      = coeff_file.file;
        _use_allom = coeff_file.use_allom;
        _out_put_path = coeff_file.output_path;
        QString test = "";
        if(_id == test)
        {
            _id = cloud_name;
        }
        QString start = "Starting computation for cloud ";
        start.append(_id);
            PS_LOG->addInfoMessage(this, start);
        _knn =16;

        create_simple_tree_cloud(itemCpy_cloud_in);
        enrich_cloud(itemCpy_cloud_in, resCpy_res,grpCpy_grp);
        build_tree_model(resCpy_res,grpCpy_grp);


    }
}


