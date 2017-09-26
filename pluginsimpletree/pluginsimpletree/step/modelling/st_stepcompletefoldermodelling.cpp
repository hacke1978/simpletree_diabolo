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
#include "st_stepcompletefoldermodelling.h"






ST_StepCompleteFolderModelling::ST_StepCompleteFolderModelling(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _knn = 16;
   // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

ST_StepCompleteFolderModelling::~ST_StepCompleteFolderModelling()
{
}


// Step description (tooltip of contextual menu)
QString ST_StepCompleteFolderModelling::getStepDescription() const
{
    return tr("QSM spherefollowing method.");
}

// Step detailled description
QString ST_StepCompleteFolderModelling::getStepDetailledDescription() const
{
    return tr("See SimpleTree homepage." );
}

// Step URL
QString ST_StepCompleteFolderModelling::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    //return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
ST_StepCompleteFolderModelling* ST_StepCompleteFolderModelling::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepCompleteFolderModelling(dataInit);
}





void ST_StepCompleteFolderModelling::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();

    configDialog->addTitle(tr("You will find help and description about the paramters in the following publication."));
    configDialog->addTitle(tr("This is also the correct citation you should give for scientific publications."));
    configDialog->addEmpty();
    configDialog->addTitle(tr("Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P."));
    configDialog->addTitle(tr("<em>SimpleTree —An Efficient Open Source Tool to Build Tree Models from TLS Clouds.</em>"));
    configDialog->addTitle(tr("Forests <b>2015</b>, 6, 4245-4294. "));
    configDialog->addEmpty();
    configDialog->addEmpty();
    QString des = "You need to select a CSV file here, see documentation for the right format of this file.";
    configDialog->addFileChoice( tr("Select File for generating the Map."), CT_FileChoiceButton::OneExistingFile, "Extension (*.csv)",
                                 _file_name_list, des);
    configDialog->addEmpty();
    configDialog->addBool("Uncheck if you do not want to use the DownHillSimplex Parameter Opimiztion --> more stability but less accurate, also saves computation time-","","USE DHS",_use_dhs);

}


// Creation and affiliation of IN models
void ST_StepCompleteFolderModelling::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Isolated Tree"));
    resIn_res->addItemModel(DEFin_grp, DEFin_header, CT_FileHeader::staticGetType(), tr("File Header"));

}

// Creation and affiliation of OUT models
void ST_StepCompleteFolderModelling::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addItemModel(DEFin_grp, _tree_out, new ST_Tree(), tr("tree"));
        resCpy_res->addItemModel(DEFin_grp, _coeff_out, new ST_Coefficients(), tr("coefficients"));


        resCpy_res->addGroupModel(DEFin_grp, _topologyGroup, new CT_TTreeGroup(), tr("Topology"));
        resCpy_res->addGroupModel(_topologyGroup, _stemGroup, new CT_TNodeGroup(), tr("Amap compatible model"));
        resCpy_res->addItemModel(_stemGroup, _stemCylinders, new CT_Cylinder(), tr("Amap studio"));


        resCpy_res->addItemModel(DEFin_grp, _cloud_out_normals, new CT_PointsAttributesNormal(),tr("Normals"));
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_stem, new CT_PointsAttributesScalarTemplated<float>(),tr("Stem flag"));


        resCpy_res->addGroupModel(DEFin_grp, _outCylinderGroupModelName, new CT_StandardItemGroup(), tr("Cylinder group"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_improved_by_allometry, new CT_Cylinder(), tr("Final Model"));



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
    }
}

void ST_StepCompleteFolderModelling::compute()
{

    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    QMap<QString, FileCoefficients> map = get_map();
    while (itCpy_grp.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        _number_trees++;
        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in =
                (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        CT_FileHeader * itemCpy_header =
                (CT_FileHeader*)grpCpy_grp->firstItemByINModelName(this, DEFin_header);
        StepParameter param;
        MethodCoefficients coeff;
        param.itemCpy_header = itemCpy_header;
        param.map = map;
        param.resCpy_res = resCpy_res;
        param.itmgrp = grpCpy_grp;
        param.coeff = coeff;
        param.itemCpy_cloud_in = itemCpy_cloud_in;
        param._cloud_out_normals = _cloud_out_normals.completeName();
        param._cloud_out_stem    = _cloud_out_stem.completeName();
        param._model = _tree_out.completeName();
        param._coeff = _coeff_out.completeName();
        param._outCylinderGroupModelName = _outCylinderGroupModelName.completeName();
        param._outCylinderModelName_improved_branch_junctions = _outCylinderModelName_improved_branch_junctions.completeName();
        param._outCylinderModelName_unimproved = _outCylinderModelName_unimproved.completeName();
        param._outCylinderModelName_removed_false_cylinders = _outCylinderModelName_removed_false_cylinders.completeName();
        param._outCylinderModelName_removed_improved_by_median = _outCylinderModelName_removed_improved_by_median.completeName();
        param._outCylinderModelName_improved_by_fit = _outCylinderModelName_improved_by_fit.completeName();
        param._outCylinderModelName_improved_by_allometry = _outCylinderModelName_improved_by_allometry.completeName();
        param._outCylinderModelName_improved_by_merge = _outCylinderModelName_improved_by_merge.completeName();
        param._branchIDModelName = _branchIDModelName.completeName();
        param._branchOrderModelName = _branchOrderModelName.completeName();
        param._segmentIDModelName = _parentSegmentIDModelName.completeName();
        param._parentSegmentIDModelName = _parentSegmentIDModelName.completeName();
        param._growthVolumeModelName = _growthVolumeModelName.completeName();
        param._tree_species = _tree_species.completeName();
        param._tree_id = _tree_id.completeName();
        param._detection_type = _detection_type.completeName();
        param._improvement_type = _improvement_type.completeName();
        param._topologyGroup = _topologyGroup.completeName();
        param._stemGroup     = _stemGroup.completeName();
        param._stemCylinders = _stemCylinders.completeName();

        param.use_dhs = _use_dhs;


        _step_parameters.push_back(param);
    }
    if(_number_trees<=0)
    {
        _number_trees = 1;
    }
    qDebug() << "foasd";
    QSharedPointer<ModellingThreadPool> mtp(new ModellingThreadPool(_step_parameters));
    QObject::connect(mtp.data(), SIGNAL(emit_finished_tp()), this, SLOT(sent_finished_step()) );
    QObject::connect(mtp.data(), SIGNAL(emit_qstring_tp(QString)), this, SLOT(sent_qstring_step(QString)) );
    qDebug() << "foasda";
    mtp->start_computation();
    qDebug() << "foasdb";
}







QMap<QString, FileCoefficients> ST_StepCompleteFolderModelling::get_map()
{
    QMap<QString, FileCoefficients> map;
    if(!_file_name_list.empty())
    {
        QString file = _file_name_list.at(0);
        ReadCSV read_csv (file);
        map = read_csv.get_map();
        QString success = "Loaded the Map file successful and crate a map with ";
        success.append(QString::number(map.size()));
        success.append("entries.");
        PS_LOG->addInfoMessage(this, success);
    }
    else
    {
        QString path = "D:/Data/output/GT_102.csv";
        QFileInfo check_file(path);
        if(check_file.exists() && check_file.isFile())
        {
            ReadCSV read_csv (path);
            map = read_csv.get_map();
            QString success = "Loaded the Map file successful and create a map with ";
            success.append(QString::number(map.size()));
            success.append("entries.");
            PS_LOG->addInfoMessage(this, success);
        } else
        {
            qDebug() << "Could not load Map file, please rerun the step.";
            PS_LOG->addInfoMessage(this, tr("Could not load Map file, please rerun the step."));
        }
    }

    return map;
}

void ST_StepCompleteFolderModelling::sent_qstring_step(QString str)
{
    PS_LOG->addInfoMessage(this, str);
}

void ST_StepCompleteFolderModelling::sent_finished_step()
{
    _modelled_trees ++;
    int percentage = (((float)_modelled_trees)/((float)_number_trees)*100.0f);
    setProgress(percentage);
}
