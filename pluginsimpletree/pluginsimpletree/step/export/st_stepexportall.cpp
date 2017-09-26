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

#include "st_stepexportall.h"





ST_StepExportAll::ST_StepExportAll(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

ST_StepExportAll::~ST_StepExportAll()
{
}


// Step description (tooltip of contextual menu)
QString ST_StepExportAll::getStepDescription() const
{
    return tr("Exports SimpleTree QSM output.");
}

// Step detailled description
QString ST_StepExportAll::getStepDetailledDescription() const
{
    return tr("Exports SimpleTree QSM output. All files are linked with the name of the file of the input cloud. According file names are written"
              "in a user specific folder. You can generate ply files, parameter output, QSM files and plot output." );
}

// Step URL
QString ST_StepExportAll::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
}

// Step copy method
ST_StepExportAll* ST_StepExportAll::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepExportAll(dataInit);
}





void ST_StepExportAll::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addText("This step can create different output files. Please unselect if you want to print less information");
    configDialog->addEmpty();
    configDialog->addBool("Save one detailed CSV file per tree","","",_save_st_tree);
    //    configDialog->addBool("Save one Amap file per tree (not working at the moment)","","",_save_amap_tree);
    configDialog->addBool("Save the computed parameters in one file per pipeline","","",_save_coefficients);
    configDialog->addBool("Save the most important forestry parameters in one file per pipeline","","",_save_all);
    configDialog->addBool("Save the segmented trees into 1 file/scene","","",_save_cloud);
    configDialog->addBool("Save one ply file per pipeline","","",_save_ply);
    configDialog->addFileChoice( tr("Select a folder to write the output files."), CT_FileChoiceButton::OneExistingFolder, "",
                                 _file_name_list);
    dialog_simple_tree(configDialog);
}


// Creation and affiliation of IN models
void ST_StepExportAll::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("tree_model_in"));
    resIn_res->setZeroOrMoreRootGroup();

    resIn_res->addGroupModel("", DEFin_grp,  CT_AbstractItemGroup::staticGetType(), tr("tree_model_group"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resIn_res->addItemModel(DEFin_grp, DEFin_model_in, ST_Tree::staticGetType(), tr("tree"));
    resIn_res->addItemModel(DEFin_grp, DEFin_coeff_in, ST_Coefficients::staticGetType(), tr("coeff"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Segmented tree cloud"));



    CT_InResultModelGroup *resHeader = createNewInResultModel(DEF_SearchInHeaderResult, tr("Header model in"), "", true);
    resHeader->setZeroOrMoreRootGroup();
    resHeader->addGroupModel("", DEFin_grp3, CT_AbstractItemGroup::staticGetType(), tr("header_in"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resHeader->addItemModel(DEFin_grp3, DEFin_header, CT_FileHeader::staticGetType(), tr("File Header"));







//    CT_InResultModelGroup *resultMNT = createNewInResultModel(DEFin_res2, tr("Tarnsformation Matrix group)"), "", true);
//    resultMNT->setZeroOrMoreRootGroup();
//    resultMNT->addGroupModel("", DEFin_grp2);
//    resultMNT->addItemModel(DEFin_grp2, DEFin_trMat, CT_TransformationMatrix::staticGetType(), tr("Tarnsformation Matrix") );
//    resultMNT->setMinimumNumberOfPossibilityThatMustBeSelectedForOneTurn(0);


}

// Creation and affiliation of OUT models
void ST_StepExportAll::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);
}

void ST_StepExportAll::compute()
{
    QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    QVector<MethodCoefficients> coeff_vec;
    QVector<QSharedPointer<Tree> > tree_vec;
    QString path;


    CT_TransformationMatrix* tm  = NULL;
    if (getInputResults().size() > 2)
    {
        CT_ResultGroup* inMNTResult = getInputResults().at(2);
        CT_ResultItemIterator it(inMNTResult, this, DEFin_grp2);
        if(it.hasNext())
        {
            CT_StandardItemGroup* grpCpy_grp2 = (CT_StandardItemGroup*) it.next();
            tm = (CT_TransformationMatrix*) grpCpy_grp2->firstItemByINModelName(this, DEFin_trMat);
        }
    }

    CT_ResultGroup* inHeaderResult = getInputResults().at(1);
    CT_ResultGroupIterator it_HeaderGrp(inHeaderResult, this, DEFin_grp3);
    QString name_of_plot;
    int size_of_headers = 0;

    while (it_HeaderGrp.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grp_header = (CT_StandardItemGroup*) it_HeaderGrp. next();
        CT_FileHeader* itemCpy_header  = (CT_FileHeader*)grp_header->firstItemByINModelName(this, DEFin_header);
        name_of_plot = itemCpy_header->getFileName();
        size_of_headers++;
    }
    if(!_file_name_list.empty())
    {
        path = _file_name_list.at(0);
    }
    if(size_of_headers == 1)
            {
        path.append("/").append(name_of_plot).append("/");
    }
    while (itCpy_grp.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        ST_Tree * tree_model = (ST_Tree*) grpCpy_grp->firstItemByINModelName(this,DEFin_model_in);
        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in =
                (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);

        if(tree_model!=0)
        {
            if(tree_model->getTree()!=0)
            {



                QSharedPointer<Tree> tree = tree_model->getTree();
//                if(tm!= NULL)

//                {
//                    QSharedPointer<Tree> tree_clone = tree->clone();
//                    QVector<QSharedPointer<Cylinder> > cylinders =  tree_clone->get_all_cylinders();
//                    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);

//                    while(it.hasNext())
//                    {
//                        QSharedPointer<Cylinder> cylinder = it.next();
//                        QSharedPointer<PointS> start = cylinder->get_start_ptr();
//                        Eigen::Vector3d start_vec(start->x,start->y,start->z);
//                        Eigen::Vector3d start_back_translated = tm->getTransformed(start_vec);

//                        start->x = start_back_translated(0);
//                        start->y = start_back_translated(1);
//                        start->z = start_back_translated(2);
//                        QSharedPointer<PointS> end = cylinder->get_end_ptr();

//                        Eigen::Vector3d end_vec(end->x,end->y,end->z);
//                        Eigen::Vector3d end_back_translated = tm->getTransformed(end_vec);

//                        end->x = end_back_translated(0);
//                        end->y = end_back_translated(1);
//                        end->z = end_back_translated(2);
//                        cylinder->set_start_end(start,end);
//                    }
//                    tree = tree_clone;

//                }
                if(tree->get_all_cylinders().size()!=0)
                {
                    ST_Coefficients * coefficients = (ST_Coefficients*) grpCpy_grp->firstItemByINModelName(this, DEFin_coeff_in);

                    coeff_vec.push_back(coefficients->get_coeff());

                    tree_vec.push_back(tree);

                    QString file_name = coefficients->get_coeff().id;

                    if(_save_st_tree)
                    {
                        Export::export_tree_detail(tree,path, file_name,coefficients->get_coeff());
                    }

                    if(_save_cloud)
                    {
                        QString path2 = path;
                        QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
                        QStringList file_list = file_name.split(".");
                        QString file_id = file_list.at(0);
                        file_id.append(".asc");
                        path2.append("cloud/");
                        QDir dir(path2);
                        if(!dir.exists())
                        {
                            dir.mkpath(".");
                        }
                        path2.append(file_id);
                        QFile file(path2);
                        if(file.open(QIODevice::WriteOnly))
                        {
                            QTextStream txtStream(&file);
                            const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in->getPointCloudIndex();
                            CT_PointIterator itP(pointCloudIndex);
                            while(itP.hasNext())
                            {
                                const CT_Point &point = itP.next().currentPoint();
                                txtStream << CT_NumericToStringConversionT<double>::toString(point(0)) << "\t";
                                txtStream << CT_NumericToStringConversionT<double>::toString(point(1)) << "\t";
                                txtStream << CT_NumericToStringConversionT<double>::toString(point(2)) << "\n";
                            }
                        }
                        file.close();
                    }


                    if(_save_ply)
                    {
                        Export::export_ply( path, file_name,tree,8);
                    }
                } else {
                    groupsToBeRemoved.push_back(grpCpy_grp);
                }


            } else {
                groupsToBeRemoved.push_back(grpCpy_grp);
            }
        }else {
            groupsToBeRemoved.push_back(grpCpy_grp);
        }

    }

    if(_save_coefficients)
    {
        Export::export_coefficients(path,coeff_vec);
    }

    if(_save_all)
    {
        Export::export_tree_list(tree_vec, path,coeff_vec);
    }
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
}
