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

#include "st_stepexportdtm.h"





ST_StepExportDTM::ST_StepExportDTM(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{

}

ST_StepExportDTM::~ST_StepExportDTM()
{

}


// Step description (tooltip of contextual menu)
QString ST_StepExportDTM::getStepDescription() const
{
    return tr("Raster to ply exporter.");
}

// Step detailled description
QString ST_StepExportDTM::getStepDetailledDescription() const
{
    return tr("Exports a DTM, CHM or similar to ply file format." );
}

// Step URL
QString ST_StepExportDTM::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
}

// Step copy method
ST_StepExportDTM* ST_StepExportDTM::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepExportDTM(dataInit);
}





void ST_StepExportDTM::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addText("");
    configDialog->addEmpty();
    configDialog->addFileChoice( tr("Select a folder to write the output files."), CT_FileChoiceButton::OneExistingFolder, "",
                                 _file_name_list);
    dialog_simple_tree(configDialog);
}


// Creation and affiliation of IN models
void ST_StepExportDTM::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("In Result"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp,  CT_AbstractItemGroup::staticGetType(), tr("In group"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resIn_res->addItemModel(DEFin_grp, DEFin_header, CT_FileHeader::staticGetType(), tr("header"));
    resIn_res->addItemModel(DEFin_grp, DEFin_Raster, CT_Image2D<float>::staticGetType(), tr("dtm"));

}

// Creation and affiliation of OUT models
void ST_StepExportDTM::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);
}

void ST_StepExportDTM::compute()
{
        QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp_header(resCpy_res, this, DEFin_grp);



    CT_Image2D<float>* dtm = NULL;
    CT_FileHeader* header = NULL;
    while (itCpy_grp_header.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp_header.next();
        dtm = (CT_Image2D<float>*) grpCpy_grp->firstItemByINModelName(this, DEFin_Raster);
        header = (CT_FileHeader*)grpCpy_grp->firstItemByINModelName(this, DEFin_header);

        if(header!=NULL && dtm != NULL)
        {
            QString file_name = header->getFileName();
            QString path;
            if(!_file_name_list.empty())
            {
                path = _file_name_list.at(0);
            }


            QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
            QStringList file_list = file_name.split(".");

            QString file_id = file_list.at(0);
            file_id.append("_dtm.ply");
            path.append("/ply/");
            QDir dir(path);
            if(!dir.exists())
            {
                dir.mkpath(".");
            }
            path.append(file_id);






            QFile file(path);

            if(file.open(QIODevice::WriteOnly))
            {
                QTextStream out(&file);
                int number_points =  dtm->xArraySize() * dtm->yArraySize();
                int number_faces  =  ( dtm->xArraySize() - 1 ) * ( dtm->yArraySize() -1) * 2;

                out << "ply\n"
                       "format ascii 1.0\n"
                       "comment author: Jan Hackenberg\n"
                       "comment object: SimpleTree calculated DTM, CHM\n";
                out << "element vertex ";
                out << QString::number(number_points).append("\n");
                out << "property float x\n"
                       "property float y\n"
                       "property float z\n"
                       "property uchar red\n"
                       "property uchar green\n"
                       "property uchar blue\n";
                out << "element face ";
                out << QString::number(number_faces).append("\n");
                out << "property list uchar int vertex_index\n"
                       "end_header\n";





                for(size_t i = 0; i < dtm->yArraySize(); i++)
                {
                    for(size_t j = 0; j < dtm->xArraySize(); j++)
                    {
                        Eigen::Vector3d center;
                        dtm->getCellCenterCoordinates( j,i,center);
                        size_t ind11;
                        dtm->index(j,i,ind11);
                        double x = center(0);
                        double y = center(1);
                        double z = dtm->value(j,i);
                        out << QString::number(x);
                        out << " ";
                        out << QString::number(y);
                        out << " ";
                        out << QString::number(z);
                        out << " ";
                        out << "139 69 19\n";

                    }
                }

                for(size_t i = 0; i < dtm->yArraySize()-1; i++)
                {
                    for(size_t j = 0; j < dtm->xArraySize()-1; j++)
                    {
                        size_t ind11, ind12, ind21,ind22;
                        dtm->index(j,i,ind11);
                        dtm->index(j,i+1,ind12);
                        dtm->index(j+1,i,ind21);
                        dtm->index(j+1,i+1,ind22);

                        out << "3 ";
                        out << QString::number(ind11);
                        out << " ";
                        out << QString::number(ind12);
                        out << " ";
                        out << QString::number(ind22);
                        out << "\n";


                        out << "3 ";
                        out << QString::number(ind22);
                        out << " ";
                        out << QString::number(ind21);
                        out << " ";
                        out << QString::number(ind11);
                        out << "\n";

                    }
                }




                file.close();

            }
        }

    }
}
