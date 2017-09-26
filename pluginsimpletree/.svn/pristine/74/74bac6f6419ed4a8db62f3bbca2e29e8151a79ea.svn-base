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

#include "st_stepexportmultipleclouds.h"






ST_StepExportMultipleClouds::ST_StepExportMultipleClouds(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
   // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

ST_StepExportMultipleClouds::~ST_StepExportMultipleClouds()
{
}


// Step description (tooltip of contextual menu)
QString ST_StepExportMultipleClouds::getStepDescription() const
{
    return tr("Exports a list of files to xyb format.");
}

// Step detailled description
QString ST_StepExportMultipleClouds::getStepDetailledDescription() const
{
    return tr("Exports a list of files to xyb format. All files are linked with the name of the file of the input cloud. According file names are written"
              "in a user specific folder." );
}

// Step URL
QString ST_StepExportMultipleClouds::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
}

// Step copy method
ST_StepExportMultipleClouds* ST_StepExportMultipleClouds::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepExportMultipleClouds(dataInit);
}





void ST_StepExportMultipleClouds::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addEmpty();
    QString des = "You need to select a CSV file here, see documentation for the right format of this file.";
    configDialog->addFileChoice( tr("Select a folder to write the output files."), CT_FileChoiceButton::OneExistingFolder, "",
                                 _file_name_list, des);
      dialog_simple_tree(configDialog);
}


// Creation and affiliation of IN models
void ST_StepExportMultipleClouds::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Isolated Tree"));
    resIn_res->addItemModel(DEFin_grp, DEFin_header, CT_FileHeader::staticGetType(), tr("File Header"));

}

// Creation and affiliation of OUT models
void ST_StepExportMultipleClouds::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);
}

void ST_StepExportMultipleClouds::compute()
{
        QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    while (itCpy_grp.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();


        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in =
                (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);

        if(itemCpy_cloud_in!=NULL)
        {
            CT_FileHeader * itemCpy_header =
                    (CT_FileHeader*)grpCpy_grp->firstItemByINModelName(this, DEFin_header);

            QString path;

            if(!_file_name_list.empty())
            {
                path = _file_name_list.at(0);
            }
            QString file_name_total = itemCpy_header->getFileName();
            QStringList pieces = file_name_total.split(".");
            QString file_name = pieces.at(0);

            QString suffix = "xyb";





            QString filePath = QString("%1/%2.%3").arg(path).arg(file_name).arg(suffix);
            QFile file(filePath);

            if(file.open(QFile::WriteOnly | QFile::Text))
            {
                QTextStream txtStream(&file);
                txtStream << "# SCENE XYZ binary format v1.0\n";
                txtStream << "ScanPosition 0.00000000 0.00000000 0.00000000 \n";
                txtStream << "Rows 0\n";
                txtStream << "Cols 0\n";
                file.close();
            }

            if(file.open(QFile::Append))
            {
                //CT_AbstractColorCloud *cc = createColorCloudBeforeExportToFile();

                QDataStream stream(&file);
                stream.setByteOrder(QDataStream::LittleEndian);

                char d_data[8];

                // write header
                d_data[0] = 0;
                d_data[1] = 0;
                d_data[2] = 0;
                d_data[3] = 0;

                stream.writeRawData(d_data, 4);

                // write data
                exportPoints(stream,  itemCpy_cloud_in->getPointCloudIndex(), NULL);

                file.close();

                PS_LOG->addMessage(LogInterface::info, LogInterface::exporter, tr("File %1 created").arg(filePath));

            } else {
                PS_LOG->addMessage(LogInterface::error, LogInterface::exporter, tr("Error: impossible to create the file %1").arg(filePath));
            }

        } else {
            groupsToBeRemoved.push_back(grpCpy_grp);
        }


  //      setExportProgress(100.0*(float)cpt/(float)size);

    }
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
}


void ST_StepExportMultipleClouds::exportPoints(QDataStream &stream,
                                  const CT_AbstractPointCloudIndex *constPCIndex,
                                  const CT_AbstractColorCloud *cc)
{
    CT_PointIterator it(constPCIndex);

    while(it.hasNext())
    {
        const CT_Point &point = it.next().currentPoint();

        stream << point(CT_Point::X);
        stream << point(CT_Point::Y);
        stream << point(CT_Point::Z);

        if(cc == NULL)
        {
            stream << (quint16)0;
        }
        else
        {
            const CT_Color &col = cc->constColorAt(it.cIndex());
            quint16 tmp = (quint16)col.r();
            stream << tmp;
        }
    }
}





