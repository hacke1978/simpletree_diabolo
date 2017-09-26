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
#include "st_importcoeff.h"

ST_StepImportCoeff::ST_StepImportCoeff(CT_StepInitializeData &dataInit):  CT_AbstractStep(dataInit)

{

}

ST_StepImportCoeff::~ST_StepImportCoeff()
{

}

//// Step description (tooltip of contextual menu)
QString ST_StepImportCoeff::getStepDescription() const
{
    return tr("Imports precomputed method coefficients.");
}

//// Step detailled description
QString ST_StepImportCoeff::getStepDetailledDescription() const
{
    return tr("See for now SimpleTree homepage." );
}

//// Step URL
QString ST_StepImportCoeff::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    //    return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

//Step copy method
CT_VirtualAbstractStep* ST_StepImportCoeff::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepImportCoeff(dataInit);
}



// Creation and affiliation of IN models
void ST_StepImportCoeff::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Isolated Tree"));
    resIn_res->addItemModel(DEFin_grp, DEFin_header, CT_FileHeader::staticGetType(), tr("File Header"));

}

// Creation and affiliation of OUT models
void ST_StepImportCoeff::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
        resCpy_res->addItemModel(DEFin_grp, _coeff_out, new ST_Coefficients(), tr("coefficients"));
}


void ST_StepImportCoeff::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    QString des = "This step will import a set of method coefficients from a preselected folder. The method coefficients file names have to be in agreement with the cloud names.";
    configDialog ->addText("","","",des);
    configDialog->addFileChoice( tr("Select a File containing the method coefficients."), CT_FileChoiceButton::OneExistingFile, "Extension (*.csv)",
                                 _file_name_list, des);
    configDialog->addEmpty();
  dialog_simple_tree(configDialog);
}

void ST_StepImportCoeff::compute()
{
        QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);


    if(!_file_name_list.empty())
    {
        QString path = _file_name_list.at(0);
        ReadCoeff rc(path);
        QVector<MethodCoefficients> cf_list = rc.get_coeff();

        CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
        while (itCpy_grp.hasNext() && !isStopped())
        {
            CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
            CT_FileHeader * itemCpy_header =
                    (CT_FileHeader*)grpCpy_grp->firstItemByINModelName(this, DEFin_header);
            QString cloud_name = itemCpy_header->getFileName();
            QStringList cloud_name_split = cloud_name.split(".");
            cloud_name = cloud_name_split.at(0);

            QVectorIterator<MethodCoefficients> git(cf_list);
            MethodCoefficients coeff;
            bool found = false;
            while(git.hasNext())
            {
                MethodCoefficients cf = git.next();
                QString id = cf.id;
                QStringList id_split_list = id.split(".");
                id = id_split_list.at(0);
                if(id == cloud_name)
                {
                    if(!found)
                    {

                        QString str = id;
                        str.append(" - paramter set found.");
                        PS_LOG->addInfoMessage(this, str);
                        found = true;
                        coeff = cf;
                    } else {
                        QString str = id;
                        str.append(" - multiple parameter set found, only the first is used.");
                        PS_LOG->addInfoMessage(this, str);
                    }
                }
            }
            ST_Coefficients *  st_coeff = new ST_Coefficients(_coeff_out.completeName(), resCpy_res,coeff);
            grpCpy_grp->addItemDrawable(st_coeff);
            if(!found)
            {
                QString str = cloud_name;
                str.append(" - no parameter set found for this cloud, standard parameters used.");
                PS_LOG->addInfoMessage(this, str);
            }


        }
    }
}




