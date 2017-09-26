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

#include "st_ascidexporter.h"

#include <math.h>
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QEventLoop>
#include <QApplication>
#include <QProgressDialog>
#include <QFileInfo>

#include "ct_global/ct_context.h"
#include "ct_tools/ct_numerictostringconversiont.h"
#include "ct_iterator/ct_pointiterator.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"


ST_ASCIDExporter::ST_ASCIDExporter() : CT_AbstractExporterPointAttributesSelection()
{
}

ST_ASCIDExporter::~ST_ASCIDExporter()
{

}

QString ST_ASCIDExporter::getExporterCustomName() const
{
    return tr("Points + ID item, ASCII(X,Y,Z), 1 file / scene");
}

CT_StepsMenu::LevelPredefined ST_ASCIDExporter::getExporterSubMenuName() const
{
    return CT_StepsMenu::LP_Points;
}

void ST_ASCIDExporter::init()
{
    addNewExportFormat(FileFormat("asc", tr("Fichier asc")));

    setToolTip(tr("Exporte les points au format ASCII. Tous les items contenant des points sont exportés dans le même fichier, avec les champs suivants :<br>"
                  "- ID : ID Computree de l'item contenant le point<br>"
                  "- X  : Coordonnée X<br>"
                  "- Y  : Coordonnée Y<br>"
                  "- Z  : Coordonnée Z<br>"));
}


bool ST_ASCIDExporter::setItemDrawableToExport(const QList<CT_AbstractItemDrawable*> &list)
{
    clearErrorMessage();

    QList<CT_AbstractItemDrawable*> myList;
    QListIterator<CT_AbstractItemDrawable*> it(list);

    while(it.hasNext())
    {
        CT_AbstractItemDrawable *item = it.next();

        if (dynamic_cast<CT_AbstractItemDrawableWithPointCloud*>(item) != NULL)
            myList.append(item);
    }

    if(myList.isEmpty())
    {
        setErrorMessage(tr("Aucun ItemDrawable du type CT_AbstractItemDrawableWithPointCloud"));
        return false;
    }

    return CT_AbstractExporter::setItemDrawableToExport(myList);
}

CT_AbstractExporter* ST_ASCIDExporter::copy() const
{
    return new ST_ASCIDExporter();
}

bool ST_ASCIDExporter::protectedExportToFile()
{
    QFileInfo exportPathInfo = QFileInfo(exportFilePath());




    int nExported = 0;

    int number = 1;

    // write data
    QListIterator<CT_AbstractItemDrawable*> it(itemDrawableToExport());

    while(it.hasNext())
    {
        CT_AbstractItemDrawable *item = it.next();
        size_t id = item->id();

        CT_PointIterator itP(dynamic_cast<CT_AbstractItemDrawableWithPointCloud*>(item)->getPointCloudIndex());

        QString path = exportPathInfo.path();
        QString baseName = exportPathInfo.baseName().append(QString::number(number));
        QString suffix = "asc";
        QString filePath = QString("%1/%2.%4").arg(path).arg(baseName).arg(suffix);

        QFile file(filePath);

        if(file.open(QFile::WriteOnly | QFile::Text))
        {

            QTextStream txtStream(&file);

            txtStream << "ID\tX\tY\tZ\n";

            int totalToExport = itemDrawableToExport().size();



            size_t totalSize = itP.size();
            size_t i = 0;

            while(itP.hasNext())
            {
                const CT_Point &point = itP.next().currentPoint();

                txtStream << id << "\t";
                txtStream << CT_NumericToStringConversionT<double>::toString(point(0)) << "\t";
                txtStream << CT_NumericToStringConversionT<double>::toString(point(1)) << "\t";
                txtStream << CT_NumericToStringConversionT<double>::toString(point(2)) << "\n";

                ++i;

                setExportProgress((((i*100)/totalSize)+nExported)/totalToExport);
            }

            nExported += 100;
            number++;
            file.close();
        } else {
            return false;
        }



    }

    return true;
}
