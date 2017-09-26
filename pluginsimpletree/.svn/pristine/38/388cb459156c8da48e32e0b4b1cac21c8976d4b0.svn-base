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

#include "readtree.h"

QVector<QSharedPointer<Cylinder> > ReadTree::get_cylinders() const
{
    return _cylinders;
}

ReadTree::ReadTree(QString path)
{

    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    QFile file(path);
    if(!file.open(QIODevice::ReadOnly)) {
        QMessageBox::information(0, "error in ReadTree", file.errorString());
    }
    QTextStream in (&file);
    int counter = 0;
    _cylinders.clear();
    while(!in.atEnd())
    {
        QString line = in.readLine();
        if(counter>0)
        {
            QStringList fields = line.split(";");
            float sx,sy,sz,ex,ey,ez,rad,length;
            sx = fields.at(9).toFloat();
            sy = fields.at(10).toFloat();
            sz = fields.at(11).toFloat();
            ex = fields.at(12).toFloat();
            ey = fields.at(13).toFloat();
            ez = fields.at(14).toFloat();
            rad = fields.at(15).toFloat();
            length = fields.at(16).toFloat();
            int improvementtype = 0;

            QString str = fields.at(8);
            if(str == QString("RANSAC"))
            {
                improvementtype = 0;
            } else if(str == QString("MEDIAN"))
            {
                improvementtype = 1;
            } else if(str == QString("NO"))
            {
                improvementtype = 2;
            } else
            {
                improvementtype = -1;
            }
//            switch (str) {
//            case "RANSAC":
//                improvementtype = 0;
//                break;
//            case "MEDIAN":
//                improvementtype = 1;
//                break;
//            case "NO":
//                improvementtype = 2;
//                break;
//            default:
//                imrpovementtype = 0;
//                break;
//            }
            int detectiontype;
            str = fields.at(7);
            if(str == QString("spherefollowing"))
            {
                detectiontype = 0;
            } else if(str == QString("attractor"))
            {
                detectiontype = 1;
            } else
            {
                detectiontype = -1;
            }



//            switch (str) {
//            case "spherefollowing":
//                detectiontype = 0;
//                break;
//            case "attractor":
//                detectiontype = 1;
//                break;
//            default:
//                detectiontype = 0;
//                break;
//            }
            pcl::ModelCoefficients coeff;
            coeff.values.push_back(sx);
            coeff.values.push_back(sy);
            coeff.values.push_back(sz);
            coeff.values.push_back(ex-sx);
            coeff.values.push_back(ey-sy);
            coeff.values.push_back(ez-sz);
            coeff.values.push_back(rad);
            QSharedPointer<Cylinder> cyl (new Cylinder(coeff));
            cyl->set_improvement(static_cast<ImprovementType> (improvementtype));
            cyl->set_detection(static_cast<DetectionType>(detectiontype));
            _cylinders.push_back(cyl);
        }
        counter++;
    }
    file.close();

}
