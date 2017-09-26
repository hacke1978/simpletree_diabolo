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

#include "readcsv.h"

QMap<QString, FileCoefficients> ReadCSV::get_map() const
{
    return map;
}

ReadCSV::ReadCSV(QString path)
{
    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    map.clear();
    QFile file(path);
    if(!file.open(QIODevice::ReadOnly)) {
        QMessageBox::information(0, "error in ReadCSV", file.errorString());
    }
    QTextStream in (&file);
    while(!in.atEnd())
    {
        QString line = in.readLine();
        QStringList fields = line.split(",");
        QString file = fields.at(0);
        QString output_path = fields.at(1);
        QString species = fields.at(2);
        QString volume_string = fields.at(3);
        QString use_allom = fields.at(4);
        float volume = volume_string.toFloat();
        QString cut_height_string = fields.at(5);
        float cut_height = cut_height_string.toFloat();
        FileCoefficients coeff;
        coeff.file = file;
        coeff.output_path = output_path;
        coeff.species = species;
        coeff.volume = volume;
        coeff.use_allom = use_allom;
        coeff.cut_height = cut_height;
        map.insert(file,coeff);
    }
    file.close();
}
