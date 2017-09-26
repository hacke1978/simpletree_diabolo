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

#ifndef READCSV_H
#define READCSV_H

#include <QString>
#include <QMap>
#include <QFile>
#include <QTextStream>
#include <QStandardPaths>
#include <QDebug>
#include <QMessageBox>


struct FileCoefficients{

    /**
     * @brief file QString for the output name
     */
    QString file = "";

    /**
     * @brief output_path The output path, pointing to Desktop standard
     */
    QString output_path = ""; //QStandardPaths::DesktopLocation;

    /**
     * @brief species A QString containing the species name
     */
    QString species = "unknown_species";

    /**
     * @brief volume a float to contain the volume
     */
    float volume = 0;

    /**
     * @brief use_allom a QString for allometry usage
     */
    QString use_allom = "true";


    float cut_height = 0.2f;
};

class ReadCSV
{
    /**
     * @brief map A Map between the FileCoefficients and the name
     */
    QMap<QString, FileCoefficients> map;
public:
    /**
     * @brief ReadCSV Standard constructor
     * @param path the path to where the csv should be written
     */
    ReadCSV(QString path);

    /**
     * @brief get_map The Getter for the map
     * @return
     */
    QMap<QString, FileCoefficients> get_map() const;
};

#endif // READCSV_H
