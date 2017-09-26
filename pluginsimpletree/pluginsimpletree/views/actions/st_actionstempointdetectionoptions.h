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

#ifndef ST_ACTIONSTEMPOINTDETECTIONOPTIONS_H
#define ST_ACTIONSTEMPOINTDETECTIONOPTIONS_H

#include "ct_view/actions/abstract/ct_gabstractactionoptions.h"

class ST_ActionStemPointDetection;

namespace Ui {
class ST_ActionStemPointDetectionOptions;
}

class ST_ActionStemPointDetectionOptions : public CT_GAbstractActionOptions
{
    Q_OBJECT

public:

    explicit ST_ActionStemPointDetectionOptions(const ST_ActionStemPointDetection *action);
    ~ST_ActionStemPointDetectionOptions();

    double get_eigen_1_min() const;
    double get_eigen_1_max() const;

    double get_eigen_2_min() const;
    double get_eigen_2_max() const;

    double get_eigen_3_min() const;
    double get_eigen_3_max() const;

    void set_eigen_1_min(double x);
    void set_eigen_1_max(double x);
    void set_eigen_2_min(double x);
    void set_eigen_2_max(double x);
    void set_eigen_3_min(double x);
    void set_eigen_3_max(double x);


//    bool isExemple2Checked();

private:
    Ui::ST_ActionStemPointDetectionOptions *ui;

signals:
    void parametersChanged();

};

#endif // ST_ACTIONSTEMPOINTDETECTIONOPTIONS_H
