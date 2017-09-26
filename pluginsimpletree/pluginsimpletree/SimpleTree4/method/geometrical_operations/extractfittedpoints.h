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

#ifndef EXTRACTFITTEDPOINTS_H
#define EXTRACTFITTEDPOINTS_H

#include <SimpleTree4/model/tree.h>
#include <SimpleTree4/method/computedistancecylinderscloud.h>

class ExtractFittedPoints
{

    PointCloudS::Ptr _cloud;
    PointCloudS::Ptr _cloud_out;

public:

    ExtractFittedPoints(PointCloudS::Ptr cloud, QVector<pcl::ModelCoefficients> coeff);


    PointCloudS::Ptr get_cloud_out() const;
};

#endif // EXTRACTFITTEDPOINTS_H
