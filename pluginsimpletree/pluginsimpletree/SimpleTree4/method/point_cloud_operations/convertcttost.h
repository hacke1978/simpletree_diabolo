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

#ifndef CONVERTCTTOST_H
#define CONVERTCTTOST_H

#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "SimpleTree4/model/pointsimpletree.h"
#include "SimpleTree4/method/point_cloud_operations/enrichcloud.h"
#include "SimpleTree4/method/point_cloud_operations/stempointdetection.h"
#include "ct_iterator/ct_pointiterator.h"
#include "QDebug"

class ConvertCTtoST
{
    CT_AbstractItemDrawableWithPointCloud* _itemCpy_cloud_in;

    PointCloudS::Ptr _cloud;

    bool _remove;
public:
    ConvertCTtoST(CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in, int knn, bool compute_curvature = true, bool remove = false, bool use_knn = true);

    void convert();

    PointCloudS::Ptr make_two_dimenstional();

    PointCloudS::Ptr get_cloud() const;

private:
    bool _use_knn = true;

    int _knn;

    bool _compute_curvature = true;

    PointCloudS::Ptr reject_points();
};

#endif // CONVERTCTTOST_H
