/****************************************************************************

 Copyright (C) 2016-2017 Jan Hackenberg
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

#ifndef STD_MULT_PARAM_H
#define STD_MULT_PARAM_H
#include <QString>
#include "ct_result/ct_resultgroup.h"
#include "ct_itemdrawable/ct_standarditemgroup.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "SimpleTree4/model/pointsimpletree.h"

struct St_step_std_out_param{
    float std_mult;
    int knn;
    int iterations;
    QString _cloud_out_name;
    CT_StandardItemGroup* grpCpy_grp;
    CT_ResultGroup* resCpy_res;
            CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in;
            PointCloudS::Ptr cloud;
};
#endif // STD_MULT_PARAM_H
