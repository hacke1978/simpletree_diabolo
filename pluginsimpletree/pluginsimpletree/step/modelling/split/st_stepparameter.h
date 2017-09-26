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

#ifndef ST_STEPPARAMETER_H
#define ST_STEPPARAMETER_H

#include <QMap>
#include <SimpleTree4/import/readcsv.h>
#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_result/ct_resultgroup.h"
#include "ct_itemdrawable/ct_fileheader.h"
#include "ct_itemdrawable/ct_standarditemgroup.h"
#include <SimpleTree4/method/method_coefficients.h>
#include "item/st_coefficients.h"
#include "item/st_tree.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include <ct_step/abstract/ct_abstractstep.h>

struct ST_StepParameter{
    CT_StandardItemGroup* grpCpy_grp;
    CT_ResultGroup* resCpy_res;
    CT_FileHeader* itemCpy_header;

    ST_Coefficients* coeff_in;
    ST_Tree * tree_in;

    CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in;



    QString     _cylinder_grp;
    QString     _cylinders;

    QString     _model;
    QString     _coeff;
    QString     _tree_out;


    QString _branchIDModelName;
    QString _branchOrderModelName;
    QString _segmentIDModelName;
    QString _parentSegmentIDModelName;
    QString _growthVolumeModelName;
    QString _tree_species;
    QString _tree_id;
    QString _detection_type;
    QString _improvement_type;


    QString     _topologyGroup;
    QString     _stemGroup;
    QString     _stemCylinders;

};



#endif // ST_STEPPARAMETER_H
