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

#ifndef STRUCTSTEPPRAMETER2_H
#define STRUCTSTEPPRAMETER2_H

#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_result/ct_resultgroup.h"
#include "ct_itemdrawable/ct_fileheader.h"
#include "ct_itemdrawable/ct_standarditemgroup.h"
#include <SimpleTree4/method/method_coefficients.h>
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include <ct_step/abstract/ct_abstractstep.h>

struct StepParameter2{
    CT_StandardItemGroup* itmgrp;
    CT_ResultGroup* resCpy_res;
    CT_FileHeader* itemCpy_header;
    MethodCoefficients coeff;
    CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in;

   // CT_AbstractStep * step;
    QString    _cloud_out_normals;
    QString    _cloud_out_stem;

    QString     _outCylinderGroupModelName;
    QString     _outCylinderModelName_unimproved;
    QString     _outCylinderModelName_improved_branch_junctions;
    QString     _outCylinderModelName_removed_false_cylinders;
    QString     _outCylinderModelName_removed_improved_by_median;
    QString     _outCylinderModelName_improved_by_fit;
    QString     _outCylinderModelName_improved_by_allometry;
    QString     _outCylinderModelName_improved_by_merge;
    QString     _cluster_grp;
    QString     _clusters;

    int id;

    float cut_height = 0.1f;

    bool is_extracted = true;

    bool use_knn = true;



    QString     _model;
    QString     _coeff;


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

    QString     _outCylinderModel_bo_1;
    QString     _outCylinderModel_bo_2;
    QString     _outCylinderModel_bo_3;
    QString     _outCylinderModel_bo_4;
    QString     _outCylinderModel_bo_5;

    QString     _outCylinderModel_reverse_bo_1;
    QString     _outCylinderModel_reverse_bo_2;
    QString     _outCylinderModel_reverse_bo_3;
    QString     _outCylinderModel_reverse_bo_4;
    QString     _outCylinderModel_reverse_bo_5;

    QString     _outCylinderModel_spherefollowing;
    QString     _outCylinderModel_attractor;
    QString     _outCylinderModel_stem_spherefollowing;
    QString     _outCylinderModel_stem_attractor;
    QString     _outCylinderModel_branch_spherefollowing;
    QString     _outCylinderModel_branch_attractor;
    QString     _outCylinderModel_spherefollowing_allom;
    QString     _outCylinderModel_spherefollowing_no_allom;
};



#endif // STRUCTSTEPPRAMETER2_H
