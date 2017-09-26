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

#ifndef ST_STEPCOMPLETEFOLDERMODELLING2
#define ST_STEPCOMPLETEFOLDERMODELLING2

#include "structstepprameter2.h"
#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_result/model/outModel/ct_outresultmodelgroupcopy.h"
#include "ct_itemdrawable/ct_cylinder.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h"
#include "ct_itemdrawable/ct_pointsattributesscalartemplated.h"
#include "ct_itemdrawable/ct_pointsattributesnormal.h"
#include "ct_itemdrawable/ct_ttreegroup.h"
#include "ct_itemdrawable/ct_cylinder.h"
#include <pcl/console/print.h>
#include <step/modelling/new/modellingthreadpool2.h>
#include "item/st_coefficients.h"
#include "item/st_tree.h"
#include "ct_itemdrawable/abstract/ct_abstractareashape2d.h"

#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_itemdrawable/ct_referencepoint.h"
#include "ct_itemdrawable/ct_image2d.h"
#include <step/simpletreestep.h>
#include "SimpleTree4/model/pointsimpletree.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"
#define DEFin_header "header"
#define DEFin_cluster_grp "cluster_grp"
#define DEFin_clusters "clusters"
#define DEFin_DTM "DTM"
#define DEF_SearchInMNTResult   "mntres"
#define DEF_SearchInMNTGroup    "mntgrp"
#define DEF_SearchInHeaderResult   "headerres"


class ST_StepCompleteFolderModelling2: public CT_AbstractStep, public SimpleTreeStep
{
    Q_OBJECT

public:

    float compute_height_above_DTM(CT_Image2D<float>* mnt, CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in);

    /*! \brief Step constructor
     *
     * Create a new instance of the step
     *
     * \param dataInit Step parameters object
     */
    ST_StepCompleteFolderModelling2(CT_StepInitializeData &dataInit);

    ~ST_StepCompleteFolderModelling2();

    /*! \brief Step description
     *
     * Return a description of the step function
     */
    virtual QString getStepDescription() const;

    /*! \brief Step detailled description
     *
     * Return a detailled description of the step function
     */
    virtual QString getStepDetailledDescription() const;

    /*! \brief Step URL
     *
     * Return a URL of a wiki for this step
     */
    virtual QString getStepURL() const;

    /*! \brief Step copy
     *
     * Step copy, used when a step is added by step contextual menu
     */
    virtual ST_StepCompleteFolderModelling2* createNewInstance(CT_StepInitializeData &dataInit);



protected:

    /*! \brief Input results specification
     *
     * Specification of input results models needed by the step (IN)
     */
    void createInResultModelListProtected();


    /*! \brief Output results specification
     *
     * Specification of output results models created by the step (OUT)
     */
    void createOutResultModelListProtected();

    /*! \brief Parameters DialogBox
     *
     * DialogBox asking for step parameters
     */
    virtual void createPostConfigurationDialog();



    /*! \brief Algorithm of the step
     *
     * Step computation, using input results, and creating output results
     */
    virtual void compute();







private:

    bool _use_knn = true;

    QVector<StepParameter2> _step_parameters;

    int _knn = 16;

    QStringList _file_name_list;

    int _modelled_trees = 0;

    int _number_trees = 0;

    double _cut_height;

    Eigen::Affine3f _transform = Eigen::Affine3f::Identity();

    Eigen::Vector3f _rotation_axis;

    float _rotation_angle;

    bool _use_simple_error_term_sphere = true;
    bool _use_simple_error_term_fit = true;
    bool _use_remove_false_cylinders = true;
    bool _use_improve_by_median = true;
    bool _use_stem_taper = true;
    bool _use_improve_branch_junctions = true;
    bool _use_mlesac = true;

    bool _use_median_filter_later = true;
    bool _use_pipe_check = true;


public:

    CT_AutoRenameModels    _tree_out;
    CT_AutoRenameModels    _coeff_out;
    CT_AutoRenameModels    _tree_out_grp;
    CT_AutoRenameModels    _coeff_out_grp;
    CT_AutoRenameModels    _cloud_out_normals;
    CT_AutoRenameModels    _cloud_out_stem;

    CT_AutoRenameModels     _outCylinderGroupModelName;
    CT_AutoRenameModels     _outCylinderModelName_unimproved;
    CT_AutoRenameModels     _outCylinderModelName_improved_branch_junctions;
    CT_AutoRenameModels     _outCylinderModelName_removed_false_cylinders;
    CT_AutoRenameModels     _outCylinderModelName_removed_improved_by_median;
    CT_AutoRenameModels     _outCylinderModelName_improved_by_fit;
    CT_AutoRenameModels     _outCylinderModelName_improved_by_allometry;
    CT_AutoRenameModels     _outCylinderModelName_improved_by_merge;


    CT_AutoRenameModels     _outCylinderModel_bo_1;
    CT_AutoRenameModels     _outCylinderModel_bo_2;
    CT_AutoRenameModels     _outCylinderModel_bo_3;
    CT_AutoRenameModels     _outCylinderModel_bo_4;
    CT_AutoRenameModels     _outCylinderModel_bo_5;

    CT_AutoRenameModels     _outCylinderModel_reverse_bo_1;
    CT_AutoRenameModels     _outCylinderModel_reverse_bo_2;
    CT_AutoRenameModels     _outCylinderModel_reverse_bo_3;
    CT_AutoRenameModels     _outCylinderModel_reverse_bo_4;
    CT_AutoRenameModels     _outCylinderModel_reverse_bo_5;

    CT_AutoRenameModels     _outCylinderModel_spherefollowing;
    CT_AutoRenameModels     _outCylinderModel_attractor;
    CT_AutoRenameModels     _outCylinderModel_stem_spherefollowing;
    CT_AutoRenameModels     _outCylinderModel_stem_attractor;
    CT_AutoRenameModels     _outCylinderModel_branch_spherefollowing;
    CT_AutoRenameModels     _outCylinderModel_branch_attractor;
    CT_AutoRenameModels     _outCylinderModel_spherefollowing_allom;
    CT_AutoRenameModels     _outCylinderModel_spherefollowing_no_allom;

    CT_AutoRenameModels _branchIDModelName;
    CT_AutoRenameModels _branchOrderModelName;
    CT_AutoRenameModels _segmentIDModelName;
    CT_AutoRenameModels _parentSegmentIDModelName;
    CT_AutoRenameModels _growthVolumeModelName;
    CT_AutoRenameModels _tree_species;
    CT_AutoRenameModels _tree_id;
    CT_AutoRenameModels _detection_type;
    CT_AutoRenameModels _improvement_type;

    CT_AutoRenameModels     _topologyGroup;
    CT_AutoRenameModels     _stemGroup;
    CT_AutoRenameModels     _stemCylinders;

    CT_AutoRenameModels     _cluster_grp;
    CT_AutoRenameModels     _clusters;


public slots:

    void sent_qstring_step(QString str);

    void sent_finished_step();

    void sent_timings(QStringList timings);

    void plot_number_cylinders(int number);




};

#endif // ST_STEPCOMPLETEFOLDERMODELLING2

