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

#ifndef ST_STEPCROPBRANCHORDER
#define ST_STEPCROPBRANCHORDER

#include "ct_result/ct_resultgroup.h"
#include "ct_itemdrawable/ct_fileheader.h"
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
#include "item/st_coefficients.h"
#include "item/st_tree.h"
#include "SimpleTree4/method/method_coefficients.h"
#include "SimpleTree4/method/spherefollowing2.h"
#include "SimpleTree4/model/tree.h"
#include "SimpleTree4/method/point_cloud_operations/convertcttost.h"
#include "SimpleTree4/model/build_tree/improvebyattractor.h"
#include <SimpleTree4/model/build_tree/buildtree.h>
#include <SimpleTree4/model/build_tree/improvebymedian.h>
#include <SimpleTree4/model/build_tree/removefalsecylinders.h>
#include <SimpleTree4/model/build_tree/improvebymerge.h>
#include <SimpleTree4/model/build_tree/reordertree.h>
#include <SimpleTree4/model/build_tree/improvedbyadvancedmedian.h>
#include <SimpleTree4/model/build_tree/improvebypipemodel.h>
#include <SimpleTree4/model/build_tree/improvefit.h>
#include <SimpleTree4/model/build_tree/improvebranchjunctions.h>
#include <SimpleTree4/model/build_tree/improvebyallometry.h>
#include <SimpleTree4/model/build_tree/improvebyallometry_growth_length.h>
#include <SimpleTree4/method/computeallometry.h>
#include <SimpleTree4/method/computeallometry_length.h>
#include <SimpleTree4/export/exporttree.h>
#include "SimpleTree4/export/export.h"
#include <step/simpletreestep.h>

#include <QDir>

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"
#define DEFin_coeff_in "coeff_in"
#define DEFin_header "header"
#define DEFin_tree_in "tree_in"



class ST_StepCropBranchOrder: public CT_AbstractStep, public SimpleTreeStep
{
    Q_OBJECT

public:

    /*! \brief Step constructor
     *
     * Create a new instance of the step
     *
     * \param dataInit Step parameters object
     */
    ST_StepCropBranchOrder(CT_StepInitializeData &dataInit);

    ~ST_StepCropBranchOrder();

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
    virtual ST_StepCropBranchOrder* createNewInstance(CT_StepInitializeData &dataInit);



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


    int _modelled_trees = 0;

    int _number_trees = 0;


    bool _redo_pipe = false;

    QVector<int> generate_crop_bo(QSharedPointer<Tree> tree);

    QSharedPointer<Tree> crop_tree(QSharedPointer<Tree> tree, int bo);


    QStringList _file_name_list;

public:

    CT_AutoRenameModels    _tree_out;
    CT_AutoRenameModels    _coeff_out;
    CT_AutoRenameModels    _tree_out_grp;
    CT_AutoRenameModels    _coeff_out_grp;
    CT_AutoRenameModels    _cloud_out_normals;
    CT_AutoRenameModels    _cloud_out_stem;






    CT_AutoRenameModels     _topologyGroup;
    CT_AutoRenameModels     _stemGroup;
    CT_AutoRenameModels     _stemCylinders;

    int _min_bo = 0;

    int _max_bo = 1;


    CT_AutoRenameModels     _outCylinderGroupModelName;


};

#endif // ST_STEPCROPBRANCHORDER


