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

#ifndef ST_STEPFITMULTIPLEDBH_H
#define ST_STEPFITMULTIPLEDBH_H

#include "SimpleTree4/model/pointsimpletree.h"
#include "SimpleTree4/method/point_cloud_operations/enrichcloud.h"
#include "SimpleTree4/method/point_cloud_operations/stempointdetection.h"

#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_normalcloud/ct_normalcloudstdvector.h"
#include "ct_itemdrawable/ct_pointsattributesnormal.h"
#include "ct_itemdrawable/ct_pointsattributesscalartemplated.h"
#include "ct_tools/model/ct_autorenamemodels.h"
#include "ct_itemdrawable/ct_scene.h"
#include "ct_result/model/inModel/ct_inresultmodelgroup.h"
#include "ct_result/model/outModel/ct_outresultmodelgroup.h"
#include "ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h"

#include "ct_iterator/ct_pointiterator.h"
#include "ct_iterator/ct_mutablepointiterator.h"

#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_itemdrawable/tools/iterator/ct_groupiterator.h"
#include "ct_iterator/ct_resultgroupiterator.h"

#include <step/simpletreestep.h>


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>




#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_itemdrawable/ct_pointsattributesnormal.h"
#include "ct_itemdrawable/ct_pointsattributesscalartemplated.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_itemdrawable/tools/iterator/ct_groupiterator.h"
#include "ct_result/ct_resultgroup.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_normalcloud/ct_normalcloudstdvector.h"
#include "ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h"
#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_iterator/ct_pointiterator.h"
#include "ct_itemdrawable/ct_scene.h"
#include "SimpleTree4/model/tree.h"

#include <stdlib.h>

#include <QtGlobal>
#include <QDebug>
#include <QMessageBox>

// Inclusion of auto-indexation system
#include "ct_tools/model/ct_autorenamemodels.h"


#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_result/model/outModel/ct_outresultmodelgroupcopy.h"
#include "ct_itemdrawable/ct_cylinder.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h"



/*!
 * \class ST_StepUpscaleCloud
 * \ingroup Steps_ST
 * \brief <b>detects stem points.</b>
 *
 * No detailled description for this step
 *
 * \param _param1
 *
 */

class ST_StepFITMultipleDBH : public CT_AbstractStep
{
    Q_OBJECT

public:

    /*! \brief Step constructor
     *
     * Create a new instance of the step
     *
     * \param dataInit Step parameters object
     */
    ST_StepFITMultipleDBH(CT_StepInitializeData &dataInit);

    ~ST_StepFITMultipleDBH();

    /*! \brief Step description
     *
     * Return a description of the step function
     */
    QString getStepDescription() const;

    /*! \brief Step detailled description
     *
     * Return a detailled description of the step function
     */
    QString getStepDetailledDescription() const;

    /*! \brief Step URL
     *
     * Return a URL of a wiki for this step
     */
    QString getStepURL() const;

    /*! \brief Step copy
     *
     * Step copy, used when a step is added by step contextual menu
     */
    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &dataInit);

protected:


    void create_simple_tree_cloud(const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in);

    void enrich_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp);

    /*! \brief Input results specification
     *
     * Specification of input results models needed by the step (IN)
     */
    void createInResultModelListProtected();

    /*! \brief Parameters DialogBox
     *
     * DialogBox asking for step parameters
     */
    void createPostConfigurationDialog();

    /*! \brief Output results specification
     *
     * Specification of output results models created by the step (OUT)
     */
    void createOutResultModelListProtected();

    /*! \brief Algorithm of the step
     *
     * Step computation, using input results, and creating output results
     */
    void compute();


private:

    float _get_min_z();

    PointCloudS::Ptr extract_slice(float length, float min_z);

    double _cut_height = 1.3f;
    double _min_z_height = 0.60f;

    PointCloudS::Ptr _cloud_in;

    int _nmb_inliers = 100;
    int _iterations = 100;
    double _max_dist = 0.01;

    // Declaration of autoRenames Variables (groups or items added to In models copies)
    CT_AutoRenameModels    _cylinder;
    CT_AutoRenameModels    _outCylinderGroupModelName;
    CT_AutoRenameModels    _outCylinderGroupModelName2;
    CT_AutoRenameModels    _outCylinderModelName;

    CT_AutoRenameModels    _outCylinderModelName_r5;
    CT_AutoRenameModels    _outCylinderModelName_r10;
    CT_AutoRenameModels    _outCylinderModelName_r15;
    CT_AutoRenameModels    _outCylinderModelName_r20;
    CT_AutoRenameModels    _outCylinderModelName_r25;
    CT_AutoRenameModels    _outCylinderModelName_r30;
    CT_AutoRenameModels    _outCylinderModelName_r35;
    CT_AutoRenameModels    _outCylinderModelName_r40;
    CT_AutoRenameModels    _outCylinderModelName_r45;
    CT_AutoRenameModels    _outCylinderModelName_r50;

    CT_AutoRenameModels    _outCylinderModelName_m5;
    CT_AutoRenameModels    _outCylinderModelName_m10;
    CT_AutoRenameModels    _outCylinderModelName_m15;
    CT_AutoRenameModels    _outCylinderModelName_m20;
    CT_AutoRenameModels    _outCylinderModelName_m25;
    CT_AutoRenameModels    _outCylinderModelName_m30;
    CT_AutoRenameModels    _outCylinderModelName_m35;
    CT_AutoRenameModels    _outCylinderModelName_m40;
    CT_AutoRenameModels    _outCylinderModelName_m45;
    CT_AutoRenameModels    _outCylinderModelName_m50;



//    QList<CT_Scene*>* _sceneList;

    //void create_simple_tree_cloud(const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in);

};
#endif // ST_STEPFITMULTIPLEDBH_H
