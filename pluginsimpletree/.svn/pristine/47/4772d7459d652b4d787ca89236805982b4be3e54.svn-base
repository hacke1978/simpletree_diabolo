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

#ifndef ST_STEPEUCLIDEANCLUSTERING_H
#define ST_STEPEUCLIDEANCLUSTERING_H


#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include <step/simpletreestep.h>

#include "SimpleTree4/model/pointsimpletree.h"
#include "SimpleTree4/method/point_cloud_operations/convertcttost.h"

#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_normalcloud/ct_normalcloudstdvector.h"
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




#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_result/model/outModel/ct_outresultmodelgroupcopy.h"
#include "ct_pointcloudindex/ct_pointcloudindexvector.h"
#include "ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h"

// Inclusion of standard result class
#include "ct_result/ct_resultgroup.h"

// Inclusion of used ItemDrawable classes
#include "ct_itemdrawable/ct_scene.h"
#include "ct_iterator/ct_pointiterator.h"





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
#include "ct_cloudindex/tools/ct_cloudindexstdvectortmethodimpl.h"


#include <QtGlobal>
#include <QDebug>
#include <QMessageBox>

// Inclusion of auto-indexation system
#include "ct_tools/model/ct_autorenamemodels.h"




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

class ST_StepEuclideanClustering : public CT_AbstractStep, public SimpleTreeStep
{
    Q_OBJECT

public:

    /*! \brief Step constructor
     *
     * Create a new instance of the step
     *
     * \param dataInit Step parameters object
     */
    ST_StepEuclideanClustering(CT_StepInitializeData &dataInit);

    ~ST_StepEuclideanClustering();

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


    PointCloudS::Ptr _cloud_in;
    std::vector<pcl::PointIndices> _cluster_indices;


    double _distance = 0.05f;
    int _number_clusters = 1;
    int _minPts = 1;



    CT_AutoRenameModels    _outScene_cluster;
    CT_AutoRenameModels    _outScene_grp;

//void recursiveRemoveGroupIfEmpty(CT_AbstractItemGroup *parent, CT_AbstractItemGroup *group) const;

 //   QList<CT_Scene*>* _sceneList;

    //void create_simple_tree_cloud(const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in);

};
#endif // ST_STEPEUCLIDEANCLUSTERING_H
