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

#ifndef ST_STEP_STD_OUT_MULTITHREADED
#define ST_STEP_STD_OUT_MULTITHREADED


#include <step/simpletreestep.h>
#include <step/filtering/std_out_multithread/threadpool_std_out_multithread.h>
#include <step/filtering/std_out_multithread/std_mult_param.h>

#include "ct_result/model/inModel/ct_inresultmodelgroup.h"
#include "ct_result/model/outModel/ct_outresultmodelgroup.h"
#include "ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_result/model/outModel/ct_outresultmodelgroupcopy.h"
#include "ct_result/ct_resultgroup.h"

#include "ct_iterator/ct_pointiterator.h"
#include "ct_iterator/ct_mutablepointiterator.h"
#include "ct_iterator/ct_resultgroupiterator.h"
#include "ct_iterator/ct_pointiterator.h"

#include "ct_itemdrawable/ct_scene.h"
#include "ct_itemdrawable/ct_pointsattributesnormal.h"
#include "ct_itemdrawable/ct_pointsattributesscalartemplated.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_itemdrawable/tools/iterator/ct_groupiterator.h"
#include "ct_itemdrawable/ct_scene.h"

#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_pointcloudindex/ct_pointcloudindexvector.h"
#include "ct_cloudindex/tools/ct_cloudindexstdvectortmethodimpl.h"
#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_tools/model/ct_autorenamemodels.h"

#include <QtGlobal>
#include <QDebug>
#include <QMessageBox>





class ST_Step_Std_Out_Multithreaded : public CT_AbstractStep, public SimpleTreeStep
{
    Q_OBJECT

public:

    /*! \brief Step constructor
     *
     * Create a new instance of the step
     *
     * \param dataInit Step parameters object
     */
    ST_Step_Std_Out_Multithreaded(CT_StepInitializeData &dataInit);

    ~ST_Step_Std_Out_Multithreaded();

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

    int _k = 16;

    double _std_mult = 3.0;

    QVector<QPair<PointCloudS::Ptr, PointCloudS::Ptr> > _clouds;

    CT_AutoRenameModels    _outScene_filtered;

    int _size_total = 0;

    int _iterations = 1;



public slots:

void
sent_finished_worker(int i);
};
#endif // ST_STEP_STD_OUT_MULTITHREADED
