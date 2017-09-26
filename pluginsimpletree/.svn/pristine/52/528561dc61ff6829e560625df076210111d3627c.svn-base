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

#ifndef WORKERTHREAD_STD_OUT_MULTITHREAD
#define WORKERTHREAD_STD_OUT_MULTITHREAD

#include <QRunnable>
#include <QPair>
#include <QString>

#include <ct_step/abstract/ct_virtualabstractstep.h>
#include <ct_global/ct_repositorymanager.h>
#include "ct_normalcloud/ct_normalcloudstdvector.h"
#include "ct_itemdrawable/ct_pointsattributesnormal.h"
#include "ct_itemdrawable/ct_pointsattributesscalartemplated.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"

#include "step/simpletreestep.h"
#include "step/modelling/new/st_stepcompletefoldermodelling2.h"
#include <pcl/console/time.h>

// Alias for indexing models
#define DEFin_res_in "res_in"
#define DEFin_grp_in "grp_in"
#define DEFin_cloud_in "cloud_in"
#define DEFin_cloud_out "cloud_out"
#define DEFin_res_out "res_out"
#define DEFin_grp_out "grp_out"



#include <QThreadPool>
#include <QStringList>
#include <QEnableSharedFromThis>
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

class ThreadPoool_std_out_multithread;


class Worker_Std_out_multithread:  public QObject, public QRunnable
{
    Q_OBJECT

public:
    Worker_Std_out_multithread(St_step_std_out_param params, QSharedPointer<ThreadPoool_std_out_multithread> tp_std_out);

    void run();

private:

   St_step_std_out_param _param;

    QSharedPointer<ThreadPoool_std_out_multithread> _tp_std_out;

    int _i;

    int _knn = 16;

    float _std_mult = 2;

    int _iterations = 1;


public slots:


signals:

    void emit_result(int i);

     void emit_finished_worker();

     void emit_qstring_worker(QString str);


};

#endif // WORKERTHREAD_STD_OUT_MULTITHREAD
