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

#ifndef ST_WORKERMODELLING_H
#define ST_WORKERMODELLING_H

#include <QRunnable>
#include <QString>
#include <ct_step/abstract/ct_virtualabstractstep.h>
#include <ct_global/ct_repositorymanager.h>
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "SimpleTree4/method/point_cloud_operations/convertcttost.h"
#include "ct_normalcloud/ct_normalcloudstdvector.h"
#include "ct_itemdrawable/ct_pointsattributesnormal.h"
#include "ct_itemdrawable/ct_pointsattributesscalartemplated.h"
#include "ct_itemdrawable/ct_cylinder.h"

#include "item/st_coefficients.h"
#include "item/st_tree.h"
#include "step/modelling/split/st_stepparameter.h"
#include "step/modelling/split/st_modellingthreadpool.h"

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


// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"
#define DEFin_header "header"

//struct StepParameter;
class ST_ModellingThreadPool;

class ST_Workemodelling: public QObject, public QRunnable
{
    Q_OBJECT

public:
    ST_Workemodelling(ST_StepParameter param, QSharedPointer<ST_ModellingThreadPool> mtp);

    void run();

private:

    QSharedPointer<ST_ModellingThreadPool> _mtp;

    void add_cylinder_data(QSharedPointer<Tree> tree, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp);


    ST_StepParameter _param;

    int _knn = 16;

    PointCloudS::Ptr _cloud;

    PointCloudS::Ptr _cloud_noise;

    void split_cloud();

public slots:

    void sent_qstring_worker(QString str);

signals:

     void emit_finished_worker();

     void emit_qstring_worker(QString str);


};

#endif // ST_WORKERMODELLING_H
