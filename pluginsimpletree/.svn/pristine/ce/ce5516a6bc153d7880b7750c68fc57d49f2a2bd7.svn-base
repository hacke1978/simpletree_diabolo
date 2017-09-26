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

#ifndef WORKERMODELLING2_H
#define WORKERMODELLING2_H

#include <QRunnable>
#include <QString>
#include <ct_step/abstract/ct_virtualabstractstep.h>
//#include <step/modelling/new/structstepprameter2.h>
#include <ct_global/ct_repositorymanager.h>
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "SimpleTree4/method/point_cloud_operations/convertcttost.h"
#include "ct_normalcloud/ct_normalcloudstdvector.h"
#include "ct_itemdrawable/ct_pointsattributesnormal.h"
#include "ct_itemdrawable/ct_pointsattributesscalartemplated.h"
#include "step/modelling/new/st_stepcompletefoldermodelling2.h"
#include "step/modelling/new/predictstartparameters2.h"
#include "SimpleTree4/method/spherefollowingrecursive.h"
#include "SimpleTree4/export/exporttree.h"
#include <SimpleTree4/method/geometrical_operations/stem_taper.h>
#include <SimpleTree4/method/point_cloud_operations/extractcloudnearmodel.h>
#include "step/simpletreestep.h"

#include <pcl/console/time.h>

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"
#define DEFin_header "header"

//struct StepParameter;
class ModellingThreadPool2;
class ST_StepCompleteFolderModelling2;
class WorkerModelling2: public QObject, public QRunnable, public SimpleTreeStep
{
    Q_OBJECT

public:
    WorkerModelling2(StepParameter2 param, QSharedPointer<ModellingThreadPool2> mtp);

    void run();

    static bool comparefunc (const QPair<PointCloudS::Ptr,float>& a, const QPair<PointCloudS::Ptr,float>& b) {
        return a.second < b.second;
    }

private:

    Eigen::Vector3f get_vec(PointCloudS::Ptr cloud_in, QVector<pcl::ModelCoefficients> tree_coeff);


    QSharedPointer<ModellingThreadPool2> _mtp;

    void add_cylinder_data(Tree tree, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp, QString string);

    CT_TTreeGroup *constructTopology(const CT_AbstractResult *res_r, QSharedPointer<Tree> tree, QString string);

    void constructTopologyRecursively(const CT_AbstractResult *res_r, CT_TNodeGroup *parent,
                                      QSharedPointer<Segment>  segment,
                                      QSharedPointer<Tree> tree, QString string);

    void setCylinders(const CT_AbstractResult *res_r, CT_TNodeGroup *root,
                      QSharedPointer<Segment> segment,
                      QSharedPointer<Tree> tree, QString string);

    QVector<pcl::ModelCoefficients>  iterative_run(PointCloudS::Ptr remaining_cloud, QVector<pcl::ModelCoefficients> cylinder_coeff, MethodCoefficients coeff);

    QVector<pcl::ModelCoefficients> connect_iterative_cylinders(QVector<pcl::ModelCoefficients> cylinder_coeff_large, QVector<pcl::ModelCoefficients> cylinder_coeff_iterative);

    StepParameter2 _param;

    int _knn = 16;

    size_t _number_of_cluster_iterative = 5;

    void enrich_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp);

    PointCloudS::Ptr _cloud;

    PointCloudS::Ptr _cloud_noise;

    void split_cloud();

    float _percent = 0.02;

public slots:

    void sent_qstring_worker(QString str);

signals:

    void emit_number_cylinders(int number);

     void emit_finished_worker();

     void emit_qstring_worker(QString str);


};

#endif // WORKERMODELLING2_H
