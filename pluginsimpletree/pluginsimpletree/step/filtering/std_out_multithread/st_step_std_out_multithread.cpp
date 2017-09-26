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

#include "st_step_std_out_multithread.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"


//// Alias for indexing out models
#define DEF_resultOut_translated "extractedResult"
#define DEF_groupOut_pointCloud "extractedGroup"
#define DEF_itemOut_scene "extractedScene"

#include <QTime>



// Constructor : initialization of parameters
ST_Step_Std_Out_Multithreaded::ST_Step_Std_Out_Multithreaded(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{

}

ST_Step_Std_Out_Multithreaded::~ST_Step_Std_Out_Multithreaded()
{

}

// Step description (tooltip of contextual menu)
QString ST_Step_Std_Out_Multithreaded::getStepDescription() const
{
    return tr("Statistical Outlier Filter - multithreaded.");
}

// Step detailled description
QString ST_Step_Std_Out_Multithreaded::getStepDetailledDescription() const
{
    return tr("Statistical Outlier Filter - multithreaded." );
}

// Step URL
QString ST_Step_Std_Out_Multithreaded::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
}

// Step copy method
CT_VirtualAbstractStep* ST_Step_Std_Out_Multithreaded::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_Step_Std_Out_Multithreaded(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_Step_Std_Out_Multithreaded::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("Result_In"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("Grp_In"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Tree_Cloud"));
}

// Creation and affiliation of OUT models
void ST_Step_Std_Out_Multithreaded::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {
        res->addItemModel(DEFin_grp, _outScene_filtered, new CT_Scene(), tr("Statistical_Outlier_Filtered_Cloud"));
    }
}

// Semi-automatic creation of step parameters DialogBox
void ST_Step_Std_Out_Multithreaded::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addText("This method iterates over all points in the input cloud. For each point the average distance to its <b>k</b>");
    configDialog->addText(" nearest neighbours is computed.");
    configDialog->addText(" Mean <b>m</b> and standard deviation <b>sd</b> are calculated for those distances.");
    configDialog->addText(" If a point’s average distance is larger than <b>m</b> + <b>sdMult</b> x <b>sd</b> or smaller");
    configDialog->addText(" than <b>m</b> - <b>sdMult</b> x <b>sd</b>, it is marked as an outlier and removed from the cloud.");
    configDialog->addEmpty();




    configDialog->addInt(tr("The number of neighbors for which the average distance is analysed "), "k", 2,1000,_k,"");
    configDialog->addDouble("The multiplier for the standard deviation of the average distance ","stMult",0.1,10.0,3,_std_mult);

    configDialog->addEmpty();
      configDialog->addInt(tr("The number of iterations "), "k", 1,1000,_iterations,"");
    dialog_simple_tree(configDialog);
}

void ST_Step_Std_Out_Multithreaded::compute()
{
    QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    _clouds.clear();
    _size_total = 0;
    QVector<St_step_std_out_param> params;


    QTime time;
    time.start();

    while (itCpy_grp.hasNext() && !isStopped())
    {

        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in
                = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        if(itemCpy_cloud_in!=NULL)
        {
            St_step_std_out_param  param;
            param.knn = _k;
            param.iterations = _iterations;
            param.std_mult = _std_mult;
            param.grpCpy_grp = grpCpy_grp;
            param.resCpy_res = resCpy_res;
            param._cloud_out_name = _outScene_filtered.completeName();



            const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();
            size_t size = index->size();
            PointCloudS::Ptr cloud (new PointCloudS);
            cloud->width = (int) size;
            cloud->height = 1;
            if(size > 0) {
                cloud->points.resize(size);
                size_t i = 0;
                CT_PointIterator it (index);
                while(it.hasNext())
                {
                    const CT_Point &internalPoint = it.next().currentPoint();
                    PointS p(internalPoint(0),internalPoint(1),internalPoint(2));
                    cloud->points[i] = p;
                    ++i;
                }
            }


                    param.cloud = cloud;
            param.itemCpy_cloud_in = itemCpy_cloud_in;
            params.push_back(param);
        }
        _size_total ++;
    }
//    qDebug () << " std out multi preprocessing" << time.restart();
    QSharedPointer<ThreadPoool_std_out_multithread> threadpool(new ThreadPoool_std_out_multithread(params));
    QObject::connect(threadpool.data(), SIGNAL(emit_finished_worker(int)), this, SLOT(sent_finished_worker(int)) );
    threadpool->compute();

    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
    params.clear();
}

void ST_Step_Std_Out_Multithreaded::sent_finished_worker(int i)
{
    int percentage = (((float)i)/((float)_size_total)*100.0f);
    setProgress(percentage);
}















