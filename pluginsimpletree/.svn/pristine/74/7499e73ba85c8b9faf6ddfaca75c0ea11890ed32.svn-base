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

#include "st_stepsplitbyheight.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"

//#define DEF_resultIn_inputResult "inputResult"
//#define DEF_groupIn_inputScene "inputGroup"
//#define DEF_itemIn_scene "inputScene"


#define DEFin_normals_in "no_in"
#define DEFin_stem_in "stem_in"


//// Alias for indexing out models
//#define DEF_resultOut_translated "extractedResult"
//#define DEF_groupOut_pointCloud "extractedGroup"
//#define DEF_itemOut_scene "extractedScene"


#include "opencv2/opencv.hpp"
#include "opencv2/ml.hpp"

//using namespace cv;
//using namespace cv::ml;
//using namespace std;


// Constructor : initialization of parameters
ST_StepSplitByHeight::ST_StepSplitByHeight(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _cut_height = 1;
}

ST_StepSplitByHeight::~ST_StepSplitByHeight()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepSplitByHeight::getStepDescription() const
{
    return tr("Splits the cloud by a vertical cut.");
}

// Step detailled description
QString ST_StepSplitByHeight::getStepDetailledDescription() const
{
    return tr("Splits the cloud by a vertical cut. The minimum z-coordinate is computed and a user given height is added to this minimum z-coordinate. "
              "The cloud is split into a lower and an upper part according to the calculated height." );
}

// Step URL
QString ST_StepSplitByHeight::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
}

// Step copy method
CT_VirtualAbstractStep* ST_StepSplitByHeight::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepSplitByHeight(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepSplitByHeight::createInResultModelListProtected()
{


    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);

    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("cloud to be split vertically"));
}

// Creation and affiliation of OUT models
void ST_StepSplitByHeight::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {

        res->addItemModel(DEFin_grp, _outScene_lower, new CT_Scene(), tr("cloud lower part"));
        res->addItemModel(DEFin_grp, _outScene_upper, new CT_Scene(), tr("cloud upper part"));
    }
}

// Semi-automatic creation of step parameters DialogBox
void ST_StepSplitByHeight::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addDouble( tr("Split height"), "",  0.01,100,3,_cut_height);
    configDialog->addEmpty();
    dialog_simple_tree(configDialog);
}

void ST_StepSplitByHeight::compute()
{
    QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);

    while (itCpy_grp.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in
                = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        if(itemCpy_cloud_in!=0)
        {
            const CT_AbstractPointCloudIndex *pointCloudIndex_b = itemCpy_cloud_in->getPointCloudIndex();
            if(pointCloudIndex_b->size()!=0)
            {
                double min_point_cloud_height =std::numeric_limits<double>::max();
                CT_PointIterator itP_a(pointCloudIndex_b);
                while (itP_a.hasNext() && !isStopped())
                {
                    itP_a.next();

                    CT_Point point =  itP_a.currentPoint();

                    if(point[2] < min_point_cloud_height)
                    {
                        min_point_cloud_height = point[2];
                    }
                }
                double slice_height = min_point_cloud_height + _cut_height;
                size_t size_original = pointCloudIndex_b->size();
                CT_PointCloudIndexVector *extracted_cloud_lower = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
                CT_PointCloudIndexVector *extracted_cloud_upper = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
                CT_PointIterator itP(pointCloudIndex_b);
                while (itP.hasNext() && !isStopped())
                {
                    itP.next();
                    CT_Point point =  itP.currentPoint();
                    size_t index = itP.currentGlobalIndex();
                    if(point[2] <= slice_height) {
                        extracted_cloud_lower->addIndex(index);
                    } else {
                        extracted_cloud_upper->addIndex(index);
                    }
                }

                QString str = tr("Splitted a cloud of size ");
                str.append(QString::number(size_original));
                str.append(tr(" points vertically in the relative height of "));
                str.append(QString::number(slice_height));
                str.append("m.");
                PS_LOG->addInfoMessage(this, str);

                if (extracted_cloud_lower->size() > 0)
                {
                    CT_Scene* outScene = new CT_Scene(_outScene_lower.completeName(), resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extracted_cloud_lower));
                    outScene->updateBoundingBox();
                    grpCpy_grp->addItemDrawable(outScene);

                    QString str;
                    str.append(QString::number(extracted_cloud_lower->size()));
                    str.append(tr(" points are in the lower cloud."));
                    PS_LOG->addInfoMessage(this, str);
                }
                if(extracted_cloud_upper->size() > 0)
                {
                    CT_Scene* outScene = new CT_Scene(_outScene_upper.completeName(), resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extracted_cloud_upper));
                    outScene->updateBoundingBox();
                    grpCpy_grp->addItemDrawable(outScene);

                    QString str;
                    str.append(QString::number(extracted_cloud_upper->size()));
                    str.append(tr(" points are in the upper cloud."));
                    PS_LOG->addInfoMessage(this, str);
                }
            } else {
                groupsToBeRemoved.push_back(grpCpy_grp);
            }
        } else {
            groupsToBeRemoved.push_back(grpCpy_grp);
        }
    }
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
}















