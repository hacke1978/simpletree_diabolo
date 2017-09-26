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

#include "st_stepradiusoutlierremoval.h"

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
#define DEF_resultOut_translated "extractedResult"
#define DEF_groupOut_pointCloud "extractedGroup"
#define DEF_itemOut_scene "extractedScene"



// Constructor : initialization of parameters
ST_StepRadiusOutlierRemoval::ST_StepRadiusOutlierRemoval(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _num = 10;
    _range = 0.03f;
}

ST_StepRadiusOutlierRemoval::~ST_StepRadiusOutlierRemoval()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepRadiusOutlierRemoval::getStepDescription() const
{
    return tr("Radius Outlier Removal.");
}

// Step detailled description
QString ST_StepRadiusOutlierRemoval::getStepDetailledDescription() const
{
    return tr("Radius Outlier Removal." );
}

// Step URL
QString ST_StepRadiusOutlierRemoval::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    // return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepRadiusOutlierRemoval::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepRadiusOutlierRemoval(dataInit);
}


// Creation and affiliation of IN models
void ST_StepRadiusOutlierRemoval::createInResultModelListProtected()
{



    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("Result_In"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("Grp_In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);

    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Tree_Cloud"));
}

// Creation and affiliation of OUT models
void ST_StepRadiusOutlierRemoval::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {
        res->addItemModel(DEFin_grp, _outScene_ror, new CT_Scene(), tr("Radius outlier filtered cloud"));
    }

}

// Semi-automatic creation of step parameters DialogBox
void ST_StepRadiusOutlierRemoval::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addText("This method iterates over all points in the input cloud. For each point the number of neighbours inside a range <b>r</b> is computed. If a point has less than <b>k</b> neighbours, it is removed from the point cloud.");
    configDialog->addEmpty();
    configDialog->addInt(tr("The minimum number of points in a given range <b>k</b>"), "", 2,1000,_num,"");
    configDialog->addDouble("The range within the points are counted <b>r</b>","m",0.001,0.1,3,_range);
      dialog_simple_tree(configDialog);
}

void ST_StepRadiusOutlierRemoval::compute()
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

        if(itemCpy_cloud_in!=NULL)
        {
            ConvertCTtoST ctst(itemCpy_cloud_in,16);

            ctst.convert();
            _cloud_in = ctst.get_cloud();
            if(_cloud_in->points.size()!=0)
            {
                warning_geo_referenced(_cloud_in);

                PointCloudS::Ptr cloud_filtered (new PointCloudS);


                pcl::RadiusOutlierRemoval<PointS> ror;
                ror.setInputCloud(_cloud_in);
                ror.setRadiusSearch(_range);
                ror.setMinNeighborsInRadius (_num);
                ror.filter (*cloud_filtered);


                const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in->getPointCloudIndex();

                size_t nbPoints = _cloud_in->points.size();
                CT_PointCloudIndexVector *extractedCloud = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
                CT_PointIterator itP(pointCloudIndex);
                pcl::KdTreeFLANN<PointS> kdtree;

                kdtree.setInputCloud (cloud_filtered);
                size_t i = 0;
                while (itP.hasNext() && !isStopped())
                {
                    PointS p = _cloud_in->points.at(i);
                    itP.next();
                    size_t index = itP.currentGlobalIndex();


                    std::vector<int> pointIdxNKNSearch(1);
                    std::vector<float> pointNKNSquaredDistance(1);

                    kdtree.nearestKSearch(p,1,pointIdxNKNSearch,pointNKNSquaredDistance);

                    if ( pointNKNSquaredDistance[0] < 0.0001)
                    {
                        extractedCloud->addIndex(index); // 2) adding kept indices
                    }
                    i++;
                    setProgress( 99.0*i /nbPoints );
                }
                float before = _cloud_in->points.size();
                float after  = cloud_filtered->points.size();
                int percentage = (after/before)*10000.0f;
                float perc = percentage/100.0;

                PS_LOG->addInfoMessage(this, tr("Radius Outlier Removal successfull."));
                QString str;
                str.append(QString::number(perc));
                str.append(" % of points are remaining.");
                PS_LOG->addInfoMessage(this, str);

                if (extractedCloud->size() > 0)
                {
                    CT_Scene* outScene = new CT_Scene(_outScene_ror.completeName(), resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud)); // 3) create scene, registering the pointcloudindex
                    outScene->updateBoundingBox(); // 4) don't forget to update the bounding box, to be fitted to filtered points

                    grpCpy_grp->addItemDrawable(outScene);
                }
            }else {
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
    _cloud_in.reset(new PointCloudS);
}















