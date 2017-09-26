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
#include "st_stepupscalecloud.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"



//// Alias for indexing out models
#define DEF_resultOut_translated "translatedResult"
#define DEF_groupOut_pointCloud "translatedGroup"
#define DEF_itemOut_scene "translatedScene"



// Constructor : initialization of parameters
ST_StepUpscaleCloud::ST_StepUpscaleCloud(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _search_radius = 0.03;
    _upsampling_radius = 0.015;
    _upsampling_step_size = 0.015;
    _polynomial_oder = 2;
    _iterations = 0;
}

ST_StepUpscaleCloud::~ST_StepUpscaleCloud()
{
}

// Step description (tooltip of contextual menu)
QString ST_StepUpscaleCloud::getStepDescription() const
{
    return tr("Upscale Cloud.");
}

// Step detailled description
QString ST_StepUpscaleCloud::getStepDetailledDescription() const
{
    return tr("The cloud is upscaled." );
}

// Step URL
QString ST_StepUpscaleCloud::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    //return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepUpscaleCloud::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepUpscaleCloud(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepUpscaleCloud::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("Result_In"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("Grp_In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Tree_Cloud"));

}

// Creation and affiliation of OUT models
void ST_StepUpscaleCloud::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {
        res->addItemModel(DEFin_grp, _outScene_ModelName, new CT_Scene(), tr("Upscaled_Cloud"));
    }
}

// Semi-automatic creation of step parameters DialogBox
void ST_StepUpscaleCloud::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addTitle(tr("You will find help and description about the paramters in the following publication."));
    configDialog->addTitle(tr("This is also the correct citation you should give for scientific publications."));
    configDialog->addEmpty();
    configDialog->addTitle(tr("Alexa, M.; Behr, J.; Cohen-Or, D.; Fleishman, S.; Levin, D.; Silva, C. T. "));
    configDialog->addTitle(tr("<em>Computing and rendering point set surfaces. .</em>"));
    configDialog->addTitle(tr("Visualization and Computer Graphics, IEEE Transactions on <b>2003</b>, 9(1), 3-15. "));
    configDialog->addEmpty();
    configDialog->addDouble(tr("search radius"),"m",0,0.5,3,_search_radius,1,"Radius for the neighborhood search.");
    configDialog->addDouble(tr("upsampling radius"),"m",0,0.2,3,_upsampling_radius,1,"where will points be upsacled.");
    configDialog->addDouble(tr("upsampling step size"),"m",0,0.2,3,_upsampling_step_size,1,"the step size.");
    configDialog->addInt(tr(" polynomial order"),"",0,3,_polynomial_oder,"the order of the fitted polynom.");
    configDialog->addInt(tr(" number of iterations"),"",0,20,_iterations,"the number of iterations.");
}

void ST_StepUpscaleCloud::compute()
{
        QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);

    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);

    while (itCpy_grp.hasNext() && !isStopped())
    {

        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();

        const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        if(itemCpy_cloud_in!=NULL)
        {
            if(_iterations>0)
            {

                create_simple_tree_cloud(itemCpy_cloud_in);
                size_t size = _cloud_out->points.size();
                CT_NMPCIR mpcir = PS_REPOSITORY->createNewPointCloud(size);
                CT_MutablePointIterator it(mpcir);

                for(size_t i = 0; i < size; i++)
                {
                    PointS p = _cloud_out->points.at(i);
                    it.next();
                    CT_Point point;
                    point(0) = p.x;
                    point(1) = p.y;
                    point(2) = p.z;
                    it.replaceCurrentPoint(point);
                }

                CT_Scene* scene = new CT_Scene(_outScene_ModelName.completeName(), resCpy_res, mpcir);
                scene->updateBoundingBox();
                grpCpy_grp->addItemDrawable(scene);
            }
            else
            {
                CT_AbstractPointCloudIndex *origin = (CT_AbstractPointCloudIndex *) itemCpy_cloud_in->getPointCloudIndex();
                CT_PointCloudIndexVector *clone = new CT_PointCloudIndexVector();
                CT_PointIterator itP(origin);

                while (itP.hasNext() && !isStopped())
                {
                    itP.next();
                    size_t index = itP.currentGlobalIndex();
                    clone->addIndex(index);
                }

                CT_Scene* scene
                        = new CT_Scene( _outScene_ModelName.completeName(), resCpy_res);
                scene->setPointCloudIndexRegistered(PS_REPOSITORY->registerPointCloudIndex(clone));
                scene->updateBoundingBox();
                grpCpy_grp->addItemDrawable(scene);
            }
        } else {
            groupsToBeRemoved.push_back(grpCpy_grp);
        }

    }
    _cloud_in.reset(new PointCloudS);
    _cloud_out.reset(new PointCloudS);
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
}



void ST_StepUpscaleCloud::create_simple_tree_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in)
{
    if (itemCpy_cloud_in != NULL)
    {
        const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();
        int knn = 45;
        size_t size = 0;
        size = index->size();
        _cloud_in.reset(new PointCloudS);
        if(size > 0) {
            _cloud_in->points.resize(size);
            _cloud_in->width = size;
            _cloud_in->height = 1;

            size_t i = 0;
            CT_PointIterator it (index);
            while(it.hasNext())
            {
                const CT_Point &internalPoint = it.next().currentPoint();
                PointS p (internalPoint(0),internalPoint(1),internalPoint(2));
                _cloud_in->points[i] = p;
                ++i;
            }
        }
        warning_geo_referenced(_cloud_in);
        EnrichCloud enrich(_cloud_in, knn, 0, true);

        int current = 0;

        _cloud_out.reset(new PointCloudS);
        PointCloudS::Ptr temp_cloud (new PointCloudS);

        temp_cloud = _cloud_in;

        ComputeMeanAndStandardDeviation cm (temp_cloud);
        float voxel_size = cm._mean;
        //        qDebug() << "st_stepupsacle" << voxel_size;


        int size_before = temp_cloud->points.size();
        float min_z = std::numeric_limits<float>::max();
        for(size_t i = 0; i < temp_cloud->points.size(); i++)
        {
            PointS p = temp_cloud->points.at(i);
            if(p.z<min_z)
                min_z = p.z;
        }


        while(current < _iterations)
        {
            PointCloudS::Ptr temp_cloud2 (new PointCloudS);
            current++;

            pcl::MovingLeastSquares<PointS, PointS> mls;
            mls.setInputCloud (temp_cloud);
            mls.setSearchRadius (_search_radius);
            mls.setPolynomialFit (true);
            mls.setPolynomialOrder (_polynomial_oder);
            mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointS, PointS>::SAMPLE_LOCAL_PLANE);
            mls.setUpsamplingRadius (_upsampling_radius);
            mls.setUpsamplingStepSize (_upsampling_step_size);
            mls.process (*temp_cloud2);

            VoxelGridFilter filter(temp_cloud2,voxel_size);
            filter.compute();
            temp_cloud = filter.get_cloud_out();
        }

        PointCloudS::Ptr temp_cloud2 (new PointCloudS);
        pcl::PassThrough<PointS> pass;
        pass.setInputCloud (temp_cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (min_z, 100000.0);
        pass.filter (*temp_cloud2);
        temp_cloud =  temp_cloud2;


        pcl::NormalEstimationOMP<PointS, PointS> ne;
        ne.setInputCloud (temp_cloud);
        pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS> );
        ne.setSearchMethod (tree);
        ne.setKSearch( knn);
        ne.compute (*temp_cloud);

        int size_after = temp_cloud->points.size();

        QString str = "Upscaling successful. Before the cloud had ";
        str.append(QString::number(size_before));
        str.append (" points and afterwards there are ");
        str.append(QString::number(size_after));
        str.append(" points.");
        PS_LOG->addInfoMessage(this, str);
        _cloud_out = temp_cloud;
    }
}



