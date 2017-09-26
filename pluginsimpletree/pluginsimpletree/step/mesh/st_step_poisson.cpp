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
#include "st_step_poisson.h"


#include <pcl/surface/poisson.h>
#include <pcl/surface/impl/poisson.hpp>



ST_StepPoissonReconstruction::ST_StepPoissonReconstruction(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _depth = 11;
    // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

ST_StepPoissonReconstruction::~ST_StepPoissonReconstruction()
{
}


// Step description (tooltip of contextual menu)
QString ST_StepPoissonReconstruction::getStepDescription() const
{
    return tr("Poisson reconstruction.");
}

// Step detailled description
QString ST_StepPoissonReconstruction::getStepDetailledDescription() const
{
    return tr("Poisson reconstruction." );
}

// Step URL
QString ST_StepPoissonReconstruction::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    //return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
ST_StepPoissonReconstruction* ST_StepPoissonReconstruction::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepPoissonReconstruction(dataInit);
}





void ST_StepPoissonReconstruction::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();

    configDialog->addTitle(tr("Poisson reconstruction, do not reccommend to be use for now. Not explored yet. Here is the citation:"));
    configDialog->addTitle(tr(" Michael Kazhdan, Matthew Bolitho, Hugues Hoppe"));
    configDialog->addTitle(tr("Poisson surface reconstruction, SGP, 2006, "));
    configDialog->addTitle(tr("Proceedings of the fourth Eurographics symposium on Geometry processing "));
    configDialog->addTitle(tr("This is also the correct citation you should give for scientific publications."));
    configDialog->addEmpty();
    dialog_simple_tree(configDialog);
}


// Creation and affiliation of IN models
void ST_StepPoissonReconstruction::createInResultModelListProtected()
{   
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resIn_res->addItemModel(DEFin_grp, DEFin_header, CT_FileHeader::staticGetType(), tr("File Header"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Cloud In"));
}

// Creation and affiliation of OUT models
void ST_StepPoissonReconstruction::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addItemModel(DEFin_grp, _mesh_out, new CT_MeshModel(), "Mesh");
    }
}

void ST_StepPoissonReconstruction::compute()
{
    QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator grp_it(resCpy_res, this, DEFin_grp);

    while (grp_it.hasNext() && !isStopped())
    {
        CT_StandardItemGroup * item_grp = (CT_StandardItemGroup *) grp_it.next();
        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in =
                (CT_AbstractItemDrawableWithPointCloud*)item_grp->firstItemByINModelName(this, DEFin_cloud_in);
        if(itemCpy_cloud_in!=NULL)
        {
            PointCloudS::Ptr cloud_in = pcl_CT_to_PCL_cloud(itemCpy_cloud_in,this,16,false,false);
            if(cloud_in->points.size()!=0)
            {
                warning_geo_referenced(cloud_in);
                pcl::NormalEstimationOMP<PointS, PointS> ne;
                ne.setNumberOfThreads (8);
                ne.setInputCloud (cloud_in);
                ne.setKSearch(15);
                Eigen::Vector4f centroid;
                compute3DCentroid (*cloud_in, centroid);
                ne.setViewPoint (centroid[0], centroid[1], centroid[2]);
                ne.compute (*cloud_in);
                for (size_t i = 0; i < cloud_in->size (); ++i)
                {
                    cloud_in->points[i].normal_x *= -1;
                    cloud_in->points[i].normal_y *= -1;
                    cloud_in->points[i].normal_z *= -1;
                }

                PointS p = cloud_in->points.at(0);
                _depth = 11;
                pcl::Poisson<PointS> poisson;
                poisson.setScale(1.01f);
                poisson.setDepth(_depth);
                poisson.setInputCloud(cloud_in);
                pcl::PolygonMesh mesh;
                poisson.reconstruct(mesh);
//                pcl::io::savePLYFile("D:/poisson.ply", mesh);
                pcl::io::saveOBJFile ("D:/poisson.obj", mesh);
            } else {
                groupsToBeRemoved.push_back(item_grp);
            }
        } else {
            groupsToBeRemoved.push_back(item_grp);
        }


    }
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
}





