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
#include "st_stepenrichcloudwithcurvature.h"



// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"

#define DEFin_res2 "res2"
#define DEFin_grp2 "grp2"
#define DEFin_cloud_in2 "cloud_in2"



// Constructor : initialization of parameters
ST_StepEnrichCloudWithCurvature::ST_StepEnrichCloudWithCurvature(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _range = 0.03f;
    _use_knn = true;
    _knn = 15;
}

// Step description (tooltip of contextual menu)
QString ST_StepEnrichCloudWithCurvature::getStepDescription() const
{
    return tr("Computes normal and curvature information for a cloud.");
}

// Step detailled description
QString ST_StepEnrichCloudWithCurvature::getStepDetailledDescription() const
{
    return tr("The normals are computed in a multithreading routine if multiple cores are available. In addition the normalized eigenvalues of the three eigenvectors are returned." );
}

// Step URL
QString ST_StepEnrichCloudWithCurvature::getStepURL() const
{
    //return tr("STEP URL HERE");
    return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepEnrichCloudWithCurvature::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepEnrichCloudWithCurvature(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepEnrichCloudWithCurvature::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("cloud_in"));

}

// Creation and affiliation of OUT models
void ST_StepEnrichCloudWithCurvature::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_normals, new CT_PointsAttributesNormal(),tr("Normals"));
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_curvature, new CT_PointsAttributesScalarTemplated<float>(),tr("Curvature"));
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_eigen1, new CT_PointsAttributesScalarTemplated<float>(),tr("Eigenvalue1"));
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_eigen2, new CT_PointsAttributesScalarTemplated<float>(),tr("Eigenvalue2"));
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_eigen3, new CT_PointsAttributesScalarTemplated<float>(),tr("Eigenvalue3"));
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_stem, new CT_PointsAttributesScalarTemplated<float>(),tr("Stem flag"));
    }
}

// Semi-automatic creation of step parameters DialogBox
void ST_StepEnrichCloudWithCurvature::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();



    configDialog->addBool("use knn","","",_use_knn,"Uncheck only if you want to use a range search.");
    configDialog->addEmpty();
    configDialog->addDouble(tr("radius"), "m", 0, 0.1,3,  _range, 1, "All points in range are considered to influence the normals and curvature.");
    configDialog->addInt(tr("knn"), "", 3, 100, _knn,"The knn number of points are considered to influence the normals and curvature.");

}

void ST_StepEnrichCloudWithCurvature::compute()
{
    // DONT'T FORGET TO ADD THIS STEP TO THE PLUGINMANAGER !!!!!

    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);

    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    while (itCpy_grp.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        
        const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        create_simple_tree_cloud(itemCpy_cloud_in);
        enrich_cloud(itemCpy_cloud_in,resCpy_res,grpCpy_grp);
    }
}

void ST_StepEnrichCloudWithCurvature::enrich_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp)
{
    const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();

    size_t size = index->size();
    CT_NormalCloudStdVector *normalCloud = new CT_NormalCloudStdVector( size );
    CT_StandardCloudStdVectorT<float> *curvaturesCloud = new CT_StandardCloudStdVectorT<float>(size);
    CT_StandardCloudStdVectorT<float> *eigen1Cloud = new CT_StandardCloudStdVectorT<float>(size);
    CT_StandardCloudStdVectorT<float> *eigen2Cloud = new CT_StandardCloudStdVectorT<float>(size);
    CT_StandardCloudStdVectorT<float> *eigen3Cloud = new CT_StandardCloudStdVectorT<float>(size);
    CT_StandardCloudStdVectorT<float> *stemCloud = new CT_StandardCloudStdVectorT<float>(size);

    for(size_t i =0; i < size; i ++)
    {
        PointS p = _cloud->points[i];
        float n1 = p.normal_x;
        float n2 = p.normal_y;
        float n3 = p.normal_z;
        float eigen1 = p.eigen1;
        float eigen2 = p.eigen2;
        float eigen3 = p.eigen3;

        CT_Normal &ctNormal = normalCloud->normalAt(i);
        ctNormal.x() = n1;
        ctNormal.y() = n2;
        ctNormal.z() = n3;
        ctNormal.w() = p.curvature;

        curvaturesCloud->tAt(i) =p.curvature;
        eigen1Cloud->tAt(i) = eigen1;
        eigen2Cloud->tAt(i) = eigen2;
        eigen3Cloud->tAt(i) = eigen3;

    }
    CT_PointsAttributesNormal * normals = new CT_PointsAttributesNormal(_cloud_out_normals.completeName(), resCpy_res, itemCpy_cloud_in->getPointCloudIndexRegistered(),normalCloud);
    CT_PointsAttributesScalarTemplated<float> * curvatures =  new CT_PointsAttributesScalarTemplated<float>
            (_cloud_out_curvature.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(), curvaturesCloud);
    CT_PointsAttributesScalarTemplated<float> * eigenvalues1 =  new CT_PointsAttributesScalarTemplated<float>
            (_cloud_out_eigen1.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(), eigen1Cloud);
    CT_PointsAttributesScalarTemplated<float> * eigenvalues2 =  new CT_PointsAttributesScalarTemplated<float>
            (_cloud_out_eigen2.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(), eigen2Cloud);
    CT_PointsAttributesScalarTemplated<float> * eigenvalues3 =  new CT_PointsAttributesScalarTemplated<float>
            (_cloud_out_eigen3.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(), eigen3Cloud);
    CT_PointsAttributesScalarTemplated<float> * stem =  new CT_PointsAttributesScalarTemplated<float>
            (_cloud_out_stem.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(),stemCloud);


    grpCpy_grp->addItemDrawable(normals);
    grpCpy_grp->addItemDrawable(curvatures);
    grpCpy_grp->addItemDrawable(eigenvalues1);
    grpCpy_grp->addItemDrawable(eigenvalues2);
    grpCpy_grp->addItemDrawable(eigenvalues3);
    grpCpy_grp->addItemDrawable(stem);
}

void ST_StepEnrichCloudWithCurvature::create_simple_tree_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in)
{
    if (itemCpy_cloud_in != NULL)
    {
        const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();

        size_t size = 0;
        size = index->size();
        _cloud.reset(new PointCloudS);
        _cloud->width = size;
        _cloud->height = 1;
        if(size > 0) {
            _cloud->points.resize(size);
            size_t i = 0;
            CT_PointIterator it (index);
            while(it.hasNext())
            {
                const CT_PointData &internalPoint = it.next().currentConstInternalPoint();
                PointS p (internalPoint[0],internalPoint[1],internalPoint[2]);
//                p.x =internalPoint[0];
//                p.y =internalPoint[1];
//                p.z =internalPoint[2];
                _cloud->points[i] = p;
                ++i;
            }
        }
        EnrichCloud enrich(_cloud, _knn, _range, _use_knn);
    }


}
