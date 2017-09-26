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
#include "st_stepdetectstem.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"


ST_StepDetectStem::ST_StepDetectStem(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{

    _e1_min = 0;
    _e1_max = 0.15;
    _e2_min = 0.25;
    _e2_max = 1;
    _e3_min = 0;
    _e3_max = 0.8;
    _number_clusters = 3;
}

ST_StepDetectStem::~ST_StepDetectStem()
{
}

// Step description (tooltip of contextual menu)
QString ST_StepDetectStem::getStepDescription() const
{
    return tr("Classify the Stem points.");
}

// Step detailled description
QString ST_StepDetectStem::getStepDetailledDescription() const
{
    return tr("The stem points are detected in a semi automatic manner. The user has to give min and max for the Eigenvalues after a principal component analysis"
              "is applied on each points' neighborhood." );
}

// Step URL
QString ST_StepDetectStem::getStepURL() const
{
    //return tr("http://www.simpletree.uni-freiburg.de/news.html");
    return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepDetectStem::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepDetectStem(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepDetectStem::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("Result_In"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("Grp_In"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Tree_Cloud"));



}

// Creation and affiliation of OUT models
void ST_StepDetectStem::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_normals, new CT_PointsAttributesNormal(),tr("Normals"));
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_stem, new CT_PointsAttributesScalarTemplated<int>(),tr("Stem_Classification"));
    }
}

// Semi-automatic creation of step parameters DialogBox
void ST_StepDetectStem::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addTitle(tr("You will find help and description about the paramters in the following publication."));
    configDialog->addTitle(tr("This is also the correct citation you should give for scientific publications."));
    configDialog->addEmpty();
    configDialog->addText("A PCA is performed on each input points neighbourhood of the input cloud."
                          "If the eigenvalues E1, E2 and E3 fulfil user-given thresholds, the point is accepted as a stem point.");
    configDialog->addEmpty();
    configDialog->addTitle(tr("Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P."));
    configDialog->addTitle(tr("<em>SimpleTree —An Efficient Open Source Tool to Build Tree Models from TLS Clouds.</em>"));
    configDialog->addTitle(tr("Forests <b>2015</b>, 6, 4245-4294. "));
    configDialog->addEmpty();
    configDialog->addDouble( tr("E1 min"), "",  0, 1,2,  _e1_min, 1, "E1 min.");
    configDialog->addDouble( tr("E1 max"), "",  0, 1,2,  _e1_max, 1, "E1 max.");
    configDialog->addDouble( tr("E2 min"), "",  0, 1,2,  _e2_min, 1, "E2 min.");
    configDialog->addDouble( tr("E2 max"), "",  0, 1,2,  _e2_max, 1, "E2 max.");
    configDialog->addDouble( tr("E3 min"), "",  0, 1,2,  _e3_min, 1, "E3 min.");
    configDialog->addDouble( tr("E3 max"), "",  0, 1,2,  _e3_max, 1, "E3 max.");
    configDialog->addInt(tr("number of clusters" ),"",1,1000, _number_clusters, "The maximum number of clusters");
}

void ST_StepDetectStem::compute()
{
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);

    while (itCpy_grp.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        CT_Scene* itemIn_scene = (CT_Scene*) grpCpy_grp;
        const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in =
                (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        create_simple_tree_cloud(itemCpy_cloud_in);
        enrich_cloud(itemCpy_cloud_in,resCpy_res,grpCpy_grp);
    }
}

void ST_StepDetectStem::enrich_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp)
{
    const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();
    size_t size = index->size();
    CT_NormalCloudStdVector *normalCloud = new CT_NormalCloudStdVector( size );
    CT_StandardCloudStdVectorT<int> *stemCloud = new CT_StandardCloudStdVectorT<int>(size);

    for(size_t i =0; i < size; i ++)
    {
        PointS p = _cloud_in->points[i];
        float n1 = p.normal_x;
        float n2 = p.normal_y;
        float n3 = p.normal_z;
        float stem = p.is_stem;
        CT_Normal &ctNormal = normalCloud->normalAt(i);
        ctNormal.x() = n1;
        ctNormal.y() = n2;
        ctNormal.z() = n3;
        ctNormal.w() = p.curvature;
        stemCloud->tAt(i) = stem;
    }

    CT_PointsAttributesNormal * normals =
            new CT_PointsAttributesNormal(_cloud_out_normals.completeName(), resCpy_res,
                                          itemCpy_cloud_in->getPointCloudIndexRegistered(),normalCloud);

    CT_PointsAttributesScalarTemplated<int> * stem =
            new CT_PointsAttributesScalarTemplated<int>(_cloud_out_stem.completeName(), resCpy_res,
                                          itemCpy_cloud_in->getPointCloudIndexRegistered(),stemCloud);

    grpCpy_grp->addItemDrawable(normals);
    grpCpy_grp->addItemDrawable(stem);

}

void ST_StepDetectStem::create_simple_tree_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in)
{
    if (itemCpy_cloud_in != NULL)
    {
        const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();

        size_t size = 0;
        size = index->size();
        _cloud_in.reset(new PointCloudS);
        _cloud_in->width = size;
        _cloud_in->height = 1;
        if(size > 0)
        {
            _cloud_in->points.resize(size);
            size_t i = 0;
            CT_PointIterator it (index);
            while(it.hasNext())
            {
                const CT_PointData &internalPoint = it.next().currentConstInternalPoint();
                PointS p (internalPoint[0],internalPoint[1],internalPoint[2]);
                _cloud_in->points[i] = p;
                ++i;
            }
        }
        EnrichCloud enrich(_cloud_in, 16, 0, true);
        StemPointDetection stempts (_e1_min,_e1_max,_e2_min,_e2_max,_e3_min,_e3_max,0.035,_cloud_in, _number_clusters) ;
        stempts.compute();
    }
}



