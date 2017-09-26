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

#include "st_templatestep.h"

#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_itemdrawable/abstract/ct_abstractitemgroup.h"
#include "ct_itemdrawable/ct_pointsattributesnormal.h"
#include "ct_itemdrawable/ct_pointsattributesscalartemplated.h"

#include "ct_result/ct_resultgroup.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_normalcloud/ct_normalcloudstdvector.h"
#include "ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"
#define DEFin_normals_in "no_in"
#define DEFin_curvature_in "cu_in"
#define DEFin_eigen1_in "e1_in"
#define DEFin_eigen2_in "e2_in"

// Constructor : initialization of parameters
ST_TemplateStep::ST_TemplateStep(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
}

ST_TemplateStep::~ST_TemplateStep()
{
}

// Step description (tooltip of contextual menu)
QString ST_TemplateStep::getStepDescription() const
{
    return tr("Template step");
}

// Step detailled description
QString ST_TemplateStep::getStepDetailledDescription() const
{
    return tr("?" );
}

// Step URL
QString ST_TemplateStep::getStepURL() const
{
    //return tr("STEP URL HERE");
    return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_TemplateStep::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_TemplateStep(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_TemplateStep::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Isolated Tree"));
    resIn_res->addItemModel(DEFin_grp, DEFin_normals_in, CT_PointsAttributesNormal::staticGetType(), tr("Curvature"));
    resIn_res->addItemModel(DEFin_grp, DEFin_curvature_in, CT_PointsAttributesScalarTemplated<float>::staticGetType(), tr("Eigenvalue1"));
    resIn_res->addItemModel(DEFin_grp, DEFin_eigen1_in, CT_PointsAttributesScalarTemplated<float>::staticGetType(), tr("Eigenvalue2"));
    resIn_res->addItemModel(DEFin_grp, DEFin_eigen2_in, CT_PointsAttributesScalarTemplated<float>::staticGetType(), tr("Eigenvalue3"));
}

// Creation and affiliation of OUT models
void ST_TemplateStep::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_curvature, new CT_PointsAttributesScalarTemplated<float>(),tr("New curvature"));
    }
}

// Semi-automatic creation of step parameters DialogBox
void ST_TemplateStep::createPostConfigurationDialog()
{
}

void ST_TemplateStep::compute()
{
    // DONT'T FORGET TO ADD THIS STEP TO THE PLUGINMANAGER !!!!!

    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);

    // COPIED results browsing
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);

    while (itCpy_grp.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();

        const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);

        if(itemCpy_cloud_in != NULL) {
            const CT_PointsAttributesNormal* normals_in = (CT_PointsAttributesNormal*)grpCpy_grp->firstItemByINModelName(this, DEFin_normals_in);

            if(normals_in != NULL) {
                size_t size = normals_in->attributesSize();

                for(size_t i=0; i<size; ++i) {
                    const CT_Normal &normal = normals_in->constNormalAt(i);

                    // do what you want with the normal
                }
            }

            const CT_PointsAttributesScalarTemplated<float> *curvature_in = (CT_PointsAttributesScalarTemplated<float>*)grpCpy_grp->firstItemByINModelName(this, DEFin_curvature_in);

            if(curvature_in != NULL) {
                size_t size = curvature_in->attributesSize();

                CT_StandardCloudStdVectorT<float> *curvaturesOutCloud = new CT_StandardCloudStdVectorT<float>(size);

                for(size_t i=0; i<size; ++i) {
                    const float &curvature = curvature_in->valueAt(i);

                    // do what you want with the curvature
                    curvaturesOutCloud->tAt(i) = curvature + 50;
                }

                CT_PointsAttributesScalarTemplated<float> * curvatures =  new CT_PointsAttributesScalarTemplated<float>
                        (_cloud_out_curvature.completeName(), resCpy_res, curvature_in->getPointCloudIndexRegistered(), curvaturesOutCloud);

                grpCpy_grp->addItemDrawable(curvatures);
            }
        }

    }
}
