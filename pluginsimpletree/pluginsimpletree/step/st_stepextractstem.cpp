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
#include "st_stepextractstem.h"

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
ST_StepExtractStem::ST_StepExtractStem(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    //    _sceneList = new QList<CT_Scene*>();

}

ST_StepExtractStem::~ST_StepExtractStem()
{
    //    delete _sceneList;
    //    _sceneList->clear();
}

// Step description (tooltip of contextual menu)
QString ST_StepExtractStem::getStepDescription() const
{
    return tr("Extracts the detected Stem points.");
}

// Step detailled description
QString ST_StepExtractStem::getStepDetailledDescription() const
{
    return tr("The stem points are extracted." );
}

// Step URL
QString ST_StepExtractStem::getStepURL() const
{
    //return tr("STEP URL HERE");
    return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepExtractStem::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepExtractStem(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepExtractStem::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("Result_In"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("Grp_In"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Tree_Cloud"));
    resIn_res->addItemModel(DEFin_grp,DEFin_stem_in,CT_PointsAttributesScalarTemplated<int>::staticGetType(),tr("Stem_Classification"));
}

// Creation and affiliation of OUT models
void ST_StepExtractStem::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {
        res->addItemModel(DEFin_grp, _outScene_stem, new CT_Scene(), tr("Extracted_Stem"));
        res->addItemModel(DEFin_grp, _outScene_twigs, new CT_Scene(), tr("Extracted_Twigs"));
    }

}

// Semi-automatic creation of step parameters DialogBox
void ST_StepExtractStem::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addTitle(tr("You will find help and description about the paramters in the following publication."));
    configDialog->addTitle(tr("This is also the correct citation you should give for scientific publications."));
    configDialog->addEmpty();
    configDialog->addTitle(tr("Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P."));
    configDialog->addTitle(tr("<em>SimpleTree —An Efficient Open Source Tool to Build Tree Models from TLS Clouds.</em>"));
    configDialog->addTitle(tr("Forests <b>2015</b>, 6, 4245-4294. "));
        configDialog->addEmpty();
    configDialog->addText("Extracts all points which are classified as stem or major branch.");
}

void ST_StepExtractStem::compute()
{
    // CT_ResultGroup* resultIn_inputResult = getInputResults().first();
    CT_ResultGroup* resultOut_translated = getOutResultList().first();

    CT_ResultGroupIterator it(resultOut_translated, this, DEFin_grp);
    while (it.hasNext())
    {
        CT_StandardItemGroup* grp = (CT_StandardItemGroup*) it.next();
        const CT_Scene* scene = (const CT_Scene*)grp->firstItemByINModelName(this, DEFin_cloud_in);
        const CT_PointsAttributesScalarTemplated<int> *stem_flag__in
                = (CT_PointsAttributesScalarTemplated<int>*)grp->firstItemByINModelName(this, DEFin_stem_in);
        const CT_AbstractSingularItemDrawable* item = NULL;

        if (scene != NULL)
        {
            const CT_AbstractPointCloudIndex *pointCloudIndex = scene->getPointCloudIndex();
            size_t nbPoints = pointCloudIndex->size();

            // On Cree un nouveau nuage qui sera le translate
            CT_PointCloudIndexVector *extractedCloud = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
            CT_PointCloudIndexVector *extractedCloud2 = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
            CT_PointIterator itP(pointCloudIndex);
            const CT_AbstractPointCloudIndex* apci =  stem_flag__in->getPointCloudIndex();
            size_t i = 0;

            while (itP.hasNext() && !isStopped())
            {

                itP.next();
                size_t index = itP.currentGlobalIndex();
                const float stem_flag = stem_flag__in->valueAt(i);
                if ( stem_flag == 1)
                {
                    extractedCloud->addIndex(index);             // 2) adding kept indices
                } else {
                    extractedCloud2->addIndex(index);
                }

                setProgress( 99.0*i++ /nbPoints );
            }

            if (extractedCloud->size() > 0)
            {
                CT_Scene* outScene = new CT_Scene(_outScene_stem.completeName(),
                                                  resultOut_translated, PS_REPOSITORY->registerPointCloudIndex(extractedCloud)); // 3) create scene, registering the pointcloudindex
                outScene->updateBoundingBox(); // 4) don't forget to update the bounding box, to be fitted to filtered points

                grp->addItemDrawable(outScene);
            }
            if(extractedCloud2->size() > 0)
            {
                CT_Scene* outScene = new CT_Scene(_outScene_twigs.completeName(),
                                                  resultOut_translated, PS_REPOSITORY->registerPointCloudIndex(extractedCloud2));
                outScene->updateBoundingBox();
                grp->addItemDrawable(outScene);
            }
        }
    }
}





