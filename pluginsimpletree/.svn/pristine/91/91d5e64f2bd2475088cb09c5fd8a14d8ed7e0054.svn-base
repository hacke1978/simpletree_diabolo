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
#include "st_pluginmanager.h"
#include "ct_stepseparator.h"
#include "ct_steploadfileseparator.h"
#include "ct_stepcanbeaddedfirstseparator.h"
#include "ct_actions/ct_actionsseparator.h"
#include "ct_exporter/ct_standardexporterseparator.h"
#include "ct_reader/ct_standardreaderseparator.h"
#include "ct_actions/abstract/ct_abstractaction.h"
#include "step/filtering/st_stepfilterstempointscircle.h"


// Inclure ici les entetes des classes definissant des Ã©tapes/actions/exporters ou readers
#include "step/upanddownscaling/st_stepupscalecloud.h"
//#include "step/st_steploadsimpletreemodel.h"
#include "step/st_stepdetectstem.h"
#include "step/st_stepextractstem.h"
#include "step/modelling/st_stepcompletefoldermodelling.h"
#include "step/clustering/st_stepextractlargestcluster.h"
#include "step/modelling/dijkstra/st_stepdjikstra.h"
#include "step/filtering/st_stepradiusoutlierremoval.h"
#include "step/filtering/st_stepstatisticaloutlierremoval.h"
#include "step/filtering/st_stepghostpointremoval.h"
#include "step/upanddownscaling/st_stepvoxelgridfilter.h"
#include "step/export/st_stepexportmultipleclouds.h"
#include "step/clustering/st_stepmergeclouds.h"
#include "step/temp/st_stepfitsinglecylinder.h"
#include "step/temp/st_stepfitmultipleDBH.h"
#include "step/segmentation/st_stepsegmentation.h"
#include "step/segmentation/st_stepsegmentation2.h"
#include "step/segmentation/st_stepsegmentation3.h"
#include "step/segmentation/st_stepextractsliceabovedtm.h"
#include "step/clustering/st_stepeuclideanclustering.h"
#include "step/export/st_stepexportall.h"
#include "step/export/st_stepexportallEuroSDR.h"
#include "step/import/st_importcoeff.h"
#include "step/modelling/split/st_stepmodelwithparam1.h"
#include "step/modelling/split/st_stepmodelwithparam2.h"
#include "step/modelling/split/st_stepmodelwithpype.h"
#include "step/modelling/split/st_stepmodelling_growth_length.h"
#include "step/modelling/split/st_stepmodelling_growth_lengthRC.h"
#include "step/modelling/new/st_stepcompletefoldermodelling2.h"
#include "step/clustering/st_stepbuffering.h"
#include "step/clustering/st_stepsplitbyheight.h"
#include "step/clustering/st_stepfilterclusters.h"
#include "step/clustering/st_stepfilterclustersbydistance.h"
#include "step/modelling/split/st_stepmodelmovingaverage.h"
#include "step/clustering/st_stepgmm.h"
#include "step/gmm/st_step_eigen_ml.h"
#include "step/filtering/st_stepfiltergroundpoints.h"
#include "step/filtering/st_stepclearsky.h"
#include "step/modelling/st_stepdummyexport.h"
#include "step/modelling/st_stepcomputecrown.h"
#include "step/segmentation/st_stepsegmentedchm.h"
#include "step/segmentation/st_stepsegmentationall.h"
#include "step/modelling/st_step_adjust_branch_order.h"
#include "step/extract/st_extractmajorbranches.h"
#include "step/filtering/st_stepfilterstemseeds.h"
#include "step/filtering/st_stepfilterstempoints.h"
#include "step/mesh/st_step_poisson.h"
#include "step/modelling/new/st_stepdetectmisfits.h"
#include "step/dtm/st_stepcomputedtm.h"
#include "step/export/st_stepexportdtm.h"
#include "exporter/st_ascidexporter.h"
#include "step/pub2/st_stepincreasemindiameter.h"
#include "step/pub2/st_stepcropbranchorder.h"
#include "step/modelling/split/st_stepcropQSM.h"
#include "step/modelling/split/st_stepallometrycorrected_qsm.h"
#include "step/modelling/split/st_stepallometrycorrected_qsm_len.h"
#include "step/modelling/improve/st_step_detect_wrong_fits_in_qsm.h"
#include "step/diabolo_eric/st_stepcompletefoldermodelling_eric.h"
#include "step/filtering/std_out_multithread/st_step_std_out_multithread.h"

#include "ct_global/ct_context.h"

ST_PluginManager::ST_PluginManager() : CT_AbstractStepPlugin()
{
    /*m_logListener.setFilePath("bug.txt");
    PS_LOG->addPrioritaryLogListener(&m_logListener);*/
}

ST_PluginManager::~ST_PluginManager()
{
    //PS_LOG->removeLogListener(&m_logListener);
}

QString ST_PluginManager::getPluginOfficialName() const
{
    return "SimpleTree";
}

QStringList ST_PluginManager::getPluginRISCitationList() const
{
    QStringList list;
    list.append(QString("TY  - JOUR\n"
                   "T1  - SimpleTree - an efficient open source tool to build tree models from TLS clouds\n"
                   "A1  - Hackenberg, Jan\n"
                   "A1  - Spiecker, Heinrich\n"
                   "A1  - Calders, Kim\n"
                   "A1  - Disney, Mathias\n"
                   "A1  - Raumonen, Pasi\n"
                   "JO  - Forests\n"
                   "VL  - 6\n"
                   "IS  - 11\n"
                   "SP  - 4245\n"
                   "EP  - 4294\n"
                   "Y1  - 2015\n"
                   "PB  - Multidisciplinary Digital Publishing Institute\n"
                   "UL  - http://www.simpletree.uni-freiburg.de/\n"
                   "ER  - \n"));

    list.append(QString("TY  - CONF\n"
                        "T1  - 3d is here: Point cloud library (pcl)\n"
                        "A1  - Rusu, Radu Bogdan\n"
                        "A1  - Cousins, Steve\n"
                        "JO  - Robotics and Automation (ICRA), 2011 IEEE International Conference on\n"
                        "SP  - 1\n"
                        "EP  - 4\n"
                        "SN  - 1612843859\n"
                        "Y1  - 2011\n"
                        "PB  - IEEE\n"
                        "UL  - http://pointclouds.org\n"
                        "ER  - \n"));


    return list;
}

bool ST_PluginManager::loadGenericsStep()
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
//    addNewGeometricalShapesStep<ST_StepImportCoeff>("Import");
    addNewPointsStep<ST_StepUpscaleCloud> (CT_StepsMenu::LP_Filter);
//    addNewGeometricalShapesStep<ST_StepDummyExport> ("Test Armap");
    addNewPointsStep<ST_StepExtractLargestCluster> (CT_StepsMenu::LP_Filter);
    addNewPointsStep<ST_StepSegmentationAll> (CT_StepsMenu::LP_Clusters);
    addNewPointsStep<ST_StepExtactSliceAboveDTM> (CT_StepsMenu::LP_Extract);
    addNewPointsStep<ST_StepEuclideanClustering> (CT_StepsMenu::LP_Clusters);
    addNewPointsStep<ST_StepBuffering> (CT_StepsMenu::LP_Filter);
    addNewExportStep<ST_StepExportMultipleClouds> (CT_StepsMenu::LP_Points);
    addNewExportStep<ST_StepExportAll> ("SimpleTree file Export");
    addNewExportStep<ST_StepExportAllEuroSDR> ("EuroSDR format for SimpleTree QSMS");
    addNewExportStep<ST_StepExportDTM> ("SimpleTree DTM,CHM  to ply Export");
    addNewPointsStep<ST_StepExtractMajorBranches> (CT_StepsMenu::LP_Extract);
    addNewPointsStep<ST_StepGhostPointRemoval> (CT_StepsMenu::LP_Filter);
    addNewPointsStep<ST_StepRadiusOutlierRemoval> (CT_StepsMenu::LP_Filter);
    addNewPointsStep<ST_StepStatisticalOutlierRemoval> (CT_StepsMenu::LP_Filter);
    addNewPointsStep<ST_StepVoxelGridFilter> (CT_StepsMenu::LP_Filter);
    addNewPointsStep<ST_StepFilterStemPointsCircle> (CT_StepsMenu::LP_Filter);
    addNewPointsStep<ST_StepMergeClouds>(CT_StepsMenu::LP_Clusters);
    addNewPointsStep<ST_StepFilterClusters>(CT_StepsMenu::LP_Clusters);
    addNewPointsStep<ST_StepEigenML>(CT_StepsMenu::LP_Clusters);
    addNewPointsStep<ST_StepFilterClustersByDistance>(CT_StepsMenu::LP_Clusters);
    addNewPointsStep<ST_StepSplitByHeight>(CT_StepsMenu::LP_Filter);
    addNewPointsStep<ST_StepFilterGroundPoints>(CT_StepsMenu::LP_Filter);
    addNewPointsStep<ST_StepClearSky>(CT_StepsMenu::LP_Filter);
    addNewPointsStep<ST_StepFilterStemSeeds>(CT_StepsMenu::LP_Filter);
    addNewPointsStep<ST_StepFilterStems>(CT_StepsMenu::LP_Filter);
//    addNewGeometricalShapesStep<ST_StepModelling2>(CT_StepsMenu::LP_Detect);
//    addNewGeometricalShapesStep<ST_StepCropQSM>(CT_StepsMenu::LP_Detect);
//    addNewGeometricalShapesStep<ST_StepModellingGrowthLength>(CT_StepsMenu::LP_Detect);
    addNewGeometricalShapesStep<ST_StepCompleteFolderModelling2>(CT_StepsMenu::LP_Detect);
    addNewGeometricalShapesStep<ST_StepAdjustBranchOrder>(CT_StepsMenu::LP_Detect);
    addNewGeometricalShapesStep<ST_StepPoissonReconstruction>(CT_StepsMenu::LP_Detect);
//    addNewGeometricalShapesStep<ST_StepModellingImprovePype>(CT_StepsMenu::LP_Detect);
//    addNewGeometricalShapesStep<ST_StepIncreaseDiameter>(CT_StepsMenu::LP_Detect);
    addNewRastersStep<ST_StepComputeDTM>(CT_StepsMenu::LP_DEM);
    addNewPointsStep<ST_StepGMM> (CT_StepsMenu::LP_Clusters);
    addNewPointsStep<ST_Step_Std_Out_Multithreaded> (CT_StepsMenu::LP_Filter);
    addNewRastersStep<ST_StepSegmentedCHM> (CT_StepsMenu::LP_DEM);
//    addNewGeometricalShapesStep<ST_StepCropBranchOrder>(CT_StepsMenu::LP_Detect);
//    addNewGeometricalShapesStep<ST_StepAllometry_Corrected_QSM>(CT_StepsMenu::LP_Detect);
//    addNewGeometricalShapesStep<ST_StepAllometry_Corrected_QSM_len>(CT_StepsMenu::LP_Detect);
    addNewGeometricalShapesStep<ST_StepDetectWrongFitsInQSM>(CT_StepsMenu::LP_Detect);

    addNewGeometricalShapesStep<ST_StepCompleteFolderModelling_Eric>(CT_StepsMenu::LP_Detect);


    return true;
}

bool ST_PluginManager::loadOpenFileStep()
{
    return true;
}

bool ST_PluginManager::loadCanBeAddedFirstStep()
{
    return true;
}

bool ST_PluginManager::loadActions()
{
    return true;
}

bool ST_PluginManager::loadExporters()
{    clearExporters();

     CT_StandardExporterSeparator *sep = addNewSeparator(new CT_StandardExporterSeparator("Exporters"));
     sep->addExporter(new ST_ASCIDExporter());
    return true;
}

bool ST_PluginManager::loadReaders()
{
    return true;
}


