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
#include "st_stepcompletefoldermodelling2.h"






float ST_StepCompleteFolderModelling2::compute_height_above_DTM(CT_Image2D<float> *mnt, CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in)
{
    float height_above_dtm = std::numeric_limits<float>::max();
    if (mnt != NULL)
    {
        double na = mnt->NA();

        PointCloudS::Ptr dtm_cloud ( new PointCloudS);
        int number_cells = mnt->nCells();
        dtm_cloud->points.resize(number_cells);
        for(size_t i = 0; i < number_cells; i++)
        {
            Eigen::Vector3d center;
            mnt->getCellCenterCoordinates(i,center);
            double z =  mnt->valueAtIndex(i);
            double x = center(0);
            double y = center(1);

            if(z == na)
            {
                x = 30000000;
                y = 30000000;
            }
            PointS p (x,y,0) ;
            dtm_cloud->points[i] = p;
        }
        pcl::KdTreeFLANN<PointS> kdtree;
        kdtree.setInputCloud (dtm_cloud);
        Eigen::Vector3d min, max;
        itemCpy_cloud_in->getBoundingBox(min,max);
        const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in->getPointCloudIndex();
        CT_PointIterator itP(pointCloudIndex);
        while(itP.hasNext() && !isStopped())
        {
            const CT_Point &point = itP.next().currentPoint();
            if(point(2)<(min(2)+0.2f))
            {

                PointS query (point(0),point(1),0);
                int K = 3;
                std::vector<int> pointIdxNKNSearch(K);
                std::vector<float> pointNKNSquaredDistance(K);

                kdtree.nearestKSearch (query, K, pointIdxNKNSearch, pointNKNSquaredDistance);
                float dist1 = 1/std::sqrt(pointNKNSquaredDistance[0]);
                float dist2 = 1/std::sqrt(pointNKNSquaredDistance[1]);
                float dist3 = 1/std::sqrt(pointNKNSquaredDistance[2]);


                float dist_sum = dist1+dist2+dist3;

                float hauteur1 = mnt->valueAtIndex(pointIdxNKNSearch[0]);
                float hauteur2 = mnt->valueAtIndex(pointIdxNKNSearch[1]);
                float hauteur3 = mnt->valueAtIndex(pointIdxNKNSearch[2]);
                float height = hauteur1*dist1/dist_sum + hauteur2*dist2/dist_sum + hauteur3*dist3/dist_sum;
                height = point(2)-height;
                if(height < height_above_dtm)
                {
                    height_above_dtm = height;
                }
            }

        }

        PS_LOG->addMessage(LogInterface::info, LogInterface::step, QString(tr("The cloud is %1 m above ground.")).arg(height_above_dtm));
        return height_above_dtm;

    }
    PS_LOG->addMessage(LogInterface::info, LogInterface::step, QString(tr("The cloud is %1 m above ground.")).arg(_cut_height));
    return _cut_height;
}


ST_StepCompleteFolderModelling2::ST_StepCompleteFolderModelling2(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _knn = 16;
    _cut_height = 0.32;

    // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

ST_StepCompleteFolderModelling2::~ST_StepCompleteFolderModelling2()
{
}


// Step description (tooltip of contextual menu)
QString ST_StepCompleteFolderModelling2::getStepDescription() const
{
    return tr("QSM spherefollowing method - advanced for plot.");
}

// Step detailled description
QString ST_StepCompleteFolderModelling2::getStepDetailledDescription() const
{
    return tr("See SimpleTree homepage - advanced for plot." );
}

// Step URL
QString ST_StepCompleteFolderModelling2::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    //return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
ST_StepCompleteFolderModelling2* ST_StepCompleteFolderModelling2::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepCompleteFolderModelling2(dataInit);
}





void ST_StepCompleteFolderModelling2::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addTitle(tr("Put if you do not have a DTM:"));
    configDialog->addDouble(tr("Cut height :"), "m",0 , 10, 2, _cut_height);
    configDialog->addEmpty();
    configDialog->addBool(tr("Uncheck for not use a median filter over the tree before a second cylinder fit (very recommended to keep checked)"),"","",_use_improve_by_median);
    configDialog->addEmpty();
    configDialog->addBool(tr("Uncheck for not use stem taper before a second cylinder fit (very recommended to keep checked)"),"","",_use_stem_taper);
    configDialog->addEmpty();
    configDialog->addBool(tr("Uncheck to not improve branch junctions  before a second cylinder fit (very recommended to keep checked)"),"","",_use_improve_branch_junctions);
    configDialog->addEmpty();
    configDialog->addBool(tr("Uncheck to not remove false cylinders before a second cylinder fit (very recommended to keep checked) "),"","",_use_remove_false_cylinders);
    configDialog->addEmpty();
    configDialog->addBool(tr("Uncheck if you want to use RANSAC instead of MLESAC fitting routine to keep checked (MLESAC should be better)"),"","",_use_mlesac);
    configDialog->addEmpty();
    configDialog->addBool(tr("Uncheck for not use a median filter over the tree after a second cylinder fit "),"","",_use_median_filter_later);
    configDialog->addText(tr("(recommended only if you do not know how to handle outliers (wrong radii fits) in e.g. stem tapers in your R work)"));
    configDialog->addEmpty();
    configDialog->addBool(tr("Uncheck for if you want to use all neighbors in 3cm range instead of 15 closest neighbors for normal comp."),"","",_use_knn);
    configDialog->addText(tr("(recommended if you know you have a good average resolution - like less than 1cm point distance"));
    configDialog->addEmpty();
    dialog_simple_tree(configDialog);
}


// Creation and affiliation of IN models
void ST_StepCompleteFolderModelling2::createInResultModelListProtected()
{   
    CT_InResultModelGroupToCopy *resCloud = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resCloud->setZeroOrMoreRootGroup();
    resCloud->addGroupModel("", DEFin_cluster_grp, CT_AbstractItemGroup::staticGetType(), tr("Cluster grp"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resCloud->addItemModel(DEFin_cluster_grp, DEFin_clusters, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Clusters"));


    CT_InResultModelGroup *resHeader = createNewInResultModel(DEF_SearchInHeaderResult, tr("Header"), "", true);
    resHeader->setZeroOrMoreRootGroup();
    resHeader->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resHeader->addItemModel(DEFin_grp, DEFin_header, CT_FileHeader::staticGetType(), tr("File Header"));


    CT_InResultModelGroup *resultMNT = createNewInResultModel(DEF_SearchInMNTResult, tr("MNT (Raster)"), "", true);
    resultMNT->setZeroOrMoreRootGroup();
    resultMNT->addGroupModel("", DEF_SearchInMNTGroup);
    resultMNT->addItemModel(DEF_SearchInMNTGroup, DEFin_DTM, CT_Image2D<float>::staticGetType(), tr("DTM")
                            , "", CT_InAbstractModel::C_ChooseOneIfMultiple);
    resultMNT->setMinimumNumberOfPossibilityThatMustBeSelectedForOneTurn(0);

}

// Creation and affiliation of OUT models
void ST_StepCompleteFolderModelling2::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addItemModel(DEFin_cluster_grp, _tree_out, new ST_Tree(), tr("tree"));
        resCpy_res->addItemModel(DEFin_cluster_grp, _coeff_out, new ST_Coefficients(), tr("coefficients"));


//        resCpy_res->addGroupModel(DEFin_cluster_grp, _topologyGroup, new CT_TTreeGroup(), tr("Topology"));
//        resCpy_res->addGroupModel(_topologyGroup, _stemGroup, new CT_TNodeGroup(), tr("Amap compatible model"));
//        resCpy_res->addItemModel(_stemGroup, _stemCylinders, new CT_Cylinder(), tr("Amap studio"));





//        resCpy_res->addItemModel(DEFin_cluster_grp, _cloud_out_normals, new CT_PointsAttributesNormal(),tr("Normals"));
//        resCpy_res->addItemModel(DEFin_cluster_grp, _cloud_out_stem, new CT_PointsAttributesScalarTemplated<float>(),tr("Stem flag"));


        resCpy_res->addGroupModel(DEFin_cluster_grp, _outCylinderGroupModelName, new CT_StandardItemGroup(), tr("Cylinder group"));

        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_bo_1, new CT_Cylinder(), tr("Up to Branch Order 0"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_bo_2, new CT_Cylinder(), tr("Up to Branch Order 1"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_bo_3, new CT_Cylinder(), tr("Up to Branch Order 2 or larger"));


//        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_reverse_bo_1, new CT_Cylinder(), tr("Up to Reverse Branch Order 1"));
//        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_reverse_bo_2, new CT_Cylinder(), tr("Up to Reverse Branch Order 2"));
//        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_reverse_bo_3, new CT_Cylinder(), tr("Up to Reverse Branch Order 3"));
//        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_reverse_bo_4, new CT_Cylinder(), tr("Up to Reverse Branch Order 4"));
//        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_reverse_bo_5, new CT_Cylinder(), tr("Up to Reverse Branch Order 5 or larger"));

//        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_spherefollowing, new CT_Cylinder(), tr("All Spherefollowing"));
//        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_attractor, new CT_Cylinder(), tr("All Attractor"));
//        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_stem_spherefollowing, new CT_Cylinder(), tr("Stem SphereFollowing"));
//        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_stem_attractor, new CT_Cylinder(), tr("Stem Attractor"));
//        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_branch_spherefollowing, new CT_Cylinder(), tr("Branch SphereFollowing"));
//        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_branch_attractor, new CT_Cylinder(), tr("Branch Attractor"));
//        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_spherefollowing_allom, new CT_Cylinder(), tr("Allometric corrected"));
//        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModel_spherefollowing_no_allom, new CT_Cylinder(), tr("Ransac fitted"));




        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_improved_by_allometry, new CT_Cylinder(), tr("Final Model"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _branchIDModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("branch_ID"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _branchOrderModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("branch_order"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _segmentIDModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("segment_ID"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _parentSegmentIDModelName,
                                          new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_ID),
                                          tr("parent_segment_ID"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _growthVolumeModelName,
                                          new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("growth_volume"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _detection_type,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("detection_method"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _improvement_type,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("improvement_method"));
    }
}

void ST_StepCompleteFolderModelling2::compute()
{
    qDebug() << "01";
    CT_Image2D<float>* mnt = NULL;
    if (getInputResults().size() > 2)
    {
        CT_ResultGroup* inMNTResult = getInputResults().at(2);
        CT_ResultItemIterator it(inMNTResult, this, DEFin_DTM);
        if(it.hasNext())
        {
            mnt = (CT_Image2D<float>*) it.next();
        }
    }
    CT_ResultGroup* inHeaderResult = getInputResults().at(1);
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_cluster_grp);
    CT_ResultGroupIterator it_HeaderGrp(inHeaderResult, this, DEFin_grp);
    CT_FileHeader* itemCpy_header = NULL;
    int id = 0;
    int cnt = 0;
    while (itCpy_grp.hasNext() && !isStopped())
    {
        if(mnt==NULL)
        {
            CT_StandardItemGroup* grp_header = (CT_StandardItemGroup*) it_HeaderGrp. next();
            itemCpy_header = (CT_FileHeader*)grp_header->firstItemByINModelName(this, DEFin_header);
        }
        else
        {
            if(cnt == 0)
            {
                cnt++;
                CT_StandardItemGroup* grp_header = (CT_StandardItemGroup*) it_HeaderGrp.next();
                itemCpy_header = (CT_FileHeader*)grp_header->firstItemByINModelName(this, DEFin_header);
            }
        }
        id++;
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        _number_trees++;
        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in =
                (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_clusters);
        if(itemCpy_cloud_in!=NULL)
        {
            StepParameter2 param;
            MethodCoefficients coeff;
            if(mnt != NULL)
            {
                coeff.cut_height = compute_height_above_DTM(mnt,itemCpy_cloud_in);

            }
            else
            {
                coeff.cut_height = _cut_height;
                param.is_extracted = false;
            }
            coeff.use_simple_error_term_sphere = _use_simple_error_term_sphere;
            coeff.use_remove_false_cylinders = _use_remove_false_cylinders;
            coeff.use_improve_by_median = _use_improve_by_median;
            coeff.use_stem_taper = _use_stem_taper;
            coeff.use_improve_branch_junctions = _use_improve_branch_junctions;
            coeff.use_simple_error_term_fit = _use_simple_error_term_fit;
            coeff.use_median_filter_later = _use_median_filter_later;
            if(_use_mlesac)
            {
                coeff.ransac_circle_type = pcl::SAC_MLESAC;
                coeff.ransac_type = pcl::SAC_MLESAC;
            } else {

                coeff.ransac_circle_type = pcl::SAC_RANSAC;
                coeff.ransac_type = pcl::SAC_RANSAC;
            }
            param.id = id;
            param.itemCpy_header = itemCpy_header;
            param.resCpy_res = resCpy_res;
            param.itmgrp = grpCpy_grp;
            param.coeff = coeff;
            param.itemCpy_cloud_in = itemCpy_cloud_in;
            param._cloud_out_normals = _cloud_out_normals.completeName();
            param._cloud_out_stem    = _cloud_out_stem.completeName();
            param._cluster_grp = _cluster_grp.completeName();
            param._clusters = _clusters.completeName();
            param._model = _tree_out.completeName();
            param._coeff = _coeff_out.completeName();
            param.use_knn = _use_knn;


            param._outCylinderModel_bo_1 = _outCylinderModel_bo_1.completeName();
            param._outCylinderModel_bo_2 = _outCylinderModel_bo_2.completeName();
            param._outCylinderModel_bo_3 = _outCylinderModel_bo_3.completeName();
            param._outCylinderModel_bo_4 = _outCylinderModel_bo_4.completeName();
            param._outCylinderModel_bo_5 = _outCylinderModel_bo_5.completeName();

            param._outCylinderModel_reverse_bo_1 = _outCylinderModel_reverse_bo_1.completeName();
            param._outCylinderModel_reverse_bo_2 = _outCylinderModel_reverse_bo_2.completeName();;
            param._outCylinderModel_reverse_bo_3 = _outCylinderModel_reverse_bo_3.completeName();;
            param._outCylinderModel_reverse_bo_4 = _outCylinderModel_reverse_bo_4.completeName();;
            param._outCylinderModel_reverse_bo_5 = _outCylinderModel_reverse_bo_5.completeName();;

            param._outCylinderModel_spherefollowing  = _outCylinderModel_spherefollowing.completeName();
            param._outCylinderModel_attractor  = _outCylinderModel_attractor.completeName();
            param._outCylinderModel_stem_spherefollowing  = _outCylinderModel_stem_spherefollowing.completeName();
            param._outCylinderModel_stem_attractor  = _outCylinderModel_stem_attractor.completeName();
            param._outCylinderModel_branch_spherefollowing  = _outCylinderModel_branch_spherefollowing.completeName();
            param._outCylinderModel_branch_attractor  = _outCylinderModel_branch_attractor.completeName();
            param._outCylinderModel_spherefollowing_allom  = _outCylinderModel_spherefollowing_allom.completeName();
            param._outCylinderModel_spherefollowing_no_allom  = _outCylinderModel_spherefollowing_no_allom.completeName();






            param._outCylinderGroupModelName = _outCylinderGroupModelName.completeName();
            param._outCylinderModelName_improved_branch_junctions = _outCylinderModelName_improved_branch_junctions.completeName();
            param._outCylinderModelName_unimproved = _outCylinderModelName_unimproved.completeName();
            param._outCylinderModelName_removed_false_cylinders = _outCylinderModelName_removed_false_cylinders.completeName();
            param._outCylinderModelName_removed_improved_by_median = _outCylinderModelName_removed_improved_by_median.completeName();
            param._outCylinderModelName_improved_by_fit = _outCylinderModelName_improved_by_fit.completeName();
            param._outCylinderModelName_improved_by_allometry = _outCylinderModelName_improved_by_allometry.completeName();
            param._outCylinderModelName_improved_by_merge = _outCylinderModelName_improved_by_merge.completeName();
            param._branchIDModelName = _branchIDModelName.completeName();
            param._branchOrderModelName = _branchOrderModelName.completeName();
            param._segmentIDModelName = _parentSegmentIDModelName.completeName();
            param._parentSegmentIDModelName = _parentSegmentIDModelName.completeName();
            param._growthVolumeModelName = _growthVolumeModelName.completeName();
            param._tree_species = _tree_species.completeName();
            param._tree_id = _tree_id.completeName();
            param._detection_type = _detection_type.completeName();
            param._improvement_type = _improvement_type.completeName();
            param._topologyGroup = _topologyGroup.completeName();
            param._stemGroup     = _stemGroup.completeName();
            param._stemCylinders = _stemCylinders.completeName();
            _step_parameters.push_back(param);
        } else {
            qDebug() << "step model null cloud";
        }


    }
    if(_number_trees<=0)
    {
        _number_trees = 1;
    }
    QSharedPointer<ModellingThreadPool2> mtp(new ModellingThreadPool2(_step_parameters));
    QObject::connect(mtp.data(), SIGNAL(emit_finished_tp()), this, SLOT(sent_finished_step()) );
    QObject::connect(mtp.data(), SIGNAL(emit_qstring_tp(QString)), this, SLOT(sent_qstring_step(QString)) );
    QObject::connect(mtp.data(), SIGNAL(emit_timinglist(QStringList)), this, SLOT(sent_timings(QStringList)) );
    QObject::connect(mtp.data(), SIGNAL(emit_number_cylinders(int)), this, SLOT(plot_number_cylinders(int)) );
    mtp->start_computation();
}







void ST_StepCompleteFolderModelling2::sent_qstring_step(QString str)
{
    PS_LOG->addInfoMessage(this, str);
}

void ST_StepCompleteFolderModelling2::sent_finished_step()
{
    _modelled_trees ++;
    int percentage = (((float)_modelled_trees)/((float)_number_trees)*100.0f);
    setProgress(percentage);
}

void ST_StepCompleteFolderModelling2::sent_timings(QStringList timings)
{
    for(int i = 0; i < timings.size(); i++)
    {
        QString str = timings.at(i);
        PS_LOG->addInfoMessage(this, str);
    }
}

void ST_StepCompleteFolderModelling2::plot_number_cylinders(int number)
{
    QString str = "On the plot a total number of ";
    str.append(QString::number(number));
    str.append(" cylinders was modelled.");

    PS_LOG->addInfoMessage(this, str);
}
