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

#include "st_stepcompletefoldermodelling_eric.h"






float ST_StepCompleteFolderModelling_Eric::compute_height_above_DTM(CT_Image2D<float> *mnt, CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in)
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


ST_StepCompleteFolderModelling_Eric::ST_StepCompleteFolderModelling_Eric(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _knn = 16;
    _cut_height = 0.2;

    // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

ST_StepCompleteFolderModelling_Eric::~ST_StepCompleteFolderModelling_Eric()
{
}


// Step description (tooltip of contextual menu)
QString ST_StepCompleteFolderModelling_Eric::getStepDescription() const
{
    return tr("QSM spherefollowing method - modified for a project submission.");
}

// Step detailled description
QString ST_StepCompleteFolderModelling_Eric::getStepDetailledDescription() const
{
    return tr("QSM spherefollowing method - modified for a project submission.");
}

// Step URL
QString ST_StepCompleteFolderModelling_Eric::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
}

// Step copy method
ST_StepCompleteFolderModelling_Eric* ST_StepCompleteFolderModelling_Eric::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepCompleteFolderModelling_Eric(dataInit);
}

void ST_StepCompleteFolderModelling_Eric::down_scale_cloud_voxel(PointCloudS::Ptr origin, PointCloudS::Ptr downscaled_cloud, QString &id, int j)
{
    float leaf_size = 0.005f;
    switch (j) {
    case 0:
        leaf_size = 0.01f;
        break;
    case 1:
        leaf_size = 0.02f;
        break;
    default:
        break;
    }

    QVector<PointCloudS::Ptr> clusters = voxelize_cloud(origin);
    for(size_t i = 0; i < clusters.size(); i++)
    {
        PointCloudS::Ptr cloud = clusters.at(i);
        PointCloudS::Ptr down_scaled_cloud (new PointCloudS);
        pcl::VoxelGrid<PointS> sor;
        sor.setInputCloud ( cloud);
        sor.setLeafSize ( leaf_size, leaf_size, leaf_size );
        sor.filter ( *down_scaled_cloud );
        *downscaled_cloud += *down_scaled_cloud;
    }

    QString str = "_voxel_grid_";
    int ls = leaf_size*1000;
    str.append(QString::number(ls));
    str.append("_");
    id.append(str);
}

void ST_StepCompleteFolderModelling_Eric::statistical_outlier_cloud_voxel(PointCloudS::Ptr origin, PointCloudS::Ptr filtered_cloud_3, QString &id, int j)
{
    filtered_cloud_3.reset(new PointCloudS);
    *filtered_cloud_3 += *origin;
    //    QVector<PointCloudS::Ptr> clusters = voxelize_cloud(origin);
    //    for(size_t i = 0; i < clusters.size(); i++)
    //    {




    //        PointCloudS::Ptr cloud = clusters.at(i);



    //        PointCloudS::Ptr filtered_cloud (new PointCloudS);
    //        PointCloudS::Ptr filtered_cloud_2 (new PointCloudS);
    //        {

    //            pcl::StatisticalOutlierRemoval<PointS> sor;
    //            sor.setInputCloud (cloud);
    //            sor.setMeanK (2);
    //            sor.setStddevMulThresh (3);
    //            sor.filter (*filtered_cloud);
    //        }

    //        {

    //            pcl::StatisticalOutlierRemoval<PointS> sor;
    //            sor.setInputCloud (filtered_cloud);
    //            sor.setMeanK (20);
    //            sor.setStddevMulThresh (3);
    //            sor.filter (*filtered_cloud_2);
    //        }

    //        *filtered_cloud_3 += *filtered_cloud_2;
    //    }


}

QVector<PointCloudS::Ptr> ST_StepCompleteFolderModelling_Eric::voxelize_cloud(PointCloudS::Ptr cloud_in, float res)
{
    QVector<PointCloudS::Ptr> clusters;
    if (cloud_in!=NULL)
    {
        float   minX = std::numeric_limits<float>::max();
        float   minY = std::numeric_limits<float>::max();
        float   minZ = std::numeric_limits<float>::max();
        float   maxX = std::numeric_limits<float>::lowest();
        float   maxY = std::numeric_limits<float>::lowest();
        float   maxZ = std::numeric_limits<float>::lowest();



        for(size_t i = 0; i < cloud_in->points.size(); i++)
        {
            PointS p = cloud_in->points.at(i);
            if(p.x < minX) minX = p.x;
            if(p.y < minY) minY = p.y;
            if(p.z < minZ) minZ = p.z;
            if(p.x > maxX) maxX = p.x;
            if(p.y > maxY) maxY = p.y;
            if(p.z > maxZ) maxZ = p.z;
        }
        float dif_x  = maxX - minX;
        float dif_y  = maxY - minY;
        float dif_z  = maxZ - minZ;
        int dimX = std::floor(dif_x / res)+1;
        int dimY = std::floor(dif_y / res)+1;
        int dimZ = std::floor(dif_z / res)+1;

        if(dimX == 0) dimX = 1;
        if(dimY == 0) dimY = 1;
        if(dimZ == 0) dimZ = 1;
        int total = dimX * dimY * dimZ;
        for(size_t k =0 ; k < total; k++)
        {
            PointCloudS::Ptr cloud (new PointCloudS);
            clusters.push_back(cloud);
        }
        for(size_t i = 0; i < cloud_in->points.size(); i++)
        {
            PointS p = cloud_in->points.at(i);
            float x = std::floor((p.x - minX)/res);
            float y = std::floor((p.y - minY)/res);
            float z = std::floor((p.z - minZ)/res);
            int index = x * dimY *dimZ + y * dimZ + z;
            PointCloudS::Ptr cloud = clusters.at(index);
            cloud->points.push_back(p);
        }
    }
    return clusters;
}


void ST_StepCompleteFolderModelling_Eric::merge_cloud(QVector<PointCloudS::Ptr> clouds, PointCloudS::Ptr merged_cloud, QString &id, int i)
{
    switch (i) {
    case 0:
        *merged_cloud += *(clouds.at(0));
        *merged_cloud += *(clouds.at(1));
        *merged_cloud += *(clouds.at(2));
        *merged_cloud += *(clouds.at(3));
        *merged_cloud += *(clouds.at(4));
        *merged_cloud += *(clouds.at(5));
        id.append("_1_2_3_4_5_6_");
        break;
    case 1:
        *merged_cloud += *(clouds.at(0));
        *merged_cloud += *(clouds.at(2));
        *merged_cloud += *(clouds.at(4));
        id.append("_1_3_5_");
        break;
    case 2:
        *merged_cloud += *(clouds.at(1));
        *merged_cloud += *(clouds.at(3));
        *merged_cloud += *(clouds.at(5));
        id.append("_2_4_6_");
        break;
    case 3:
        *merged_cloud += *(clouds.at(0));
        *merged_cloud += *(clouds.at(3));
        id.append("_1_4_");
        break;
    case 4:
        *merged_cloud += *(clouds.at(1));
        *merged_cloud += *(clouds.at(4));
        id.append("_2_5_");
        break;
    case 5:
        *merged_cloud += *(clouds.at(2));
        *merged_cloud += *(clouds.at(5));
        id.append("_3_6_");
        break;
    case 6:
        *merged_cloud += *(clouds.at(0));
        *merged_cloud += *(clouds.at(2));
        id.append("_1_3_");
        break;
    case 7:
        *merged_cloud += *(clouds.at(1));
        *merged_cloud += *(clouds.at(3));
        id.append("_2_4_");
        break;
    case 8:
        *merged_cloud += *(clouds.at(2));
        *merged_cloud += *(clouds.at(4));
        id.append("_3_5_");
        break;
    case 9:
        *merged_cloud += *(clouds.at(0));
        id.append("_1_");
        break;
    case 10:
        *merged_cloud += *(clouds.at(2));
        id.append("_3_");
        break;
    case 11:
        *merged_cloud += *(clouds.at(4));
        id.append("_5_");
        break;
    default:
        break;
    }

}





void ST_StepCompleteFolderModelling_Eric::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();

    configDialog->addDouble(tr("Cut height :"), "m",0 , 10, 2, _cut_height);
    configDialog->addTitle(tr("Put if you do not have a DTM."));
    configDialog->addInt(tr("remove number of cases :"), " (one merge) ",0 , 3 , _remove_merge_cases);
    dialog_simple_tree(configDialog);
}


// Creation and affiliation of IN models
void ST_StepCompleteFolderModelling_Eric::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resCloud = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resCloud->setZeroOrMoreRootGroup();
    resCloud->addGroupModel("", DEFin_cluster_grp, CT_AbstractItemGroup::staticGetType(), tr("Cluster grp"));
    resCloud->addItemModel(DEFin_cluster_grp, DEFin_clusters, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Clusters"));


    CT_InResultModelGroup *resHeader = createNewInResultModel(DEF_SearchInHeaderResult, tr("Header"), "", true);
    resHeader->setZeroOrMoreRootGroup();
    resHeader->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resHeader->addItemModel(DEFin_grp, DEFin_header, CT_FileHeader::staticGetType(), tr("File Header"));
}

// Creation and affiliation of OUT models
void ST_StepCompleteFolderModelling_Eric::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addItemModel(DEFin_cluster_grp, _tree_out, new ST_Tree(), tr("tree"));
        resCpy_res->addItemModel(DEFin_cluster_grp, _coeff_out, new ST_Coefficients(), tr("coefficients"));


        resCpy_res->addGroupModel(DEFin_cluster_grp, _topologyGroup, new CT_TTreeGroup(), tr("Topology"));
        resCpy_res->addGroupModel(_topologyGroup, _stemGroup, new CT_TNodeGroup(), tr("Armap compatible model"));
        resCpy_res->addItemModel(_stemGroup, _stemCylinders, new CT_Cylinder(), tr("Armap studio"));


        resCpy_res->addItemModel(DEFin_cluster_grp, _cloud_out_normals, new CT_PointsAttributesNormal(),tr("Normals"));
        resCpy_res->addItemModel(DEFin_cluster_grp, _cloud_out_stem, new CT_PointsAttributesScalarTemplated<float>(),tr("Stem flag"));


        resCpy_res->addGroupModel(DEFin_cluster_grp, _outCylinderGroupModelName, new CT_StandardItemGroup(), tr("Cylinder group"));
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

QString ST_StepCompleteFolderModelling_Eric::root(QString &a, QString &b)
{
    QString str;

    for(int i = 0; i < a.length() && i < b.length(); ++i) {
        if(a.at(i) == b.at(i))str.push_back(a.at(i));
        else break;
    }
    return str;
}

void ST_StepCompleteFolderModelling_Eric::compute()
{
    _number_trees = 0;
    _modelled_trees = 0;



    CT_ResultGroup* inHeaderResult = getInputResults().at(1);


    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_cluster_grp);
    CT_ResultGroupIterator itCpy_grp2(resCpy_res, this, DEFin_cluster_grp);
    CT_ResultGroupIterator it_HeaderGrp(inHeaderResult, this, DEFin_grp);
    CT_ResultGroupIterator it_HeaderGrp2(inHeaderResult, this, DEFin_grp);
    QVector<QString> headers;
    QVector<CT_AbstractItemDrawableWithPointCloud*> ct_clouds;
    QVector<PointCloudS::Ptr> pcl_clouds;

    while (itCpy_grp2.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grp_header = (CT_StandardItemGroup*) it_HeaderGrp2. next();
        CT_FileHeader* itemCpy_header = (CT_FileHeader*)grp_header->firstItemByINModelName(this, DEFin_header);
        if(itemCpy_header!=0)
        {
            QString file_name = itemCpy_header->getFileName();
            headers.push_back(file_name);
        }

        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp2.next();
        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in =
                (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_clusters);

        if(itemCpy_cloud_in!=0)
        {
            ct_clouds.push_back(itemCpy_cloud_in);

            ConvertCTtoST ctst(itemCpy_cloud_in,16 , false,false);
            ctst.convert();
            PointCloudS::Ptr cloud = ctst.get_cloud();
            pcl_clouds.push_back(cloud);
        }
    }


    if(headers.size()==6 &&ct_clouds.size()==6)
    {
        for(int i = _remove_merge_cases; i < 9; i++)
        {
            QString st1 = headers.at(0);
            QString st2 = headers.at(1);
            QString id = root(st1,st2);
            QString id_2 = id;
            MethodCoefficients coeff;
            StepParameter_Eric param;
            param._cloud_name_ori = id_2;
            coeff.cut_height = _cut_height;
            param.is_extracted = false;
            PointCloudS::Ptr merged_cloud(new PointCloudS);
            merge_cloud(pcl_clouds,merged_cloud, id, i);
            for(int j = 1; j < 2; j++)
            {
                QString id2 = id;
                PointCloudS::Ptr ds_cloud(new PointCloudS);
                down_scale_cloud_voxel(merged_cloud,ds_cloud,id2,j);
                qDebug() << "ST_StepCompleteFolderModelling2::compute()";
                qDebug() << merged_cloud->points.size();
                qDebug() << ds_cloud->points.size();
                qDebug() << id2;

                pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS>);
                tree->setInputCloud (ds_cloud);
                param.pcl_cloud= ds_cloud;


                param.id = id2;
                param.coeff = coeff;
                param._cloud_out_normals = _cloud_out_normals.completeName();
                param._cloud_out_stem    = _cloud_out_stem.completeName();
                param._cluster_grp = _cluster_grp.completeName();
                param._clusters = _clusters.completeName();

                param._model = _tree_out.completeName();
                param._coeff = _coeff_out.completeName();

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
            }
        }
    }



    QSharedPointer<ModellingThreadPool_Eric> mtp(new ModellingThreadPool_Eric(_step_parameters));

    QObject::connect(mtp.data(), SIGNAL(emit_finished_tp()), this, SLOT(sent_finished_step()) );
    QObject::connect(mtp.data(), SIGNAL(emit_qstring_tp(QString)), this, SLOT(sent_qstring_step(QString)) );
    QObject::connect(mtp.data(), SIGNAL(emit_timinglist(QStringList)), this, SLOT(sent_timings(QStringList)) );
    QObject::connect(mtp.data(), SIGNAL(emit_number_cylinders(int)), this, SLOT(plot_number_cylinders(int)) );
    mtp->start_computation();
}







void ST_StepCompleteFolderModelling_Eric::sent_qstring_step(QString str)
{
    PS_LOG->addInfoMessage(this, str);
}

void ST_StepCompleteFolderModelling_Eric::sent_finished_step()
{
    _modelled_trees ++;
    int percentage = (((float)_modelled_trees)/((float)_number_trees)*100.0f);
    setProgress(percentage);
}

void ST_StepCompleteFolderModelling_Eric::sent_timings(QStringList timings)
{
    for(int i = 0; i < timings.size(); i++)
    {
        QString str = timings.at(i);
        PS_LOG->addInfoMessage(this, str);
    }
}

void ST_StepCompleteFolderModelling_Eric::plot_number_cylinders(int number)
{
    QString str = "On the plot a total number of ";
    str.append(QString::number(number));
    str.append(" cylinders was modelled.");

    PS_LOG->addInfoMessage(this, str);
}
