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
#include "simpletreestep.h"

SimpleTreeStep::SimpleTreeStep()
{

}

void SimpleTreeStep::recursiveRemoveGroupIfEmpty(CT_AbstractItemGroup *parent, CT_AbstractItemGroup *group) const
{
    if(parent != NULL)
    {
        parent->removeGroup(group);

        if(parent->isEmpty())
            recursiveRemoveGroupIfEmpty(parent->parentGroup(), parent);
    }
    else
    {
        ((CT_ResultGroup*)group->result())->removeGroupSomethingInStructure(group);
    }
}


QString SimpleTreeStep::toQString(int const i) const
{
    QString result = QString("%1").arg(i, 12, 10, QLatin1Char('0'));
    int w_size = result.size();
    if (w_size > 3) {
        result.insert(result.size() - 3, ".");
    }
    if (w_size > 6) {
        result.insert(result.size() - 7, ".");
    }
    if (w_size > 9) {
        result.insert(result.size() - 10, ".");
    }
    return result;
}

QString SimpleTreeStep::tofQString(const float f, int dec_places) const
{
    QString result;
    if(std::abs(f) < 1000)
    {
        result = QString::number( f, 'f', dec_places );
    } else {
        result = QString::number( f, 'e', dec_places );
    }
    return result;
}

float SimpleTreeStep::get_percentage(const int before, const int after) const
{
    float b = before;
    float a = after;
    if(b == 0)
    {
        return 0;
    }
    return (a/b*100);
}

QVector<PointCloudS::Ptr> SimpleTreeStep::voxelize_cloud(PointCloudS::Ptr cloud_in, float res)
{
    QVector<PointCloudS::Ptr> clusters;
    //    PointCloudS::Ptr dummy (new PointCloudS);
    //    clusters.push_back(dummy);
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

PointS SimpleTreeStep::get_center_of_mass(PointCloudS::Ptr cloud)
{
    PointS p (0,0,0);
    if(cloud->points.size()==0)
    {
        return p;
    }
    float x = 0;
    float y = 0;
    float z = 0;
    for(size_t i = 0; i < cloud->points.size(); i++)
    {
        PointS pt = cloud->points.at(i);
        x += pt.x;
        y += pt.y;
        z += pt.z;
    }
    x /= cloud->points.size();
    y /= cloud->points.size();
    z /= cloud->points.size();
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

void SimpleTreeStep::dialog_simple_tree(CT_StepConfigurableDialog *configDialog)
{
    configDialog->addEmpty();
    configDialog->addTitle(QObject::tr("You are using the SimpleTree Plugin developed by Jan Hackenberg."));
    configDialog->addTitle(QObject::tr("Please look for additional information as well as for citable resources on the following page:"));
    configDialog->addTitle(QObject::tr("http://rdinnovation.onf.fr/projects/computree-simpletree-beta-version"));
    configDialog->addTitle(QObject::tr("If for a single step a special citation is given, please use the special citation."));
    configDialog->addTitle(QObject::tr("For general usage of the SimpleTree plugin at least the following open source release paper is mandatory:"));
    configDialog->addEmpty();
    configDialog->addTitle(QObject::tr("Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P."));
    configDialog->addTitle(QObject::tr("<em>SimpleTree — An Efficient Open Source Tool to Build Tree Models from TLS Clouds.</em>"));
    configDialog->addTitle(QObject::tr("Forests <b>2015</b>, 6, 4245-4294."));
    //    configDialog->addTitle(QObject::tr("the Diablo project (http://diabolo-project.eu/) - working package (WP) 2.2"));

    //    configDialog->addTitle(QObject::tr("The project is founded by European Union and working package 2 has the generale purpose of Harmonizing growing stack, biomass and carbon estimation. "));

    //    configDialog->addTitle(QObject::tr("A pipeline is available which consists in general a QSM building (Full 3D description) method, an improved version of the"
    //                              "one published in "));
    //    configDialog->addTitle(QObject::tr("Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P., <em>SimpleTree —An Efficient Open Source Tool to Build Tree Models from TLS Clouds.</em>,Forests <b>2015</b>, 6, 4245-4294. "));
    //    configDialog->addTitle(QObject::tr("This is a software paper of an earlier Version of the here presented plugin which could in majority be integrated as well as proposals mentioned in future work section"));


    //        configDialog->addEmpty();
    //    configDialog->addTitle(QObject::tr("The modellding method was hugely improved by AI advancements, but not yet published.I try to provide with more information as soon as possible. You can always defend with the help of data provided here:"));
    //    configDialog->addTitle(QObject::tr("http://www.simpletree.uni-freiburg.de/openData.html  -  Please pay attention to the license of that data."));
    //        configDialog->addEmpty();

    //    configDialog->addTitle(QObject::tr("To account to occlusion problems adaptations based on point cloud density insprired by the solution discuscussed in:"));
    //    configDialog->addTitle(QObject::tr("Cote, J.F.; Fournier, R.A.; Egli, R.; <em>An architectural model of trees to estimate forest structural attributes using terrestrial LiDAR.</em>;Environ. Model. Softw. <b>2011</b>, 26, 761-777. "));

    //    configDialog->addEmpty();
    //    configDialog->addTitle(QObject::tr("In addition a full crown segmentation is presented. Crown segmentation is considered a more complex problem than stem segmentation and a publication is in preparation."));

    //    configDialog->addEmpty();
    //    configDialog->addTitle(QObject::tr("Also adapatations as well as DTM  modelling was adapted from the ONF-Plugin maybe best in the following citable form given:"));

    //    configDialog->addTitle(QObject::tr("Ahlem Othmani, Alexandre Piboule, M. Krebs, C. Stolz, L.F.C. Lew Yan Voon; <em>Towards automated and operational forest inventories with T-Lidar.  11th International Conference  on LiDAR Applications for Assessing Forest"
    //                                       "Ecosystems (SilviLaser) <b>2011</b>, Hobart Australia. "));
    //    configDialog->addTitle(QObject::tr("This is also a good citation in respect to the work done by the core team of Computree."));
    //    configDialog->addEmpty();
    //    configDialog->addTitle(QObject::tr("You should also cite in general the point cloud library:"));

    //    configDialog->addTitle(QObject::tr("Radu Bogdan Rusu and Steve Cousins, <em>3D is here: Point Cloud Library (PCL)</em> IEEE International Conference on Robotics and Automation (ICRA) <b>2011</b>, China Shanghai. "));


    //    configDialog->addTitle(QObject::tr("as well as the OpenCV library:"));

    //    configDialog->addTitle(QObject::tr("http://code.opencv.org/projects/opencv/wiki/CiteOpenCV"));
    //        configDialog->addEmpty();
    //       configDialog->addTitle(QObject::tr("I try to provide with more information as soon as possible.Please cite the given resources in case you use this version public."));


    //       configDialog->addEmpty();
    //      configDialog->addTitle(QObject::tr("You need to utilize clouds with the prequesitions:."));
    //      configDialog->addTitle(QObject::tr("The center of the cloud is near the origin (max 50m), no georeferenced clouds allowed. No real world height"));
    //      configDialog->addTitle(QObject::tr("I reach good runtime for plots smaller than 40m. I know the segmentation time increases exponentially "));
    //      configDialog->addTitle(QObject::tr("with the radius and running the pipeline on hectare sized plots seems impossible without manual interaction."));
    //      configDialog->addTitle(QObject::tr("Beware if you try to model exotic trees (not expected in Euorpean NFI) I did not analysie such with the current version of SimpleTree. No single scan design analysed yet."));
    //      configDialog->addEmpty();
    //     configDialog->addTitle(QObject::tr("I am not aware of crashes which cannot be prevented but I am aware the program can crash if you do not prevent it."));
    //     configDialog->addTitle(QObject::tr("I fixed already several of those issues and further appreciate bug reports in case of problems."));
    //     configDialog->addEmpty();
    //    configDialog->addTitle(QObject::tr("Dr. Jan Hackenberg"));
}

void SimpleTreeStep::warning_geo_referenced(PointCloudS::Ptr cloud)
{
    PointS center = get_center_of_mass(cloud);

    float dist = std::sqrt(center.x*center.x + center.y*center.y + center.z*center.z);

    if(dist > 250)
    {
        PS_LOG->addInfoMessage(LogInterface::unknow, QString("The input cloud seems to be georefernced. The  SimpleTree plugin relies on PCL functionalities. Floating point issues based on PCL  cause problems with that data. Please shift your cloud to the center of origin."));
        //        int ret = QMessageBox::warning( NULL, "Warning - do not use georeferenced data",
        //                                        "The input cloud seems to be georefernced. The  SimpleTree plugin relies on PCL functionalities. Floating point issues based on PCL  cause problems with that data. Please shift your cloud to the center of origin.");
    }
}

PointCloudS::Ptr SimpleTreeStep::downscale_cloud_ct(CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, float res)
{
    PointCloudS::Ptr downscaled_cloud (new PointCloudS);
    if (itemCpy_cloud_in!=NULL)
    {
        float   minX = itemCpy_cloud_in->minX();
        float   minY = itemCpy_cloud_in->minY();
        float   minZ = itemCpy_cloud_in->minZ();
        CT_Grid3D_Sparse<int>* hitGrid = CT_Grid3D_Sparse<int>::createGrid3DFromXYZCoords(NULL, NULL,
                                                                                          minX, minY, minZ,
                                                                                          itemCpy_cloud_in->maxX(), itemCpy_cloud_in->maxY(), itemCpy_cloud_in->maxZ(),
                                                                                          res, -1, 0);


        const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in->getPointCloudIndex();
        CT_PointIterator itP(pointCloudIndex);
        while(itP.hasNext() )
        {
            const CT_Point &point = itP.next().currentPoint();
            size_t index;
            hitGrid->indexAtXYZ(point(0), point(1), point(2),index);
            int val = hitGrid->valueAtIndex(index);
            val++;
            hitGrid->setValueAtIndex(index,val);
        }
        for(size_t i = 0; i < hitGrid->xArraySize(); i++)
        {
            for(size_t j = 0; j < hitGrid->yArraySize(); j++)
            {
                for(size_t k = 0; k < hitGrid->zArraySize(); k++)
                {
                    int val = hitGrid->value(i,j,k);
                    if(val>0 && val < 100000000) //ToDO fix 10 mio....
                    {
                        double x = hitGrid->getCellCenterX(i);
                        double y = hitGrid->getCellCenterY(j);
                        double z = hitGrid->getCellCenterZ(k);
                        PointS p(x,y,z);
                        p.ID = val;
                        downscaled_cloud->points.push_back(p);
                    }
                }
            }
        }
       // delete hitGrid;
//TODO not sure why but delete hitGrid seems to crash the filter
    }
    return downscaled_cloud;
}

PointCloudS::Ptr SimpleTreeStep::make_two_dimensional_pcl(PointCloudS::Ptr cloud)
{
    PointCloudS::Ptr cloud_out(new PointCloudS);
    size_t size = cloud->points.size();
    for(size_t i = 0; i < size; i++)
    {
        PointS origin = cloud->points.at(i);
        PointS flat (origin.x,origin.y,0);
        cloud_out->points.push_back(flat);
    }
    return cloud_out;
}


QVector<PointCloudS::Ptr> SimpleTreeStep::voxelize_cloud_from_ct(CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, float res)
{




    QVector<PointCloudS::Ptr> clusters;
    PointCloudS::Ptr dummy (new PointCloudS);
    clusters.push_back(dummy);
    if (itemCpy_cloud_in!=NULL)
    {

        float   minX = itemCpy_cloud_in->minX();
        float   minY = itemCpy_cloud_in->minY();
        float   minZ = itemCpy_cloud_in->minZ();


        // Declaring the output grids
        CT_Grid3D_Sparse<int>* hitGrid = CT_Grid3D_Sparse<int>::createGrid3DFromXYZCoords(NULL, NULL,
                                                                                          minX, minY, minZ,
                                                                                          itemCpy_cloud_in->maxX(), itemCpy_cloud_in->maxY(), itemCpy_cloud_in->maxZ(),
                                                                                          res, -1, 0);


        int clusterNumber = 1;

        const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in->getPointCloudIndex();
        CT_PointIterator itP(pointCloudIndex);
        while(itP.hasNext() )
        {
            const CT_Point &point = itP.next().currentPoint();
            int val = hitGrid->valueAtXYZ(point(0), point(1), point(2));

            if (val == -1)
            {
                val = clusterNumber;
                hitGrid->setValueAtXYZ(point(0), point(1), point(2), clusterNumber++);
                PointCloudS::Ptr clstr (new PointCloudS);
                clusters.push_back(clstr);
            }
            PointS p(point(0), point(1), point(2));
            clusters.at(val)->points.push_back(p);
        }
        delete hitGrid;
    }
    clusters.remove(0);
    return clusters;
}

PointCloudS::Ptr SimpleTreeStep::pcl_voxel_grid_filter_depricated(const PointCloudS::Ptr cloud_in,  CT_AbstractStep * step, float leaf_size, bool print_console) const
{
    VoxelGridFilter filter(cloud_in,leaf_size);
    filter.compute();
    if(print_console)
    {
        QString str;
        str.append("The input cloud was downscaled to a  cloud with ");
        float before = cloud_in->points.size();
        float after = filter.get_cloud_out()->points.size();
        float perc = get_percentage(before,after);
        QString percentage = tofQString(perc,2);
        str.append(percentage);
        str.append("% points remaining.");
        PS_LOG->addInfoMessage(step, str);
    }
    return filter.get_cloud_out();
}

PointCloudS::Ptr SimpleTreeStep::pcl_voxel_grid_filter(const PointCloudS::Ptr cloud_in,  CT_AbstractStep * step, float leaf_size, bool print_console)
{
    QVector<PointCloudS::Ptr> clusters = voxelize_cloud(cloud_in);
    PointCloudS::Ptr down_scaled_cloud_complete (new PointCloudS);
    for(size_t i = 0; i < clusters.size(); i++)
    {
        PointCloudS::Ptr cloud = clusters.at(i);
        PointCloudS::Ptr down_scaled_cloud (new PointCloudS);
        pcl::VoxelGrid<PointS> sor;
        sor.setInputCloud ( cloud);
        sor.setLeafSize ( leaf_size, leaf_size, leaf_size );
        sor.filter ( *down_scaled_cloud );
        *down_scaled_cloud_complete += *down_scaled_cloud;
    }
    if(print_console)
    {
        QString str;
        str.append("The input cloud was downscaled to a  cloud with ");
        float before = cloud_in->points.size();
        float after = down_scaled_cloud_complete->points.size();
        float perc = get_percentage(before,after);
        QString percentage = tofQString(perc,2);
        str.append(percentage);
        str.append("% points remaining.");
        PS_LOG->addInfoMessage(step, str);
    }
    return down_scaled_cloud_complete;
}

void SimpleTreeStep::pcl_compute_normals(PointCloudS::Ptr cloud_in, int k)
{
    pcl::NormalEstimationOMP<PointS, PointS> ne;
    ne.setInputCloud (cloud_in);
    pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS> ());
    ne.setSearchMethod (tree);
    ne.setKSearch(k);
    ne.compute (*cloud_in);
}

PointCloudS::Ptr SimpleTreeStep::pcl_CT_to_PCL_cloud(CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in,
                                                     CT_AbstractStep * step, int knn, bool convert_2D,
                                                     bool print_console)
{
    ConvertCTtoST ctst(itemCpy_cloud_in,knn, false);
    ctst.convert();
    if(!convert_2D)
    {
        return ctst.get_cloud();
    }
    if(print_console)
    {
        QString str;
        str.append("The input cloud was converted to a pcl cloud with ");
        QString number = toQString(ctst.get_cloud()->points.size());
        str.append(number);
        str.append(" points.");
        PS_LOG->addInfoMessage(step, str);
    }
    return ctst.make_two_dimenstional();
}

PointCloudS::Ptr SimpleTreeStep::pcl_copy_PCL_cloud(PointCloudS::Ptr cloud_in) const
{
    PointCloudS::Ptr cloud_out (new PointCloudS);
    pcl::copyPointCloud(*cloud_in,*cloud_out);
    return cloud_out;
}

void SimpleTreeStep::pcl_PCL_to_CT_cloud(const PointCloudS::Ptr cloud_in, CT_AbstractResult* resCpy_res,
                                         CT_StandardItemGroup* grpCpy_grp, QString const model_name, CT_AbstractStep *step,
                                         bool print_console)
{
    CT_AbstractUndefinedSizePointCloud* mpcir = PS_REPOSITORY->createNewUndefinedSizePointCloud();
    size_t size = cloud_in->points.size();
    for(size_t i = 0; i < size; i++)
    {
        PointS p = cloud_in->points.at(i);
        CT_Point point;
        point[0] = p.x;
        point[1] = p.y;
        point[2] = p.z;
        mpcir->addPoint(point);
    }
    CT_Scene* scene = new CT_Scene(model_name, resCpy_res, PS_REPOSITORY->registerUndefinedSizePointCloud(mpcir));

    scene->updateBoundingBox();
    if(print_console)
    {
        QString str;
        str.append("The processed cloud was written to the output ");
        QString number = toQString(cloud_in->points.size());
        str.append(number);
        str.append(" points.");
        PS_LOG->addInfoMessage(step, str);
    }
    grpCpy_grp->addItemDrawable(scene);
}


void SimpleTreeStep::copy_attribute(PointS &from, PointS &to, PointAtrributeType type)
{
    switch (type) {
    case (PointAtrributeType::EIGEN1):
        to.eigen1 = from.eigen1;
        break;
    case (PointAtrributeType::EIGEN2):
        to.eigen2 = from.eigen2;
        break;
    case (PointAtrributeType::EIGEN3):
        to.eigen3 = from.eigen3;
        break;
    case (PointAtrributeType::EIGENALL):
        if(from.eigen1 >= 0  && from.eigen2>= 0 && from.eigen3>= 0 && from.eigen1 <= 1  && from.eigen2<= 1 && from.eigen3 <= 1)
        {
            to.eigen1 = from.eigen1;
            to.eigen2 = from.eigen2;
            to.eigen3 = from.eigen3;
        } else {
            to.eigen1 = -1;
            to.eigen2 = -1;
            to.eigen3 = -1;
        }
        break;
    case (PointAtrributeType::NORMALX):
        to.normal_x = from.normal_x;
        break;
    case (PointAtrributeType::NORMALY):
        to.normal_y = from.normal_y;
        break;
    case (PointAtrributeType::NORMALZ):
        to.normal_z = from.normal_z;
        break;
    case (PointAtrributeType::NORMAL):
        to.normal_x = from.normal_x;
        to.normal_y = from.normal_y;
        to.normal_z = from.normal_z;
        break;
    case (PointAtrributeType::TREEID):
        to.treeID = from.treeID;
        break;
    case (PointAtrributeType::ID):
        to.ID = from.ID;
        break;
    case (PointAtrributeType::CLUSTER):
        to.cluster = from.cluster;
        break;
    case (PointAtrributeType::RANGEBIN):
        to.rangebin = from.rangebin;
        break;
    case (PointAtrributeType::DISTANCE):
        to.distance = from.distance;
        break;
    case (PointAtrributeType::TRUEDISTANCE):
        to.true_distance = from.true_distance;
        break;
    case (PointAtrributeType::VISITED):
        to.was_visited = from.was_visited;
        break;
    case (PointAtrributeType::ALL):
        to.eigen1 = from.eigen1;
        to.eigen2 = from.eigen2;
        to.eigen3 = from.eigen3;
        to.normal_x = from.normal_x;
        to.normal_y = from.normal_y;
        to.normal_z = from.normal_z;
        to.treeID = from.treeID;
        to.ID = from.ID;
        to.cluster = from.cluster;
        to.rangebin = from.rangebin;
        to.distance = from.distance;
        to.true_distance = from.true_distance;
        to.was_visited = from.was_visited;
        break;
    default:
        break;
    }

}

void SimpleTreeStep::pcl_transfer_attribute(PointCloudS::Ptr cloud_low_res, PointCloudS::Ptr cloud_high_res, PointAtrributeType type)
{
    pcl::KdTreeFLANN<PointS> kdtree;
    kdtree.setInputCloud (cloud_low_res);
    for(size_t i = 0; i < cloud_high_res->points.size(); i++)
    {
        PointS search_point = cloud_high_res->points.at(i);
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        if ( kdtree.nearestKSearch (search_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (size_t j = 0; j < pointIdxNKNSearch.size (); ++j)
            {
                PointS reference_point = cloud_low_res->points[ pointIdxNKNSearch[j] ];
                copy_attribute(reference_point, search_point, type);
                cloud_high_res->points[i] = search_point;
            }
        }
    }
}






