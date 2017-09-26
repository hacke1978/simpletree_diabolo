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

#include "voxelgridfilter.h"



const QVector<PointCloudS::Ptr> VoxelGridFilter::extract_sub_clouds()
{
    QVector<PointCloudS::Ptr> output;
    PointS p1;
    PointS p2;
    pcl::getMinMax3D<PointS>(*_cloud_in,p1,p2);

    float xMin = p1.x;
    float xMax = p2.x;

    float yMin = p1.y;
    float yMax = p2.y;

    float zMin = p1.z;
    float zMax = p2.z;


    float xCurrent= xMin;
    while(xCurrent < xMax)
    {
        float yCurrent = yMin;
        while(yCurrent < yMax)
        {
            float zCurrent = zMin;
            while(zCurrent < zMax)
            {
                PointS min (xCurrent,yCurrent,zCurrent);
                PointS max (xCurrent + _split_size,yCurrent + _split_size,zCurrent + _split_size);
                PointCloudS::Ptr sub_cloud = extract_cloud_in_box(min,max);
                output.push_back(sub_cloud);
                zCurrent += _split_size;
            }
            yCurrent += _split_size;
        }
        xCurrent += _split_size;
    }
    return output;
}

PointCloudS::Ptr VoxelGridFilter::get_cloud_out() const
{
    return _cloud_out;
}

void VoxelGridFilter::compute()
{
    QVector<PointCloudS::Ptr> clouds = extract_sub_clouds();
    down_sample(clouds);
}

PointCloudS::Ptr VoxelGridFilter::extract_cloud_in_box(PointS min, PointS max)
{
    PointCloudS::Ptr sub_cloud (new PointCloudS);
    std::vector<int> pointIdxBoxSearch;
    Eigen::Vector3f min_pt ( min.x, min.y, min.z );
    Eigen::Vector3f max_pt ( max.x, max.y, max.z );
    _octree->boxSearch ( min_pt, max_pt, pointIdxBoxSearch );
    pcl::ExtractIndices<PointS> extract;
    pcl::PointIndices::Ptr inliers ( new pcl::PointIndices () );
    inliers->indices = pointIdxBoxSearch;
    extract.setInputCloud ( _cloud_in );
    extract.setIndices ( inliers );
    extract.setNegative ( false );
    extract.filter ( *sub_cloud );
    return sub_cloud;
}

PointCloudS::Ptr VoxelGridFilter::extract_cloud_in_box2(PointS min, PointS max)
{
        PointCloudS::Ptr sub_cloud (new PointCloudS);
//    pcl::ConditionAnd<PointS>::Ptr range_cond (new
//         pcl::ConditionAnd<PointS> ());
//       range_cond->addComparison (pcl::FieldComparison<PointS>::ConstPtr (new
//         pcl::FieldComparison<PointS> ("z", pcl::ComparisonOps::GE, min.z))); //Check if GE is right, copied and pasted from tutorial and tried to replace GT (Greater than) by Greater or Equal
//       range_cond->addComparison (pcl::FieldComparison<PointS>::ConstPtr (new
//         pcl::FieldComparison<PointS> ("z", pcl::ComparisonOps::LT, max.z)));
//       // build the filter
//       pcl::ConditionalRemoval<PointS> condrem;
//       condrem.setCondition (range_cond);
//       condrem.setInputCloud (_cloud_in);
//       condrem.setKeepOrganized(true); //Check this one too, I do not work with organized clouds...
//       // apply filter
//       condrem.filter (*sub_cloud);
        return sub_cloud;
}

void VoxelGridFilter::down_sample(QVector<PointCloudS::Ptr> clouds)
{
    _cloud_out.reset(new PointCloudS);
    QVectorIterator<PointCloudS::Ptr> it( clouds);
    while(it.hasNext())
    {
        PointCloudS::Ptr downsampled_cloud (new PointCloudS);
        PointCloudS::Ptr sub_cloud = it.next();
        pcl::VoxelGrid<PointS> sor;
        sor.setInputCloud ( sub_cloud);
        sor.setLeafSize ( _cell_size, _cell_size, _cell_size );
        sor.filter ( *downsampled_cloud );
        *_cloud_out += *downsampled_cloud;
    }
}

VoxelGridFilter::VoxelGridFilter(PointCloudS::Ptr cloud, float cell_size, float split_size)
{
    _cloud_in = cloud;
    _cell_size = cell_size;
    _split_size = split_size;
    _octree.reset(new pcl::octree::OctreePointCloudSearch<PointS>(SimpleMath<float>::_OCTREE_RESOLUTION));
    _octree->setInputCloud(_cloud_in);
    _octree->addPointsFromInputCloud();
}

