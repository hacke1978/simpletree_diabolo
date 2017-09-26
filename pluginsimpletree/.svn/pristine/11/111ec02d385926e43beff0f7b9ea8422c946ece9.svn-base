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

#include "spheresurfaceextraction.h"

SphereSurfaceExtraction::SphereSurfaceExtraction()
{

}

const PointCloudS::Ptr SphereSurfaceExtraction::extract_sphere_surface(QSharedPointer<pcl::octree::OctreePointCloudSearch<PointS> > octree,
                                                                       pcl::ModelCoefficients &sphere, PointCloudS::Ptr cloud_in, float epsilon_sphere)
{

    PointCloudS::Ptr cloud_out (new PointCloudS);
    PointS searchPoint;
    searchPoint.x = sphere.values[0];
    searchPoint.y = sphere.values[1];
    searchPoint.z = sphere.values[2];
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float radius = sphere.values[3] + epsilon_sphere;
    float minSquaredDist = (sphere.values[3] - epsilon_sphere)
            * (sphere.values[3] - epsilon_sphere);
    float maxSquaredDist = (sphere.values[3] + epsilon_sphere)
            * (sphere.values[3] + epsilon_sphere);
    octree->radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
            pointRadiusSquaredDistance);


    for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++) {
        if (pointRadiusSquaredDistance[i] > minSquaredDist
                && pointRadiusSquaredDistance[i] < maxSquaredDist) {
            cloud_out->push_back(cloud_in->points[pointIdxRadiusSearch[i]]);
        }
        octree->deleteVoxelAtPoint(pointIdxRadiusSearch[i]);
    }
    return cloud_out;
}
