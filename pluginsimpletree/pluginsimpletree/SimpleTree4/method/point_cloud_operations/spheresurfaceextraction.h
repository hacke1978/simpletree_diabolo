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

#ifndef SPHERESURFACEEXTRACTION_H
#define SPHERESURFACEEXTRACTION_H

#include <SimpleTree4/model/pointsimpletree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/impl/octree_search.hpp>
#include <QVector>
#include <QSharedPointer>


class SphereSurfaceExtraction
{
public:
    SphereSurfaceExtraction();

    /**
     * @brief extract_sphere_surface Extracts from an input stored in an octree all spheres in the surface
     * @param octree The octree
     * @param sphere the sphere paramters
     * @param cloud_in The input cloud
     * @param epsilon_sphere The epsilon neighborhood of the sphere where the points should be extracted
     * @return The points on the surface
     */
    const static PointCloudS::Ptr extract_sphere_surface(QSharedPointer<pcl::octree::OctreePointCloudSearch<PointS> > octree,
    pcl::ModelCoefficients& sphere, PointCloudS::Ptr cloud_in, float epsilon_sphere);
};

#endif // SPHERESURFACEEXTRACTION_H


