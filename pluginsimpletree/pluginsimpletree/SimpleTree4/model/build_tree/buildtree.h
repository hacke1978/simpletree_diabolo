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

#ifndef BUILDTREE_H
#define BUILDTREE_H

#include "../tree.h"

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <QDebug>

class BuildTree
{

    void
    allocate_cylinders();

    /**
     * @brief _octree An Octree to store the start points into
     */
    QSharedPointer<pcl::octree::OctreePointCloudSearch<PointS> > _octree;
    /**
     * @brief _root_segment The root Segment
     */
    QSharedPointer<Segment> _root_segment;

    /**
     * @brief _coeff A list of cylinder coefficients
     */
    QVector<pcl::ModelCoefficients> _coeff;


    /**
     * @brief _cylinders A Vector of cylinders derived from the coefficients
     */
    QVector<QSharedPointer<Cylinder> > _cylinders;


    /**
     * @brief remove_zero_length_cylinders Removes all the cylinders who have equal start and end point
     * Does not know why those cylinders occur.
     */
    void remove_zero_length_cylinders();

    /**
     * @brief generate_cylinders Generates cylinders from the modelcoefficients
     */
    void
    generate_cylinders();

    /**
     * @brief generate_octree Resets the octree and fills it with cylinders start points.
     */
    void
    generate_octree();
    /**
     * @brief get_child_cylinders Returns all Child Cylinders of the current cylinder
     * @param cylinder the current cylinder
     * @return The children of the cylinder
     */
    const QVector<QSharedPointer<Cylinder> > get_child_cylinders(const QSharedPointer<Cylinder> cylinder) const;

public:
    BuildTree(QVector<pcl::ModelCoefficients> coeff);
    /**
     * @brief build_tree Builds the tree recursively
     * @param cylinder The current cylinder
     * @param segment The current Segment
     */
    void
    build_tree(QSharedPointer<Cylinder> cylinder, QSharedPointer<Segment> segment);

    /**
     * @brief getRoot_segment Returns the root segment
     * @return The root segment
     */
    QSharedPointer<Segment> getRoot_segment() const;
};

#endif // BUILDTREE_H
