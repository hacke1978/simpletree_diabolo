/****************************************************************************

 Copyright (C) 2016-2017 Jan Hackenberg
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

#ifndef TREE_H
#define TREE_H

#include <limits>

#include <QObject>
#include <QSharedPointer>
#include <QWeakPointer>
#include <QVector>
#include <QDebug>

#include "segment.h"

#include<SimpleTree4/method/method_coefficients.h>


class Tree
{

private:
    /**
     * @brief treeID A unique String for the Tree ID
     */
    QString _treeID;
//    /**
//     * @brief _height_above_ground The height above ground where the tree was cut;
//     */
//    float _height_above_ground = 0.1f;

    /**
     * @brief _root_segment The root Segment
     */
    QSharedPointer<Segment> _root_segment;




    /**
     * @brief get_min_height The minimum height of the model
     * @return The minimum height
     */
    const float get_min_height() const;
    /**
     * @brief get_max_height The maximum z-coordinate of the model
     * @return The maximum height
     */
    const float get_max_height() const;

    /**
     * @brief get_cylinder_direction creates a cylinder with max radius and extension from bottom to save_height
     * @param save_height maximum height above min z for cylinders to be counted
     * @return the direction cylinder
     */
    QSharedPointer<Cylinder> get_cylinder_direction( QVector<QSharedPointer<Cylinder> > stem_cylinders, float save_height = 5);



public:

    QSharedPointer<Cylinder>  get_cylinder_in_height(float height, MethodCoefficients cf);


    QSharedPointer<Tree>
    clone();

    /**
      * @brief get_stem_cylinders Returns all cylinders belonging to the stem
      * @return The stem cylinders;
      */
     const QVector<QSharedPointer<Cylinder> >
     get_stem_cylinders() const;


     QSharedPointer<Cylinder> get_parent(QSharedPointer<Cylinder> cyl);

     QSharedPointer<Cylinder> get_child(QSharedPointer<Cylinder> cyl);

      QVector<QSharedPointer<Cylinder> > get_children_non_recursive(QSharedPointer<Cylinder> cyl);

     QVector<QSharedPointer<Cylinder> > get_neighbor_cylinders(QSharedPointer<Cylinder> cyl, int neighborhoodsize = 2);

     QVector<QSharedPointer<Cylinder> > get_stem_cylinders_save();


     /**
      * @brief get_pcl_coefficients returns all cylinders as pcl modelcoefficients
      * @return the vector of pcl::ModelCoefficients
      */
     QVector<pcl::ModelCoefficients> get_pcl_coefficients() const;


     QString
     to_string(const QSharedPointer<Cylinder> &  cylinder,const MethodCoefficients&   coefficients);

     /**
      * @brief volume_to_root Follows the path from the Segment to the root and retrieves the volume of this path
      * @param segment A segment from which the volume should be calculated
      * @return The Volume to the root
      */
     const
     float get_volume_to_base(const QSharedPointer<Segment> segment, const QSharedPointer<Segment> root) const;

     /**
      * @brief get_segments_between returns the list of all segments between base and leave
      * @param base the base segment
      * @param leave the leave segment
      * @return the list between base and leafe
      */
     const QVector<QSharedPointer<Segment> > get_segments_between(const QSharedPointer<Segment> base, const QSharedPointer<Segment> leave) const;

     /**
      * @brief get_sum_length returns the summed up length of all contained segments
      * @param segments the segments
      * @return the sum up length
      */
     const float get_sum_length(const QVector<QSharedPointer<Segment> > segments) const;

     /**
      * @brief get_length_to_leave returns the length to the leave
      * @param cylinder the cylinder
      * @return the length
      */
     const float get_length_to_leave(const QSharedPointer<Cylinder> cylinder) const;

     /**
      * @brief get_length_to_leave returns the length to the leave
      * @param cylinder the cylinder
      * @return the length
      */
     const float get_length_to_leave_save(const QSharedPointer<Cylinder> cylinder) const;

     /**
      * @brief get_branch_order_from_leave gets the difference between cylinders branch order and the cylinders maximum leav branch oder
      * @param cylinder the cylinder;
      * @return the inverse branch order;
      */
     const int get_branch_order_from_leave(const QSharedPointer<Cylinder> cylinder) const;

     /**
      * @brief get_branch_order_from_leave gets the difference between cylinders branch order and the cylinders maximum leav branch oder
      * @param cylinder the cylinder;
      * @return the inverse branch order;
      */
     const int get_branch_order_cumulative( const QSharedPointer<Segment> base) const;

     /**
      * @brief get_segments_longest_path returns a list of all segments along the longest path to leaves from segment base
      * @param base the base segment
      * @return  the longest path
      */
     const QVector<QSharedPointer<Segment> > get_segments_longest_path(const QSharedPointer<Segment> base) const;

     /**
      * @brief get_leave_segments Returns all leave segments growing out of this segment
      * @param base The query segment
      * @return All leave segments growing out of this segment
      */
     const
     QVector<QSharedPointer<Segment> > get_leave_segments(const QSharedPointer<Segment> base) const;

    /**
     * @brief get_height Returns the height of the tree
     * @return  The height
     */
    const float
    get_height(MethodCoefficients cf) const;

    /**
     * @brief get_length Returns the length of the tree
     * @return The length of the tree
     */
    const float
    get_length(MethodCoefficients cf) const;


    const float
    get_length_to_root(QSharedPointer<Cylinder> cylinder, MethodCoefficients cf);


    /**
     * @brief get_volume Returns the total volume of the tree;
     * @return The total volume of the tree;
     */
    const float
    get_volume() const;
    /**
     * @brief get_growth_volume Returns the volume of the cylinder itself and the volume of all cylinders with a child relation
     * @param cylinder The query cylinder
     * @return The growth Volume
     */
    const float
    get_growth_volume(const QSharedPointer<Cylinder> cylinder) const;

    /**
     * @brief get_growth_length Returns the length of this segment and the length of all its children
     * @param cylinder the query cylinder
     * @return The growth length
     */
    float
    get_growth_length(QSharedPointer<Cylinder> cylinder);


    /**
     * @brief get_child_cylinders Returns the recursively searched children of a base Cylinder
     * @param base_cylinder The Cylinder the recursive search starts from
     * @return All the children of the base cylinder
     */
    const QVector<QSharedPointer<Cylinder> > get_child_cylinders(const QSharedPointer<Cylinder> base_cylinder) const;

    /**
     * @brief get_child_segments Returns the recursively searched children of a Segment
     * @param base_segment The Segment the recursive search starts from
     * @return All the children of the base Segment
     */
    const QVector<QSharedPointer<Segment> >
    get_child_segments(const QSharedPointer<Segment> base_segment) const;

    /**
     * @brief get_all_cylinders Returns all the Cylinders of the tree
     * @return A vector containing all the tree cylinders
     */
    const QVector<QSharedPointer<Cylinder> >
    get_all_cylinders() const;


    /**
     * @brief get_all_segments Returns all the Segments of the tree
     * @return A vector of all Segments.
     */
    const QVector<QSharedPointer<Segment> >
    get_all_segments() const;

    /**
     * @brief Tree standard constructor
     */
    Tree(QSharedPointer<Segment> root_segment,  QString treeID);

    /**
     * @brief getTreeID Getter for the Tree ID
     * @return the tree ID
     */
    const QString getTreeID() const;

    /**
     * @brief setTreeID Setter for the Tree ID
     * @param value the tree ID;
     */
    void setTreeID(const QString &value);
    /**
     * @brief get_root_segment Returns the root segment
     * @return the root segment
     */
    QSharedPointer<Segment> get_root_segment() const;
};

#endif // TREE_H
