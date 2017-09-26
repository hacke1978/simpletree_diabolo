#ifndef TREE_H
#define TREE_H

#include <limits>

#include <QObject>
#include <QSharedPointer>
#include <QWeakPointer>
#include <QVector>
#include <QDebug>

#include "segment.h"


class Tree
{

private:
    /**
     * @brief treeID A unique String for the Tree ID
     */
    QString _treeID;
    /**
     * @brief _height_above_ground The height above ground where the tree was cut;
     */
    float _height_above_ground = 0.1f;

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
      * @brief get_stem_cylinders Returns all cylinders belonging to the stem
      * @return The stem cylinders;
      */
     const QVector<QSharedPointer<Cylinder> >
     get_stem_cylinders() const;

public:
     /**
      * @brief volume_to_root Follows the path from the Segment to the root and retrieves the volume of this path
      * @param segment A segment from which the volume should be calculated
      * @return The Volume to the root
      */
     const
     float get_volume_to_base(const QSharedPointer<Segment> segment, const QSharedPointer<Segment> root) const;
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
    get_height() const;

    /**
     * @brief get_length Returns the length of the tree
     * @return The length of the tree
     */
    const float
    get_length() const;


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
    Tree(QSharedPointer<Segment> root_segment,  QString treeID = "tree", float height_above_ground = 0.1f);

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
