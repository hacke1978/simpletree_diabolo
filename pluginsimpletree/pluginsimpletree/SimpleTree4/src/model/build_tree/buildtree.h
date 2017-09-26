#ifndef BUILDTREE_H
#define BUILDTREE_H

#include "src/model/tree.h"

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <QDebug>

class BuildTree
{
    /**
     * @brief _octree An Octree to store the start points into
     */
    QSharedPointer<pcl::octree::OctreePointCloudSearch<PointI> > _octree;
    /**
     * @brief _root_segment The root Segment
     */
    QSharedPointer<Segment> _root_segment;

    /**
     * @brief _coeff A list of cylinder coefficients
     */
    std::vector<pcl::ModelCoefficients> _coeff;


    /**
     * @brief _cylinders A Vector of cylinders derived from the coefficients
     */
    QVector<QSharedPointer<Cylinder> > _cylinders;

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
    BuildTree(std::vector<pcl::ModelCoefficients> coeff);
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
