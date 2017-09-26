#ifndef IMPROVEFIT_H
#define IMPROVEFIT_H

#include "src/model/tree.h"

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "SimpleTree4/method/method_coefficients.h"

class ImproveFit
{
private:
    QSharedPointer<Tree> _tree;
    QSharedPointer<pcl::octree::OctreePointCloudSearch<PointI> > _octree;
    PointCloudI::Ptr _cloud;

    const static float _OCTREE_RESOLUTION;
    const static int _MIN_PTS_CYLINDER_IMPROVEMENT;
    const static float _MIN_DIST_TO_CYLINDER;
    const static float _CYLINDER_SIZE_MULTIPLIER;
    const static float _MIN_RADIUS_MULTIPLIER;
    const static float _MAX_RADIUS_MULTIPLIER;

public:
    ImproveFit(QSharedPointer<Tree> tree, PointCloudI::Ptr cloud);

    /**
     * @brief extract_points_near_cylinder Returns a point cloud with the points near the cylinder
     * @param cylinder the input cylinder
     * @return the cloud
     */
    PointCloudI::Ptr
    extract_points_near_cylinder(QSharedPointer<Cylinder> cylinder);
    /**
     * @brief improve Iterates through all the cylinders and improves those by either RANSAC or Median method
     */
    void
    improve();

    /**
     * @brief RANSAC does a RANSAC Improvement
     * @param cloud the input cloud
     * @return the cylinder RANSAC coefficients
     */
    pcl::ModelCoefficients
    RANSAC(PointCloudI::Ptr cloud);

    /**
     * @brief improve_with_RANSAC Applies the RANSAC coeffiencts to the cylinder
     * @param cylinder the original cylinder
     * @param coeff the RANSAC coefficients
     */
    void
    improve_with_RANSAC(QSharedPointer<Cylinder> cylinder, pcl::ModelCoefficients coeff);

    /**
     * @brief improve_with_median Improves the radius of the cylinder with the median method
     * @param cylinder the input cylinder
     * @param cloud the cloud to compute the median from.
     */
    void
    improve_with_median(QSharedPointer<Cylinder> cylinder, PointCloudI::Ptr cloud);
};

#endif // IMPROVEFIT_H
