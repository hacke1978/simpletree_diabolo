#ifndef POINTSIMPLETREE_H
#define POINTSIMPLETREE_H

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

struct PointSimpleTree
{
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  float is_stem;
  float eigen1;
  float eigen2;
  float eigen3;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointSimpleTree,
                                   (float, x, x)
                                   (float, y, y)                                   
                                   (float, z, z)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (float, is_stem, is_stem)
                                   (float, eigen1, eigen1)
                                   (float, eigen2, eigen2)
                                   (float, eigen3, eigen3)
)

#endif // POINTSIMPLETREE_H
