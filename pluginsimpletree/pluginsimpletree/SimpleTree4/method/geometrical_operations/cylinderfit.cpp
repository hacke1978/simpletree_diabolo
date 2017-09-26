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

#include "cylinderfit.h"

const float CylinderFit::_MIN_RADIUS_MULTIPLIER = 0.5f;
const float CylinderFit::_MAX_RADIUS_MULTIPLIER = 3.5f;

CylinderFit::CylinderFit()
{

}

const pcl::ModelCoefficients CylinderFit::get_cylinder_from_circles(pcl::ModelCoefficients circle1, pcl::ModelCoefficients circle2)
{
    pcl::ModelCoefficients cylinder;
    cylinder.values.resize(7);
    cylinder.values[0] = circle1.values[0];
    cylinder.values[1] = circle1.values[1];
    cylinder.values[2] = circle1.values[2];
    cylinder.values[3] = circle2.values[0] - circle1.values[0];
    cylinder.values[4] = circle2.values[1] - circle1.values[1];
    cylinder.values[5] = circle2.values[2] - circle1.values[2];
    cylinder.values[6] = circle2.values[3];
    return cylinder;
}

const pcl::ModelCoefficients CylinderFit::ransac_cylinder_fit(PointCloudS::Ptr cloud, int type, float distance, int max_iterations)
{
    pcl::ModelCoefficients coeff;
    pcl::SACSegmentationFromNormals<PointS, PointS> seg;
    seg.setOptimizeCoefficients ( true );
    seg.setModelType ( pcl::SACMODEL_CYLINDER );
    seg.setMethodType( type);

    seg.setNormalDistanceWeight (0 );
    seg.setMaxIterations ( max_iterations );
    seg.setDistanceThreshold ( distance );
    seg.setRadiusLimits ( 0, 2.2 );
    seg.setInputCloud ( cloud );
    seg.setInputNormals ( cloud );
    pcl::PointIndices::Ptr inliers_cylinder ( new pcl::PointIndices );
    seg.segment ( *inliers_cylinder, coeff );
    return coeff;
}


const void CylinderFit::median_cylinder_fit(QSharedPointer<Cylinder> cylinder, PointCloudS::Ptr cloud)
{
    QVector<float> distances;
    distances.resize(cloud->points.size());
    for(size_t i = 0; i < cloud->points.size(); i++)
    {
        PointS p = (cloud->points[i]);
        QSharedPointer<PointS> pptr (new PointS);
        pptr->x = p.x;
        pptr->y = p.y;
        pptr->z = p.z;

        distances[i] = cylinder->dist_to_axis(pptr);
    }
    float median = SimpleMath<float>::get_median(distances);
    cylinder->set_radius(median);
}

const void CylinderFit::general_cylinder_fit(PointCloudS::Ptr cloud, QSharedPointer<Cylinder> cylinder, int minPts, int type, float distance, int max_iterations)
{
    AllometryImproved ali = cylinder->get_allometry_improvement();
    DetectionType dt = cylinder->get_detection();
    std::vector<float> cf_vec = cylinder->values;
    pcl::ModelCoefficients cf;
    cf.values.push_back(cf_vec.at(0));
    cf.values.push_back(cf_vec.at(1));
    cf.values.push_back(cf_vec.at(2));
    cf.values.push_back(cf_vec.at(3));
    cf.values.push_back(cf_vec.at(4));
    cf.values.push_back(cf_vec.at(5));
    cf.values.push_back(cf_vec.at(6));
    QSharedPointer<Cylinder> copy (new Cylinder(cf));


    std::vector<float> coeff_unimproved = cylinder->values;

    if(cloud->points.size() > minPts)
    {
          pcl::ModelCoefficients ransac_coeff = ransac_cylinder_fit(cloud,type,distance,max_iterations);

        if(ransac_coeff.values.size()==7)
        {
            QSharedPointer<Cylinder> ransac (new Cylinder(ransac_coeff));

            QSharedPointer<PointS> start = cylinder->get_start_ptr();
            QSharedPointer<PointS> end  = cylinder->get_end_ptr();

            QSharedPointer<PointS> start2 =  ransac->projection_on_axis_ptr(start);
            QSharedPointer<PointS> end2 =  ransac->projection_on_axis_ptr(end);



            if( (copy->dist_to_axis(start2)>(copy->get_radius())*3.5)
                                 || (copy->dist_to_axis(end2)>(copy->get_radius())*3.5)
                                 || SimpleMath<float>::angle_between(cylinder->get_axis(),copy->get_axis())>40)
            {
                cylinder->values = coeff_unimproved;
                cylinder->set_improvement(ImprovementType::NO);
                cylinder->set_detection(dt);
                cylinder->set_allometry_improvement(ali);

            } else {

                cylinder->set_start_end(start2,end2);
                cylinder->set_radius(std::abs(ransac->get_radius()));
                cylinder->set_improvement(ImprovementType::RANSAC);
                cylinder->set_detection(DetectionType::SPHEREFOLLOWING);
                cylinder->set_allometry_improvement(AllometryImproved::NOALLOM);
            }
        }
    }
    else
    {
        cylinder->set_improvement(ImprovementType::NO);
    }
}

const void CylinderFit::advanced_cylinder_fit(PointCloudS::Ptr cloud, QSharedPointer<Cylinder> cylinder, MethodCoefficients coeff)
{

}
