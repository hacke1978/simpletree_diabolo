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

#ifndef PREDICTSTABLEVOLUME_H
#define PREDICTSTABLEVOLUME_H

#include "SimpleTree4/method/method_coefficients.h"
#include "SimpleTree4/model/pointsimpletree.h"
#include "SimpleTree4/math/simplemath.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>

#include <QDebug>



class PredictStableVolume
{
    PointCloudS::Ptr _cloud;

    MethodCoefficients _coeff;

    float _minZ = std::numeric_limits<float>::max();

    void predict_height();

    void predict_circumference();

    void predict_volume();

    PointCloudS::Ptr extract_low_cluster();

    PointCloudS::Ptr filter_low_cluster(PointCloudS::Ptr cloud_unfiltered);

    float predict_cylinder_radius(PointCloudS::Ptr cloud);

    float get_circumference_from_radius(float radius);

public:

    PredictStableVolume(PointCloudS::Ptr cloud, MethodCoefficients coeff);

    MethodCoefficients get_coeff() const;
};

#endif // PREDICTSTABLEVOLUME_H
