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

#ifndef COMPUTEALLOMETRY_H
#define COMPUTEALLOMETRY_H

#include <SimpleTree4/model/tree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "ct_math/ct_mathfittedline2d.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>



class ComputeAllometry
{
    QSharedPointer<Tree> _tree;
    bool _ignore_leave_segments;
    bool _compute_all_data;

    float _min_radius;
    float _min_volume;

    float _a;
    float _b;

    float _a_length;
    float _b_length;


    Eigen::MatrixXd generate_Jacobian(QVector<float>& rad);

    Eigen::MatrixXd generate_value_matrix(QVector<float>& rad, QVector<float>& vol);

    void gauss_newton(QVector<float>& rad, QVector<float>& vol);

    void compute();

    void compute_min_rad_vol();

    void generate_vectors(QVector<float> &rad, QVector<float> &vol, QVector<float> &rad_2, QVector<float> &vol_2);


public:
    ComputeAllometry(QSharedPointer<Tree> tree, bool compute_all_data = false, float min_radius = 0.05, bool ignore_leave_segments = true);
    float get_a() const;
    void set_a(float a);
    float get_b() const;
    void set_b(float b);
};

#endif // COMPUTEALLOMETRY_H
