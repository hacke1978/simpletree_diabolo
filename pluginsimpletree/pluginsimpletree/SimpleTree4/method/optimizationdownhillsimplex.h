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

#ifndef OPTIMIZATIONDOWNHILLSIMPLEX_H
#define OPTIMIZATIONDOWNHILLSIMPLEX_H

#include <QVector>
#include <QGenericMatrix>
#include <QEnableSharedFromThis>

#include <SimpleTree4/method/spherefollowing2.h>
#include <SimpleTree4/method/workerdownhillsimplex.h>
#include <SimpleTree4/method/spherefollowingrecursive.h>
#include "SimpleTree4/model/build_tree/buildtree.h"
#include "SimpleTree4/model/build_tree/removefalsecylinders.h"

#include <Eigen/Core>

class SphereFollowingRecursive;


class OptimizationDownHillSimplex:  public QObject, public QEnableSharedFromThis<OptimizationDownHillSimplex>
{
    Q_OBJECT

    const int _NMAX = 200;

    const double TINY = 1.0e-10;

    MethodCoefficients _start;

    MethodCoefficients _end;

    MethodCoefficients _copy;

    PointCloudS::Ptr _cloud;

    double _distance_start;

    double _distance_end;

    bool _subdivide_stem_and_branch_points;

    MethodCoefficients convert_to_coefficients(Eigen::MatrixXd const &matrix, int i);

    const double ftol;

    int nfunc = 0;

    int _mpts;

    int _ndim;

    double fmin;

    QVector<double> y;

    Eigen::MatrixXd _matrix;

    void generate_coeff_from_point(QVector<double> &coeff);

    bool _is_multithreaded = true;

    QVector<double> _results;

    QVector<double> minimize(QVector<double> &point, QVector<double> &factors);

    QVector<double> minimize(MethodCoefficients coeff, const double del);

    QVector<double> minimize(Eigen::MatrixXd &matrix_copy);

    double evaluate(QVector<double> x);

    void swap(double x1, double x2); //check this

    void get_psum(Eigen::MatrixXd & matrix, QVector<double> &psum);

    double amotry(Eigen::MatrixXd & matrix, QVector<double> &y, QVector<double> &psum, const int ihi, const double fac);

    QString get_qstring_from_eigen(Eigen::MatrixXd &matrix_copy);

    QString get_qstring_from_vector(QVector<double> &vec);

    MethodCoefficients get_coefficients_from_matrix(Eigen::MatrixXd &matrix, int i);

signals:

    void emit_qstring(QString qstring);

    void emit_counter(int timer);


public slots:

    void sent_qstring(QString qstring);

    void sent_counter(int counter);

public:

    OptimizationDownHillSimplex(const double ftoll, MethodCoefficients coeff, PointCloudS::Ptr cloud,
                                bool subdivide_stem_and_branch_points, bool is_multithreaded = true);

    MethodCoefficients get_end_coeff() const;

    void minimize();

    void set_distance(int i, double d);

    void set_ndim(const int dim);


};

#endif // OPTIMIZATIONDOWNHILLSIMPLEX_H
