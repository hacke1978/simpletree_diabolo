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

#ifndef EXPORT_H
#define EXPORT_H

#include <QSharedPointer>
#include <QVector>
#include <QString>
#include <QDir>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>

#include "SimpleTree4/model/tree.h"

#include "SimpleTree4/method/geometrical_operations/stem_taper.h"
class Export
{

public:
    Export();

    static void export_ply_color(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution);

    static void export_tree_detail(QSharedPointer<Tree> tree, QString path, QString file_name, MethodCoefficients coeff);

    static void export_tree_list(QVector<QSharedPointer<Tree> > tree_list, QString path, QVector<MethodCoefficients> coeff_list);

    static void export_coefficients(QString path, QVector<MethodCoefficients> coeff_list);

    static void export_ply(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution = 8);

    static void export_ply_stem(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution = 8);

    static void export_ply_stem_all(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution = 8);

    static void export_ply_allom(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution = 8);

    static void export_ply_detection(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution = 8);

        static void export_ply_good(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution = 8);

        static void export_ply_bad(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution = 8);

        static void export_ply_leave(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution = 8);

        static void export_ply_no_leave(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution = 8);

    static void export_ply_improvement(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution = 8);

    static PointCloudS::Ptr generate_points_from_cylinder_list(QVector<QSharedPointer<Cylinder> > cylinders, float resolution);

    static PointCloudS::Ptr generate_points_from_cylinder(QSharedPointer<Cylinder>  cylinder, float resolution);
};

#endif // EXPORT_H
