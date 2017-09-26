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

#include "readcoeff.h"



ReadCoeff::ReadCoeff(QString path)
{
    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    QFile file(path);
    if(!file.open(QIODevice::ReadOnly)) {
        QMessageBox::information(0, "error in ReadCSV", file.errorString());
    }
    QTextStream in (&file);
    in.readLine();
    while(!in.atEnd())
    {
        MethodCoefficients coeff;

        QString line = in.readLine();
        QStringList fields = line.split(";");
        coeff.min_fitted_distance = fields.at(0).toFloat();
        coeff.sphere_radius_multiplier = fields.at(1).toFloat();
        coeff.epsilon_cluster_stem = fields.at(2).toFloat();
        coeff.epsilon_cluster_branch = fields.at(3).toFloat();
        coeff.epsilon_sphere = fields.at(4).toFloat();

        coeff.minPts_improve_cylinder = fields.at(5).toInt();
        coeff.minPts_ransac_branch = fields.at(6).toInt();
        coeff.minPts_cluster_stem = fields.at(7).toInt();
        coeff.minPts_cluster_branch = fields.at(8).toInt();
        coeff.min_radius_sphere = fields.at(9).toFloat();

        coeff.height_start_sphere = fields.at(10).toFloat();
        coeff.ransac_circle_type = fields.at(11).toInt();
        coeff.ransac_circle_inlier_distance = fields.at(12).toFloat();
        coeff.cut_height = fields.at(13).toFloat();
        coeff.number_clusters_for_spherefollowing = fields.at(14).toInt();

        coeff.use_dhs = fields.at(15).toInt();
        coeff.clustering_distance = fields.at(16).toFloat();
        coeff.tree_height = fields.at(17).toFloat();
        coeff.tree_circumference = fields.at(18).toFloat();
        coeff.tree_predicted_volume = fields.at(19).toFloat();
        coeff.tree_max_angle = fields.at(20).toFloat();
        coeff.percentage_for_attractor = fields.at(21).toFloat();

        coeff.use_allom = fields.at(22).toInt();
        coeff.a = fields.at(23).toFloat();
        coeff.b = fields.at(24).toFloat();
        coeff.minRad = fields.at(25).toFloat();
        coeff.ransac_type = fields.at(26).toInt();
        coeff.ransac_inlier_distance = fields.at(27).toFloat();
        coeff.ransac_iterations = fields.at(28).toFloat();
        coeff.ransac_median_factor = fields.at(29).toFloat();
        coeff.min_dist = fields.at(30).toDouble();

        coeff.factor = fields.at(31).toFloat();
        coeff.sd = fields.at(32).toFloat();
        coeff.mean = fields.at(33).toFloat();
        coeff.sd_mult = fields.at(34).toInt();
        coeff.optimze_stem = fields.at(35).toInt();

        coeff.radius_multiplier_cylinder_test = fields.at(36).toFloat();
        coeff.times_cluster_extension = fields.at(37).toInt();
        coeff.max_times_cluster_extension = fields.at(38).toInt();
        coeff.min_ratio_pype = fields.at(39).toFloat();
        coeff.max_ratio_pype = fields.at(40).toFloat();

        coeff.min_radius_for_pype_test = fields.at(41).toFloat();
        coeff.max_number_failure_segments = fields.at(42).toInt();
        coeff.id = fields.at(43);
        coeff.species = fields.at(44);
        coeff.outputpath = fields.at(45);

        _coeff.push_back(coeff);
    }
    file.close();
}

QVector<MethodCoefficients> ReadCoeff::get_coeff() const
{
    return _coeff;
}
