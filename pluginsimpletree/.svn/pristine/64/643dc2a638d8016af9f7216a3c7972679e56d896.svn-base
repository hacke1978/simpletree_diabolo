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

#include "exporttree.h"

ExportTree::ExportTree(QSharedPointer<Tree> tree, MethodCoefficients coeff, QString path, bool extend_str)
{
    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    QString file_name = coeff.id;
    QStringList file_list = file_name.split(".");

    QString file_id = file_list.at(0);
    if(extend_str)
    {
        file_id.append("_opposite_allom");
    }
    QString output = file_id.append(".csv");
    QString full_path = path.append("/").append(output);

    QFile file(full_path);

    if(file.open(QIODevice::WriteOnly))
    {
        QTextStream out(&file);
        out << "branch_ID;branch_order;segment_ID;parent_segment_ID;growth_volume;species;tree_ID;detection_type;improvement_type;startX;startY;startZ;endX;endY;endZ;radius;length;a;b;sd;mean;inlier_distance_circle;inlier_distance;iterations;ransac_circle_type;ransac_type;sphereradius_multiplier;epsilon_cluster_stem;epsilon_cluster_branch;epsilon_sphere;min_radius_sphere;min_dist;height;cd;stable_vol\n";
        QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
        QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
        while(it.hasNext())
        {
            QSharedPointer<Cylinder>  cyl = it.next();
            out << tree->to_string(cyl,coeff);
        }
        file.close();

    }
}
