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

#include "stem_taper.h"


MethodCoefficients Stem_Taper::coeff() const
{
    return _coeff;
}

void Stem_Taper::compute()
{
    float dbh = 0;
    {
        QVector<QSharedPointer<Cylinder> > cylinders = _tree->get_stem_cylinders();
        {
            QSharedPointer<Cylinder> dbh_cylinder = cylinders.at(0);
            dbh = dbh_cylinder->get_radius();
            float dbh_z = dbh_cylinder->get_start().z + (1.3f - _coeff.cut_height);
            QVectorIterator<QSharedPointer<Cylinder> > git(cylinders);
            while(git.hasNext())
            {
                QSharedPointer<Cylinder>  cyl = git.next();
                float start_z = cyl->get_start().z;
                float end_z = cyl->get_end().z;
                if(start_z <= dbh_z && end_z >=dbh_z)
                {
                    dbh = cyl->get_radius();
                }
            }
        }
    }
    QVector<QSharedPointer<Cylinder> > stem_cylinders = _tree->get_stem_cylinders();
    float z = stem_cylinders.at(stem_cylinders.size()-1)->get_end().z;
    float height = _coeff.cut_height + z - stem_cylinders.at(stem_cylinders.size()-1)->get_start().z;
    QList<Eigen::Vector2d*> taper_values;
    for(int i = 0; i < stem_cylinders.size(); i++)
    {
        QSharedPointer<Cylinder> cylinder = stem_cylinders.at(i);
        if(cylinder->get_detection()==DetectionType::SPHEREFOLLOWING)
        {
            float cylinder_radius = cylinder->get_radius();
            float z = cylinder->get_center().z;
            float cylinder_height = _coeff.cut_height + z - stem_cylinders.at(0)->get_start().z;


            Eigen::Vector2d * vec (new Eigen::Vector2d);
            *vec << (double) ((cylinder_height)/height), (double) (cylinder_radius/dbh);
            taper_values.push_back(vec);
        }
    }
    if(taper_values.size()>=5)
    {
        CT_MathFittedLine2D math_line(taper_values);
        _coeff.taper_a = math_line._a;
        _coeff.taper_b = math_line._b;
        for(int i = 0; i < stem_cylinders.size(); i++)
        {
            QSharedPointer<Cylinder> cylinder = stem_cylinders.at(i);
            float z = cylinder->get_center().z;
            float cylinder_height = _coeff.cut_height + z - stem_cylinders.at(0)->get_start().z;
            float hr = (cylinder_height)/height;
            float dr = _coeff.taper_a*hr + _coeff.taper_b;
            float rad = dr*dbh;
            if(cylinder->get_radius()< 0.85*rad || cylinder->get_radius()>1.15*rad )
            {
                cylinder->set_radius(rad);
                cylinder->set_allometry_improvement(AllometryImproved::ALLOM_TAPER);
            }
        }
        QVector<QSharedPointer<Cylinder> > cylinders = _tree->get_all_cylinders();
        for(int i = 0; i < cylinders.size(); i++)
        {
            QSharedPointer<Cylinder> cylinder = cylinders.at(i);
            if(cylinder->get_detection()==DetectionType::ATTRACTOR )
            {
                cylinder->set_radius(0);
            }
        }
    ImproveByPipeModel pype(_tree, false, 0.5);
    } else {
    }
}

float Stem_Taper::get_DBH_from_taper(QSharedPointer<Tree> tree, MethodCoefficients cf)
{
    QVector<QSharedPointer<Cylinder> > stem_cylinders = tree->get_stem_cylinders();
    QList<Eigen::Vector2d*> taper_values;
    float min_z = stem_cylinders.at(0)->get_start().z;
    float cut_height = cf.cut_height;
    for(int i = 0; i < stem_cylinders.size(); i++)
    {
        QSharedPointer<Cylinder> cylinder = stem_cylinders.at(i);
        if(cylinder->get_detection()==DetectionType::SPHEREFOLLOWING)
        {
            float cylinder_radius = cylinder->get_radius();
            float z = cylinder->get_center().z;
            float cylinder_height = cut_height + z - min_z;
            Eigen::Vector2d * vec (new Eigen::Vector2d);
            *vec << (double) ((cylinder_height)), (double) (cylinder_radius);
            taper_values.push_back(vec);
        }
    }
    int size = taper_values.size();
    float dbh = 0;
    if(taper_values.size()>=5)
    {
        CT_MathFittedLine2D math_line(taper_values);
        float taper_a = math_line._a;
        float taper_b = math_line._b;
        dbh = 1.3*taper_a + taper_b;

    }
    if(dbh<0)
        dbh = 0;
    return dbh;

}

Stem_Taper::Stem_Taper(QSharedPointer<Tree> tree, MethodCoefficients coeff)
{
    _tree = tree;
    _coeff = coeff;
    compute();
}
