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

#include "improvebyallometry.h"


void ImproveByAllometry::improve_cropped()
{
    QVector<float> growth_volumina;
    QVector<QSharedPointer<Cylinder> > cylinders = _tree->get_all_cylinders();
    {
        QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
        while(it.hasNext())
        {
            QSharedPointer<Cylinder> cylinder = it.next();
            float y = _tree->get_growth_volume(cylinder);
            growth_volumina.push_back(y);
        }
        for(size_t i = 0; i < cylinders.size(); i++)
        {
            QSharedPointer<Cylinder> cylinder = cylinders.at(i);
            float x = cylinder->get_radius();
            float y = growth_volumina.at(i);
            float x2 = get_x_from_y(y);
            float ratio = x/x2;

            if(!cylinder->get_segment()->is_root())
            {
                if(cylinder->get_allometry_improvement() == AllometryImproved::ALLOM && x > x2)
                {
                    cylinder->set_radius(get_x_from_y(y));
                    cylinder->set_allometry_improvement(AllometryImproved::ALLOM);
                    if(cylinder->get_radius()<_min_rad)
                    {
                        cylinder->set_radius(_min_rad);
                    }
                }
            }
        }
    }
    {
        growth_volumina.clear();
        QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
        while(it.hasNext())
        {
            QSharedPointer<Cylinder> cylinder = it.next();
            float y = _tree->get_growth_volume(cylinder);
            growth_volumina.push_back(y);
        }
        for(size_t i = 0; i < cylinders.size(); i++)
        {
            QSharedPointer<Cylinder> cylinder = cylinders.at(i);
            float x = cylinder->get_radius();
            float y = growth_volumina.at(i);
            float x2 = get_x_from_y(y);

            if(!cylinder->get_segment()->is_root())
            {
                if(cylinder->get_allometry_improvement() == AllometryImproved::ALLOM)
                {
                    cylinder->set_radius(get_x_from_y(y));
                    cylinder->set_allometry_improvement(AllometryImproved::ALLOM);
                    if(cylinder->get_radius()<_min_rad)
                    {
                        cylinder->set_radius(_min_rad);
                    }
                }
            }
        }
    }
}

void ImproveByAllometry::improve()
{
    //MethodCoefficients coeff;
    QVector<float> growth_volumina;
    QVector<QSharedPointer<Cylinder> > cylinders = _tree->get_all_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        float y = _tree->get_growth_volume(cylinder);
        growth_volumina.push_back(y);
    }
    float length = _tree->get_length(_cf);
    for(size_t i = 0; i < cylinders.size(); i++)
    {
        QSharedPointer<Cylinder> cylinder = cylinders.at(i);
        float x = cylinder->get_radius();
        float y = growth_volumina.at(i);
        float x2 = get_x_from_y(y);
        float ratio = x/x2;

        if(!cylinder->get_segment()->is_root())
        {
            if(y<(get_y_from_x(x)/_fac) )
            {
                if(ratio > _cf.ratio_min)
                {
                    cylinder->set_radius(get_x_from_y(y));
                    cylinder->set_allometry_improvement(AllometryImproved::ALLOM);
                } else {
                    if (length < 10){
                        cylinder->set_radius(get_x_from_y(y));
                        cylinder->set_allometry_improvement(AllometryImproved::ALLOM);
                    }

                }
            }

            if(cylinder->get_radius()<_min_rad)
            {
                cylinder->set_radius(_min_rad);
            }
        }
    }
}

float ImproveByAllometry::get_y_from_x(float x)
{
    float y = _a * (pow (x,_b));
    return y;
}

float ImproveByAllometry::get_x_from_y(float y)
{
    float log_x = (log(y/_a))/(_b);

    float x = exp(log_x);
    return x;
}


ImproveByAllometry::ImproveByAllometry(QSharedPointer<Tree> tree, MethodCoefficients cf,float a, float b, float fac, bool inverse, bool is_cropped)
{
    _tree = tree;
    _cf = cf;
    _a = a;
    _b = b;
    _fac = fac;
    _inverse = inverse;
    _min_rad = cf.min_rad;
    if(is_cropped)
    {
        improve_cropped();
    }
    else {
        improve();
    }
}
