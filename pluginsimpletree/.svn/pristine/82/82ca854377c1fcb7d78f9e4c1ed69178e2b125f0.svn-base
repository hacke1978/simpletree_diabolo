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

#include "improvebyallometry_growth_length.h"

void ImproveByAllometryLength::improve()
{
    //MethodCoefficients coeff;
    //    QVector<float> growth_volumina;
    QVector<float> growth_lengths;
    QVector<QSharedPointer<Cylinder> > cylinders = _tree->get_all_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        //        float y = _tree->get_growth_volume(cylinder);
        //        growth_volumina.push_back(y);
        float y2 = _tree->get_growth_length(cylinder);
        growth_lengths.push_back(y2);
    }
    for(size_t i = 0; i < cylinders.size(); i++)
    {
        QSharedPointer<Cylinder> cylinder = cylinders.at(i);
        float y2 = growth_lengths.at(i);

        if(cylinder->get_allometry_improvement()==AllometryImproved::ALLOM )
        {
            cylinder->set_radius(get_x_from_y(y2));
            cylinder->set_allometry_improvement(AllometryImproved::ALLOM);

            if(cylinder->get_radius()<_min_rad)
            {
                cylinder->set_radius(_min_rad);
            }
        }/*
        else
        {
            qDebug () <<"improvebyallometry_growth_length non improve";
        }*/
    }
}

float ImproveByAllometryLength::get_y_from_x(float x)
{
    float y = _a_length * (pow (x,_b_length));
    return y;
}

float ImproveByAllometryLength::get_x_from_y(float y)
{
    float log_x = (log(y/_a_length))/(_b_length);
    float x = exp(log_x);
    return x;
}


ImproveByAllometryLength::ImproveByAllometryLength(QSharedPointer<Tree> tree, MethodCoefficients cf, float a, float b, float a_length, float b_length, float fac , float min_rad)
{
    _tree = tree;
    _cf = cf;
    _a = a;
    _b = b;
    _fac = fac;
    _a_length = a_length;
    _b_length = b_length;
    _min_rad = cf.min_rad;
    if(_b_length > 1.2f && _b_length < 2.7f)
        improve();
    else {
        qDebug() << "ImproveByAllometryL : fail allom with cloud " << _cf.id;
    }
}
