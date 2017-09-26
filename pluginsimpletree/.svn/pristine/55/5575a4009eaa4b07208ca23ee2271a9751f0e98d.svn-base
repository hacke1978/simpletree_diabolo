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

#include "predictstartparameters_eric.h"

MethodCoefficients PredictStartParameters_Eric::get_coeff() const
{
    return _coeff;
}

int PredictStartParameters_Eric::get_percentage()
{
    int total =(int) _cloud->points.size();
    int stm_pts = 0;
    for(size_t i = 0; i < total; i++)
    {
        if(_cloud->points.at(i).is_stem)
        {
            stm_pts ++;
        }
    }
    if(total==0)
    {
        return 0;
    }
    int percentage = (((float)stm_pts)/((float)total))*100.0f;
    return percentage;

}

float PredictStartParameters_Eric::get_height()
{
    float z_min = std::numeric_limits<float>::max();
    float z_max = std::numeric_limits<float>::lowest();
    for(size_t j = 0; j < _cloud->points.size(); j++)
    {
        PointS p = _cloud->points.at(j);
        if(p.z < z_min)
        {
            z_min = p.z;
        }

        if(p.z > z_max)
        {
            z_max = p.z;
        }
    }
    float height = z_max - z_min + _cut_height;
    return height;
}

PredictStartParameters_Eric::PredictStartParameters_Eric(PointCloudS::Ptr cloud, float cut_height): _cloud(cloud), _cut_height(cut_height)
{
    ComputeMeanAndStandardDeviation m_sd(_cloud);
    _coeff.species = "unknown";
    _coeff.cut_height = _cut_height;
    _coeff.sd = m_sd._sd;
    _coeff.mean = m_sd._mean;
//    _coeff.epsilon_cluster_stem = 2*m_sd._mean;
    _coeff.epsilon_cluster_branch = 3*(m_sd._mean + 2*m_sd._sd);
   _coeff.epsilon_sphere = 0.03;
   _coeff.optimze_stem = true;
     PredictStableVolume pv (_cloud, _coeff);
    _coeff = pv.get_coeff();
}
