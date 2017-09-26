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

#include "subdividestemandbranchpoints.h"

SubdivideStemAndBranchPoints::SubdivideStemAndBranchPoints()
{

}

void SubdivideStemAndBranchPoints::subdivide_points(PointCloudS::Ptr cloud_in, PointCloudS::Ptr cloud_stem, PointCloudS::Ptr cloud_branch)
{
    for(int i = 0; i < cloud_in->points.size(); i++)
    {
        if(cloud_in->points.at(i).is_stem>0.5)
        {
            cloud_stem->push_back(cloud_in->points.at(i));
        } else
        {
            cloud_branch->push_back(cloud_in->points.at(i));
        }
    }
    cloud_stem->width = cloud_stem->points.size();
    cloud_stem->height = 1;


    cloud_branch->width = cloud_branch->points.size();
    cloud_branch->height = 1;
}
