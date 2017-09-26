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

#include "convertcttost.h"

PointCloudS::Ptr ConvertCTtoST::get_cloud() const
{
    return _cloud;
}

ConvertCTtoST::ConvertCTtoST(CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, int knn, bool compute_curvature, bool remove , bool use_knn):
    _itemCpy_cloud_in(itemCpy_cloud_in), _knn(knn), _compute_curvature(compute_curvature), _remove(remove), _use_knn(use_knn)
{

}
PointCloudS::Ptr ConvertCTtoST::make_two_dimenstional()
{
    PointCloudS::Ptr result (new PointCloudS);
    size_t size = _cloud->points.size();
    result->points.resize(size);
    if(size > 0) {
        for(size_t i = 0; i < size; i++)
        {
            PointS p = _cloud->points.at(i);
            p.z = 0;

            result->points[i] = p;
        }
    }
    return result;
}

PointCloudS::Ptr ConvertCTtoST::reject_points()
{
    PointCloudS::Ptr cloud_out(new PointCloudS);
    size_t size = _cloud->points.size();
    for(size_t i = 0 ; i < size; i++)
    {
        PointS p = _cloud->points.at(i);
        if(p.eigen1<0.15)
        {
            cloud_out->points.push_back(p);
        }
    }
    return cloud_out;
}


void ConvertCTtoST::convert()
{
    if (_itemCpy_cloud_in != NULL)
    {
        const CT_AbstractPointCloudIndex* index =_itemCpy_cloud_in->getPointCloudIndex();
        size_t size = index->size();
        _cloud.reset(new PointCloudS);
        _cloud->width = (int) size;
        _cloud->height = 1;
        if(size > 0) {
            _cloud->points.resize(size);
            size_t i = 0;
            CT_PointIterator it (index);
            while(it.hasNext())
            {
                const CT_Point &internalPoint = it.next().currentPoint();
                PointS p(internalPoint(0),internalPoint(1),internalPoint(2));
                _cloud->points[i] = p;
                ++i;
            }
        }
        if(_compute_curvature)
        {
            EnrichCloud enrich(_cloud, _knn, 0.03, _use_knn);
            if(_remove)
            {
                _cloud = reject_points();
            }
            StemPointDetection stempts (0,0.15,0.3,1,0,0.8,0.035,_cloud,1) ;
            stempts.compute();
        }
    }
}
