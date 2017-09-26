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

#ifndef DIJKSTRA_COEFFICIENTS_H
#define DIJKSTRA_COEFFICIENTS_H

struct DijkstraCoefficients{

    const int TREE_CLUSTER_MIN_SIZE = 100;

    const float OCTREE_CELL_SIZE = 0.025f;

    float search_range = 0.05f;

    float max_distance = 0.05f;

    float down_scale_size = 0.01f;

    float cluster_for_bin = 0.03f;

    float bin_width = 0.06f;

    float percentage_for_large_cluster = 0.01;

    float punishment_additative = 0.02;

    float punishment_multiplicative = 5;

    float punishment_power = 1;

};




#endif // DIJKSTRA_COEFFICIENTS_H
