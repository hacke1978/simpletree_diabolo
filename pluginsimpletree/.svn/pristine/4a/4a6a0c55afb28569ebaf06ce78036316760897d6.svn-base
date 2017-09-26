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

#ifndef IMPROVEBYMEDIAN_H
#define IMPROVEBYMEDIAN_H

#include "../tree.h"

class ImproveByMedian
{
private:
    const static float _MAX_PERCENTAGE;

    const static float _MIN_PERCENTAGE;

    QSharedPointer<Tree> _tree;

    /**
     * @brief median_smooth Within the segments outlier radii are detected via median check;
     */
    void median_smooth(QSharedPointer<Segment> segment);

    /**
     * @brief median_smooth Within the segments outlier radii are detected via mean check;
     */
    void mean_smooth(QSharedPointer<Segment> segment);

public:
    ImproveByMedian(QSharedPointer<Tree> tree);
};

#endif // IMPROVEBYMEDIAN_H
