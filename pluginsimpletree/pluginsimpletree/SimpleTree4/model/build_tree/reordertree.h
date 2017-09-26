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

#ifndef REORDERTREE_H
#define REORDERTREE_H

#include <algorithm>

#include "../tree.h"
#include "removesidewardconnection.h"


class ReorderTree
{
private:
    QSharedPointer<Tree> _tree;

    static float
    get_maximum_volume_to_leave(QSharedPointer<Segment> seg);

    static bool
    compareSegments(QSharedPointer<Segment> seg1, QSharedPointer<Segment> seg2);

    static bool
    compareSegments0(QSharedPointer<Segment> seg1, QSharedPointer<Segment> seg2);

    static bool
    compareSegments1(QSharedPointer<Segment> seg1, QSharedPointer<Segment> seg2);

    static bool
    compareSegments2(QSharedPointer<Segment> seg1, QSharedPointer<Segment> seg2);

    static bool
    compareSegments3(QSharedPointer<Segment> seg1, QSharedPointer<Segment> seg2);

    static bool
    compareStemSegments(QSharedPointer<Segment> seg1, QSharedPointer<Segment> seg2);

    void
    reorder();

    void
    fill_IDs();

    void set_cylinder_ID();

    void recursive_stem_tagging(QSharedPointer<Segment> seg);

    void generate_reverse_pipe_bo(QSharedPointer<Segment> seg);


    QSharedPointer<Segment> detect_top_segment();

    void
    generate_branch_order(QSharedPointer<Segment> segment, int branch_order);

    QVector<QSharedPointer<Segment> >
    get_stem();

    QVector<QSharedPointer<Segment> >
    get_branch_roots();

    void
    generate_branch_ID();

    int _compare_type = 2;


public:
    ReorderTree(QSharedPointer<Tree> tree, int compare_type = 2);
};

#endif // REORDERTREE_H
