#ifndef REORDERTREE_H
#define REORDERTREE_H

#include <algorithm>

#include "src/model/tree.h"


class ReorderTree
{
private:
    QSharedPointer<Tree> _tree;

    static float
    get_maximum_volume_to_leave(QSharedPointer<Segment> seg);

    static bool
    compareSegments(QSharedPointer<Segment> seg1, QSharedPointer<Segment> seg2);

    void
    reorder();
public:
    ReorderTree(QSharedPointer<Tree> tree);
};

#endif // REORDERTREE_H
