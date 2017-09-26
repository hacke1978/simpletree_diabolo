#ifndef TESTTREE_H
#define TESTTREE_H

#include "src/model/tree.h"
#include "src/model/build_tree/buildtree.h"
#include "src/model/build_tree/removefalsecylinders.h"
#include "src/model/build_tree/reordertree.h"

#include <QtGlobal>


class TestTree
{
public:
    TestTree();

    void
    test_volume();

    void
    test_build_tree();

    void
    test_remove_false_cylinders();

    void
    test_reorder_tree();

    void
    plot(QSharedPointer<Segment> segment, QString begin  = "");
};

#endif // TESTTREE_H
