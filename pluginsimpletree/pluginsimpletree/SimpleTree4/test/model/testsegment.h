#ifndef TESTSEGMENT_H
#define TESTSEGMENT_H

#include "src/model/segment.h"
#include "src/model/tree.h"
#include "src/model/tree.h"
#include "src/model/build_tree/buildtree.h"
#include "src/model/build_tree/improvebymedian.h"
#include <QtGlobal>


class TestSegment
{
    const static float _MAX_DISTANCE;
public:
    TestSegment();

    void
    test_volume();

    void
    test_add_cylinder();

    void
    test__child_connection();

    void
    test_remove();

    void
    test_leave_root();

    void
    test_median_smooth();

    void
    test_mean_smooth();

    void
    test_merge();
};

#endif // TESTSEGMENT_H
