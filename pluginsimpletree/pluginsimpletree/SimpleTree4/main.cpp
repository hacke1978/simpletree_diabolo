#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <iostream>



#include "src/model/cylinder.h"
#include <pcl/ModelCoefficients.h>
#include <QObject>
#include <QSharedPointer>
#include <QWeakPointer>
#include <QVector>

#include "src/test/model/testcylinder.h"
#include "src/test/testsimplemath.h"
#include "src/test/model/testsegment.h"
#include "src/test/model/testtree.h"

struct MyPointType
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float test;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, test, test)
)


int
main (int argc, char** argv)
{
    TestCylinder test;
    TestSimpleMath test2;
    TestSegment test3;
    TestTree test4;
    return 0;
}



//#include <iostream>

//using namespace std;

//int main(int argc, char *argv[])
//{
//    cout << "Hello World!" << endl;
//    return 0;
//}
