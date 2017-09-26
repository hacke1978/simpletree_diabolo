#ifndef ST_STEPHELPER
#define ST_STEPHELPER

#include "ct_result/ct_resultgroup.h"
#include "ct_itemdrawable/ct_fileheader.h"
#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_result/model/outModel/ct_outresultmodelgroupcopy.h"
#include "ct_itemdrawable/ct_cylinder.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h"
#include "ct_itemdrawable/ct_pointsattributesscalartemplated.h"
#include "ct_itemdrawable/ct_pointsattributesnormal.h"
#include "ct_itemdrawable/ct_ttreegroup.h"
#include "ct_itemdrawable/ct_cylinder.h"
#include <pcl/console/print.h>
#include "item/st_coefficients.h"
#include "item/st_tree.h"
#include "SimpleTree4/method/method_coefficients.h"
#include "SimpleTree4/method/spherefollowing2.h"
#include "SimpleTree4/model/tree.h"
#include "SimpleTree4/method/point_cloud_operations/convertcttost.h"
#include "SimpleTree4/model/build_tree/improvebyattractor.h"
#include <SimpleTree4/model/build_tree/buildtree.h>
#include <SimpleTree4/model/build_tree/improvebymedian.h>
#include <SimpleTree4/model/build_tree/removefalsecylinders.h>
#include <SimpleTree4/model/build_tree/improvebymerge.h>
#include <SimpleTree4/model/build_tree/reordertree.h>
#include <SimpleTree4/model/build_tree/improvedbyadvancedmedian.h>
#include <SimpleTree4/model/build_tree/improvebypipemodel.h>
#include <SimpleTree4/model/build_tree/improvefit.h>
#include <SimpleTree4/model/build_tree/improvebranchjunctions.h>
#include <SimpleTree4/model/build_tree/improvebyallometry.h>
#include <SimpleTree4/method/computeallometry.h>
#include <SimpleTree4/export/exporttree.h>

#include <QDir>

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"
#define DEFin_coeff_in "coeff_in"
#define DEFin_header "header"
#define DEFin_tree_in "tree_in"



class ST_StepHelper
{
    Q_OBJECT

public:

    /*! \brief Step constructor
     *
     * Create a new instance of the step
     *
     * \param dataInit Step parameters object
     */
    ST_StepHelper();

    ~ST_StepHelper();












private:

    void add_cylinder_data(QSharedPointer<Tree> tree, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp, QString cylinder_name, QString cylinder_grp_name);


public:

    CT_AutoRenameModels    _tree_out;
    CT_AutoRenameModels    _coeff_out;
    CT_AutoRenameModels    _tree_out_grp;
    CT_AutoRenameModels    _coeff_out_grp;
    CT_AutoRenameModels    _cloud_out_normals;
    CT_AutoRenameModels    _cloud_out_stem;

    CT_AutoRenameModels     _cylinder_grp;
    CT_AutoRenameModels     _cylinders
    ;

    CT_AutoRenameModels _branchIDModelName;
    CT_AutoRenameModels _branchOrderModelName;
    CT_AutoRenameModels _segmentIDModelName;
    CT_AutoRenameModels _parentSegmentIDModelName;
    CT_AutoRenameModels _growthVolumeModelName;
    CT_AutoRenameModels _tree_species;
    CT_AutoRenameModels _tree_id;
    CT_AutoRenameModels _detection_type;
    CT_AutoRenameModels _improvement_type;





};

#endif // ST_STEPHELPER


