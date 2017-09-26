#ifndef ST_STEPABSTRACTMODELLING_H
#define ST_STEPABSTRACTMODELLING_H

#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_itemdrawable/ct_pointsattributesnormal.h"
#include "ct_itemdrawable/ct_pointsattributesscalartemplated.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_itemdrawable/tools/iterator/ct_groupiterator.h"
#include "ct_result/ct_resultgroup.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_normalcloud/ct_normalcloudstdvector.h"
#include "ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h"
#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_iterator/ct_pointiterator.h"


#include "ct_itemdrawable/ct_ttreegroup.h"

#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_itemdrawable/ct_cylinder.h"
#include "ct_itemdrawable/ct_sphere.h"
#include "ct_itemdrawable/ct_attributeslist.h"
#include "ct_itemdrawable/ct_pointsattributesscalartemplated.h"
#include "ct_itemdrawable/tools/iterator/ct_groupiterator.h"
#include "ct_result/ct_resultgroup.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_result/model/outModel/ct_outresultmodelgroupcopy.h"
#include "ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h"
#include "ct_view/ct_stepconfigurabledialog.h"
#include "ctlibpcl/tools/ct_pcltools.h"
#include "ct_itemdrawable/ct_scene.h"
#include "ct_itemdrawable/ct_fileheader.h"

#include "SimpleTree4/model/pointsimpletree.h"
#include "SimpleTree4/method/point_cloud_operations/enrichcloud.h"
#include "SimpleTree4/method/point_cloud_operations/stempointdetection.h"
#include "SimpleTree4/model/build_tree/buildtree.h"
#include "SimpleTree4/model/build_tree/improvebranchjunctions.h"
#include "SimpleTree4/model/build_tree/improvebymedian.h"
#include "SimpleTree4/model/build_tree/improvefit.h"
#include "SimpleTree4/model/build_tree/removefalsecylinders.h"
#include "SimpleTree4/model/build_tree/reordertree.h"
#include "SimpleTree4/model/build_tree/improvebymerge.h"
#include "SimpleTree4/method/optimizationspherefollowing.h"
#include "SimpleTree4/method/spherefollowingrecursive.h"
#include "SimpleTree4/model/build_tree/improvebyallometry.h"
#include "SimpleTree4/model/build_tree/improvebypipemodel.h"
#include "SimpleTree4/method/point_cloud_operations/computemeanandstandarddeviation.h"
#include "SimpleTree4/model/build_tree/improvedbyadvancedmedian.h"
#include "SimpleTree4/method/computeallometry.h"
#include "SimpleTree4/import/readcsv.h"
#include "SimpleTree4/export/exporttree.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>


#include <stdlib.h>
#include <QtGlobal>
#include <QList>
#include <QFileInfo>
#include <QString>

// Inclusion of auto-indexation system
#include "ct_tools/model/ct_autorenamemodels.h"


// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"
#define DEFin_header "header"

/*!
 * \class ST_StepStemPointDetection
 * \ingroup Steps_ST
 * \brief <b>detects stem points.</b>
 *
 * No detailled description for this step
 *
 * \param _param1
 *
 */

class ST_StepAbstractModelling: public CT_AbstractStep
{
    Q_OBJECT

public:

    /*! \brief Step constructor
     *
     * Create a new instance of the step
     *
     * \param dataInit Step parameters object
     */
    ST_StepAbstractModelling(CT_StepInitializeData &dataInit);

    ~ST_StepAbstractModelling();

    /*! \brief Step description
     *
     * Return a description of the step function
     */
    virtual QString getStepDescription() const = 0;

    /*! \brief Step detailled description
     *
     * Return a detailled description of the step function
     */
    virtual QString getStepDetailledDescription() const = 0;

    /*! \brief Step URL
     *
     * Return a URL of a wiki for this step
     */
    virtual QString getStepURL() const = 0;

    /*! \brief Step copy
     *
     * Step copy, used when a step is added by step contextual menu
     */
    virtual CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &dataInit) = 0;

public slots:

    void receive_qstring_abstract_modelling(QString qstring);

    void receive_counter_abstract_modelling(int counter);

protected:

    /*! \brief Input results specification
     *
     * Specification of input results models needed by the step (IN)
     */
    void createInResultModelListProtected();

    /*! \brief Parameters DialogBox
     *
     * DialogBox asking for step parameters
     */
    virtual void createPostConfigurationDialog() = 0;

    /*! \brief Output results specification
     *
     * Specification of output results models created by the step (OUT)
     */
    void createOutResultModelListProtected();

    /*! \brief Algorithm of the step
     *
     * Step computation, using input results, and creating output results
     */
    virtual void compute() = 0;

    void
    add_cylinder_data(Tree tree, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp, QString string );


    CT_TTreeGroup *constructTopology(const CT_AbstractResult *res_r, QSharedPointer<Tree> tree, QString string);
    void constructTopologyRecursively(const CT_AbstractResult *res_r, CT_TNodeGroup *parent,
                                      QSharedPointer<Segment>  segment,
                                      QSharedPointer<Tree> tree, QString string);
    void setCylinders(const CT_AbstractResult *res_r, CT_TNodeGroup *root,
                      QSharedPointer<Segment> segment,
                      QSharedPointer<Tree> tree, QString string);






    QMap<QString, FileCoefficients> get_map();


    void build_tree_model(CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp);

    void
    enrich_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp);

    void create_simple_tree_cloud(const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in);

    PointCloudS::Ptr _cloud;

    // Declaration of autoRenames Variables (groups or items added to In models copies)
    CT_AutoRenameModels    _cloud_out_normals;
    CT_AutoRenameModels    _cloud_out_curvature;
    CT_AutoRenameModels    _cloud_out_eigen1;
    CT_AutoRenameModels    _cloud_out_eigen2;
    CT_AutoRenameModels    _cloud_out_eigen3;
    CT_AutoRenameModels    _cloud_out_stem;


    CT_AutoRenameModels     _outCylinderGroupModelName;
    CT_AutoRenameModels     _outCylinderModelName_unimproved;
    CT_AutoRenameModels     _outCylinderModelName_improved_branch_junctions;
    CT_AutoRenameModels     _outCylinderModelName_removed_false_cylinders;
    CT_AutoRenameModels     _outCylinderModelName_removed_improved_by_median;
    CT_AutoRenameModels     _outCylinderModelName_improved_by_fit;
    CT_AutoRenameModels     _outCylinderModelName_improved_by_allometry;
    CT_AutoRenameModels     _outCylinderModelName_improved_by_merge;

    CT_AutoRenameModels _branchIDModelName;
    CT_AutoRenameModels _branchOrderModelName;
    CT_AutoRenameModels _segmentIDModelName;
    CT_AutoRenameModels _parentSegmentIDModelName;
    CT_AutoRenameModels _growthVolumeModelName;
    CT_AutoRenameModels _tree_species;
    CT_AutoRenameModels _tree_id;
    CT_AutoRenameModels _detection_type;
    CT_AutoRenameModels _improvement_type;



    CT_AutoRenameModels     _topologyGroup;
//    CT_AutoRenameModels     _rootTreeGroup;
    CT_AutoRenameModels     _stemGroup;
    CT_AutoRenameModels     _stemCylinders;
    CT_AutoRenameModels     _branchGroup;







    QList<CT_Scene*>* _sceneList;

    int _percent_stem;


    int _knn;

    QStringList _file_name_list;

    QString
    _species;

    QString
    _id;

    QString
    _use_allom;

    QString
    _out_put_path;



};

#endif // ST_STEPABSTRACTMODELLING_H

