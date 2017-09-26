#ifndef ST_STEPCOMPLETEMODELLING_H
#define ST_STEPCOMPLETEMODELLING_H

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


#include <stdlib.h>
#include <QtGlobal>
#include <QList>

// Inclusion of auto-indexation system
#include "ct_tools/model/ct_autorenamemodels.h"

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

class ST_StepCompleteModelling: public CT_AbstractStep
{
    Q_OBJECT

public:

    /*! \brief Step constructor
     *
     * Create a new instance of the step
     *
     * \param dataInit Step parameters object
     */
    ST_StepCompleteModelling(CT_StepInitializeData &dataInit);

    ~ST_StepCompleteModelling();

    /*! \brief Step description
     *
     * Return a description of the step function
     */
    QString getStepDescription() const;

    /*! \brief Step detailled description
     *
     * Return a detailled description of the step function
     */
    QString getStepDetailledDescription() const;

    /*! \brief Step URL
     *
     * Return a URL of a wiki for this step
     */
    QString getStepURL() const;

    /*! \brief Step copy
     *
     * Step copy, used when a step is added by step contextual menu
     */
    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &dataInit);

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
    void createPostConfigurationDialog();

    /*! \brief Output results specification
     *
     * Specification of output results models created by the step (OUT)
     */
    void createOutResultModelListProtected();

    /*! \brief Algorithm of the step
     *
     * Step computation, using input results, and creating output results
     */
    void compute();

    void
    add_cylinder_data(Tree tree, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp, QString string );


    CT_TTreeGroup *constructTopology(const CT_AbstractResult *res_r, QSharedPointer<Tree> tree, QString string);
    void constructTopologyRecursively(const CT_AbstractResult *res_r, CT_TNodeGroup *parent,
                                      QSharedPointer<Segment>  segment,
                                      QSharedPointer<Tree> tree, QString string);
    void setCylinders(const CT_AbstractResult *res_r, CT_TNodeGroup *root,
                      QSharedPointer<Segment> segment,
                      QSharedPointer<Tree> tree, QString string);



private:


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

    CT_AutoRenameModels _branchIDModelName2;
    CT_AutoRenameModels _branchOrderModelName2;
    CT_AutoRenameModels _segmentIDModelName2;
    CT_AutoRenameModels _parentSegmentIDModelName2;
    CT_AutoRenameModels _growthVolumeModelName2;
    CT_AutoRenameModels _tree_species2;
    CT_AutoRenameModels _tree_id2;
    CT_AutoRenameModels _detection_type2;
    CT_AutoRenameModels _improvement_type2;




    CT_AutoRenameModels     _topologyGroup_fit;
    CT_AutoRenameModels     _rootTreeGroup_fit;
    CT_AutoRenameModels     _stemGroup_fit;
    CT_AutoRenameModels     _stemCylinders_fit;
    CT_AutoRenameModels     _branchGroup_fit;
    CT_AutoRenameModels     _outCylinderGroupModelName_fit;

    CT_AutoRenameModels     _topologyGroup_allom;
    CT_AutoRenameModels     _rootTreeGroup_allom;
    CT_AutoRenameModels     _stemGroup_allom;
    CT_AutoRenameModels     _stemCylinders_allom;
    CT_AutoRenameModels     _branchGroup_allom;
    CT_AutoRenameModels     _outCylinderGroupModelName_allom;

    //    CT_AutoRenameModels _branchIDModelName3;
    //    CT_AutoRenameModels _branchOrderModelName3;
    //    CT_AutoRenameModels _segmentIDModelName3;
    //    CT_AutoRenameModels _parentSegmentIDModelName3;
    //    CT_AutoRenameModels _growthVolumeModelName3;
    //    CT_AutoRenameModels _tree_species3;
    //    CT_AutoRenameModels _tree_id3;
    //    CT_AutoRenameModels _detection_type3;
    //    CT_AutoRenameModels _improvement_type3;


    //    CT_AutoRenameModels _branchIDModelName4;
    //    CT_AutoRenameModels _branchOrderModelName4;
    //    CT_AutoRenameModels _segmentIDModelName4;
    //    CT_AutoRenameModels _parentSegmentIDModelName4;
    //    CT_AutoRenameModels _growthVolumeModelName4;
    //    CT_AutoRenameModels _tree_species4;
    //    CT_AutoRenameModels _tree_id4;
    //    CT_AutoRenameModels _detection_type4;
    //    CT_AutoRenameModels _improvement_type4;





    // Step parameters






    QList<CT_Scene*>* _sceneList;



    double    _sphereRadiusMultiplier;
    double    _epsilonClusterStem;
    double    _epsilonClusterBranch;
    double    _epsilonSphere;
    int    _minPtsRansacStem;
    int    _minPtsRansacBranch;
    int    _minPtsClusterStem;
    int    _minPtsClusterBranch;
    double    _minRadiusSphere;
    int _iterationNumber;
    double _zoffset;
    bool _stemComputation;

    double _a;
    double _b;
    double _fact;
    double _minRad;

    double _criterion;
    int _iterations;
    int _seeds;

    int _percent_stem;


    int _knn;

    QString
    _species;

    QString
    _id;



};

#endif // ST_STEPCOMPLETEMODELLING_H

