#ifndef ST_STEPSIMPLETREE_H
#define ST_STEPSIMPLETREE_H

#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_tools/model/ct_autorenamemodels.h"
#include "ct_itemdrawable/ct_pointsattributesnormal.h"
#include "ct_itemdrawable/ct_pointsattributesscalartemplated.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_itemdrawable/tools/iterator/ct_groupiterator.h"
#include "ct_itemdrawable/abstract/ct_abstractstandarditemgroup.h"

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

//#include "ct_step/abstract/ct_abstractstep.h"
//#include "ct_itemdrawable/ct_pointsattributesnormal.h"
//#include "ct_itemdrawable/ct_pointsattributesscalartemplated.h"
//#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
//#include "ct_itemdrawable/tools/iterator/ct_groupiterator.h"
//#include "ct_result/ct_resultgroup.h"
//#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
//#include "ct_normalcloud/ct_normalcloudstdvector.h"
//#include "ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h"
//#include "ct_view/ct_stepconfigurabledialog.h"
//#include "ct_iterator/ct_pointiterator.h"


#include "SimpleTree4/model/build_tree/buildtree.h"
#include "SimpleTree4/model/build_tree/improvebranchjunctions.h"
#include "SimpleTree4/model/build_tree/improvebymedian.h"
#include "SimpleTree4/model/build_tree/improvefit.h"
#include "SimpleTree4/model/build_tree/removefalsecylinders.h"
#include "SimpleTree4/model/build_tree/reordertree.h"
#include "SimpleTree4/method/spherefollowing.h"


class CT_TTreeGroup;
class CT_TNodeGroup;

/*!
 * \class ST_StepSimpleTree
 * \ingroup Steps_ST
 * \brief <b>Utilise la détection du logiciel SimpleTree pour détecter la structure de l'arbre.</b>
 *
 * No detailled description for this step
 *
 * \param _sphereRadiusMultiplier 
 * \param _epsilonClusterStem 
 * \param _epsilonClusterBranch 
 * \param _epsilonSphere 
 * \param _minPtsRansacStem 
 * \param _minPtsRansacBranch 
 * \param _minPtsClusterStem 
 * \param _minPtsClusterBranch 
 * \param _minRadiusSphereStem 
 * \param _minRadiusSphereBranch 
 *
 */

class ST_StepDetectTree: public CT_AbstractStep
{
    Q_OBJECT

public:

    /*! \brief Step constructor
     * 
     * Create a new instance of the step
     * 
     * \param dataInit Step parameters object
     */
    ST_StepDetectTree(CT_StepInitializeData &dataInit);

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


private slots:

    void setProgressWrapper(int ix);





private:

    PointCloudS::Ptr cloud;

    // Step parameters
    double    _sphereRadiusMultiplier;
    double    _epsilonClusterStem;
    double    _epsilonClusterBranch;
    double    _epsilonSphere;
    int    _minPtsRansacStem;
    int    _minPtsRansacBranch;
    int    _minPtsClusterStem;
    int    _minPtsClusterBranch;
    double    _minRadiusSphereStem;
    double    _minRadiusSphereBranch;
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

    CT_AutoRenameModels     _topologyGroup;
    CT_AutoRenameModels     _rootTreeGroup;
    CT_AutoRenameModels     _stemGroup;
    CT_AutoRenameModels     _stemCylinders;
    CT_AutoRenameModels     _branchGroup;

    CT_AutoRenameModels     _outCylinderGroupModelName;
    CT_AutoRenameModels     _outCylinderModelName;
    CT_AutoRenameModels     _outSphereGroupModelName;
    CT_AutoRenameModels     _outSphereModelName;
    CT_AutoRenameModels     _outTreeAttributesModelName;
    CT_AutoRenameModels     _outPointCloudStem;
    CT_AutoRenameModels     _outPointCloudE1;
    CT_AutoRenameModels     _outPointCloudE2;
    CT_AutoRenameModels     _outPointCloudE3;


    CT_AutoRenameModels _branchIDModelName;
    CT_AutoRenameModels _branchOrderModelName;
    CT_AutoRenameModels _segmentIDModelName;
    CT_AutoRenameModels _parentSegmentIDModelName;
    CT_AutoRenameModels _growthVolumeModelName;

    CT_AutoRenameModels _totalVolumeModelName;
    CT_AutoRenameModels _solidVolumeModelName;
    CT_AutoRenameModels _totalStemVolumeModelName;
    CT_AutoRenameModels _heightModelName;
    CT_AutoRenameModels _lengthModelName;
    CT_AutoRenameModels _DBHModelName;
    CT_AutoRenameModels _baseDiameterModelName;
    CT_AutoRenameModels _heightAboveGroundModelName;
    CT_AutoRenameModels _volumeUntilFirstBranchModelName;
    CT_AutoRenameModels _volumeUntilCrownModelName;
    CT_AutoRenameModels _crownVolumeModelName;
    CT_AutoRenameModels _crownProjectionAreaModelName;
    CT_AutoRenameModels _crownSurfaceAreaModelName;

    CT_AutoRenameModels    _cloud_out_normals;
    CT_AutoRenameModels    _cloud_out_curvature;
        CT_AutoRenameModels    _cloud_out_eigen1;
        CT_AutoRenameModels    _cloud_out_eigen2;
        CT_AutoRenameModels    _cloud_out_eigen3;
        CT_AutoRenameModels    _cloud_out_stem;
        CT_AutoRenameModels   _cloud_out_ID;


        void
        enrich_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp);

            void create_simple_tree_cloud(const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in);


//    CT_TTreeGroup *constructTopology(const Eigen::Vector3d &min, const CT_AbstractResult *res_r, boost::shared_ptr<simpleTree::Tree> tree_ptr);

//    void constructTopologyRecursively(const Eigen::Vector3d &min, const CT_AbstractResult *res_r, CT_TNodeGroup *parent, boost::shared_ptr<simpleTree::Segment>  segment, boost::shared_ptr<simpleTree::Tree> tree_ptr);
//    void setCylinders(const Eigen::Vector3d &min, const CT_AbstractResult *res_r, CT_TNodeGroup *root, boost::shared_ptr<simpleTree::Segment> segment, boost::shared_ptr<simpleTree::Tree> tree_ptr);
};

#endif // ST_STEPSIMPLETREE_H
