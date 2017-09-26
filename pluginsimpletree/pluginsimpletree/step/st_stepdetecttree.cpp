#include "st_stepdetecttree.h"

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



// Alias for indexing models
#define DEFin_r "result"
#define DEFin_treeGroup "treeGroup"
#define DEFin_scene "scene"

#define DEFin_res "result"
#define DEFin_grp "treeGroup"
#define DEFin_cloud_in "scene"

// Constructor : initialization of parameters
ST_StepDetectTree::ST_StepDetectTree(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    setDebuggable(true);

    _sphereRadiusMultiplier = 2;
    _epsilonClusterStem = 0.1;
    _epsilonClusterBranch = 0.1;
    _epsilonSphere = 0.035;
    _minPtsRansacStem = 50;
    _minPtsRansacBranch = 50;
    _minPtsClusterStem = 3;
    _minPtsClusterBranch = 3;
    _minRadiusSphereStem = 0.1;
    _minRadiusSphereBranch = 0.1;
    _iterationNumber = 1;
    _zoffset = .2; //.32
    _stemComputation = false;

    _a = 35.671;
    _b = 2.204;
    _fact = 2;
    _minRad = 0.0025;
    _criterion = 0.0001;
    _iterations = 0;
    _seeds = 81;

}

// Step description (tooltip of contextual menu)
QString ST_StepDetectTree::getStepDescription() const
{
    return tr("Tree detection");
}

// Step detailled description
QString ST_StepDetectTree::getStepDetailledDescription() const
{
    return tr("No detailled description for this step");
}

// Step URL
QString ST_StepDetectTree::getStepURL() const
{
    return "http://www.simpletree.uni-freiburg.de";
}

// Step copy method
CT_VirtualAbstractStep* ST_StepDetectTree::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepDetectTree(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepDetectTree::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_r = createNewInResultModelForCopy(DEFin_r);
    resIn_r->setZeroOrMoreRootGroup();
    resIn_r->addGroupModel("", DEFin_treeGroup);
    resIn_r->addItemModel(DEFin_treeGroup, DEFin_scene, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Isolated tree"));

}

// Creation and affiliation of OUT models
void ST_StepDetectTree::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *res_r = createNewOutResultModelToCopy(DEFin_r);

    if(res_r != NULL) {
        res_r->addGroupModel(DEFin_treeGroup, _topologyGroup, new CT_TTreeGroup(), tr("Topology"));
        res_r->addGroupModel(_topologyGroup, _rootTreeGroup, new CT_TNodeGroup(), tr("Root"));
        res_r->addGroupModel(_topologyGroup, _branchGroup, new CT_TNodeGroup(), tr("Branch"));
        res_r->addGroupModel(_topologyGroup, _stemGroup, new CT_TNodeGroup(), tr("Stem"));
        res_r->addItemModel(_stemGroup, _stemCylinders, new CT_Cylinder(), tr("Stem cylinders"));


        res_r->addGroupModel(DEFin_treeGroup, _outCylinderGroupModelName, new CT_StandardItemGroup(), tr("Cylinder group"));
        res_r->addItemModel(_outCylinderGroupModelName, _outCylinderModelName, new CT_Cylinder(), tr("Cylinder"));

        res_r->addItemAttributeModel(_outCylinderModelName, _branchIDModelName,
                                     new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                     tr("branchID"));

        res_r->addItemAttributeModel(_outCylinderModelName, _branchOrderModelName,
                                     new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                     tr("branchOrder"));

        res_r->addItemAttributeModel(_outCylinderModelName, _segmentIDModelName,
                                     new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                     tr("segmentID"));

        res_r->addItemAttributeModel(_outCylinderModelName, _parentSegmentIDModelName,
                                     new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                     tr("parentSegmentID"));

        res_r->addItemAttributeModel(_outCylinderModelName, _growthVolumeModelName,
                                     new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                     tr("growthVolume"));

        res_r->addGroupModel(DEFin_treeGroup, _outSphereGroupModelName, new CT_StandardItemGroup(), tr("Sphere group"));
        res_r->addItemModel(_outSphereGroupModelName, _outSphereModelName, new CT_Sphere(), tr("Sphere"));

        res_r->addItemModel(DEFin_treeGroup, _outPointCloudStem, new CT_PointsAttributesScalarTemplated<quint16>(), tr("Stem Flag"));
        res_r->addItemModel(DEFin_treeGroup, _outPointCloudE1, new CT_PointsAttributesScalarTemplated<float>(), tr("Lambda 1"));
        res_r->addItemModel(DEFin_treeGroup, _outPointCloudE2, new CT_PointsAttributesScalarTemplated<float>(), tr("Lambda 2"));
        res_r->addItemModel(DEFin_treeGroup, _outPointCloudE3, new CT_PointsAttributesScalarTemplated<float>(), tr("Lambda 3"));

        res_r->addItemModel(DEFin_treeGroup, _outTreeAttributesModelName, new CT_AttributesList(), tr("Trees attributes"));

        res_r->addItemAttributeModel(_outTreeAttributesModelName, _totalVolumeModelName,
                                     new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                     tr("totalVolume"));
        res_r->addItemAttributeModel(_outTreeAttributesModelName, _solidVolumeModelName,
                                     new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                     tr("solidVolume"));
        res_r->addItemAttributeModel(_outTreeAttributesModelName, _totalStemVolumeModelName,
                                     new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                     tr("totalStemVolume"));
        res_r->addItemAttributeModel(_outTreeAttributesModelName, _heightModelName,
                                     new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                     tr("height"));
        res_r->addItemAttributeModel(_outTreeAttributesModelName, _lengthModelName,
                                     new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                     tr("length"));
        res_r->addItemAttributeModel(_outTreeAttributesModelName, _DBHModelName,
                                     new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                     tr("DBH"));
        res_r->addItemAttributeModel(_outTreeAttributesModelName, _baseDiameterModelName,
                                     new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                     tr("baseDiameter"));
        res_r->addItemAttributeModel(_outTreeAttributesModelName, _heightAboveGroundModelName,
                                     new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                     tr("heightAboveGround"));
        res_r->addItemAttributeModel(_outTreeAttributesModelName, _volumeUntilFirstBranchModelName,
                                     new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                     tr("volumeUntilFirstBranch"));
        res_r->addItemAttributeModel(_outTreeAttributesModelName, _volumeUntilCrownModelName,
                                     new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                     tr("volumeUntilCrown"));
        res_r->addItemAttributeModel(_outTreeAttributesModelName, _crownVolumeModelName,
                                     new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                     tr("crownVolume"));
        res_r->addItemAttributeModel(_outTreeAttributesModelName, _crownProjectionAreaModelName,
                                     new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                     tr("crownProjectionArea"));
        res_r->addItemAttributeModel(_outTreeAttributesModelName, _crownSurfaceAreaModelName,
                                     new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                     tr("crownSurfaceArea"));
    }

}

// Semi-automatic creation of step parameters DialogBox
void ST_StepDetectTree::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();

    configDialog->addBool(tr("Sudivide stem from branch points"), "", "", _stemComputation);
    configDialog->addTitle(tr("<b>Main parameters</b>"));
    configDialog->addDouble(tr("Epsilon cluster stem"), "m", 0, 100, 3, _epsilonClusterStem, 1);
    configDialog->addDouble(tr("Epsilon cluster branch"), "m", 0, 100, 3, _epsilonClusterBranch, 1);
    configDialog->addDouble(tr("Epsilon sphere"), "m", 0, 100, 3, _epsilonSphere, 1);
    configDialog->addInt(tr("Number of iterations"), "m", 0, 10, _iterationNumber);
    configDialog->addTitle(tr("<em>Increasing the number of iterations : detect more, but is slower</em>"));

    configDialog->addEmpty();
    configDialog->addTitle(tr("<b>Other parameters (don't change except if you have a good reason)</b>"));
    configDialog->addDouble(tr("Sphere radius multiplier"), "m", 0, 100, 1.5, _sphereRadiusMultiplier, 1);
    configDialog->addInt(tr("Min pts ransac stem"), "", 0, 1e+08, _minPtsRansacStem);
    configDialog->addInt(tr("Min pts ransac branch"), "", 0, 1e+08, _minPtsRansacBranch);
    configDialog->addInt(tr("Min pts cluster stem"), "", 0, 1e+08, _minPtsClusterStem);
    configDialog->addInt(tr("Min pts cluster branch"), "", 0, 1e+08, _minPtsClusterBranch);
    configDialog->addDouble(tr("Min radius sphere stem"), "m", 0, 100, 3, _minRadiusSphereStem, 1);
    configDialog->addDouble(tr("Min radius sphere branch"), "m", 0, 100, 3, _minRadiusSphereBranch, 1);


    configDialog->addEmpty();
    configDialog->addDouble(tr("Z offset"), "m", -1e+08, 1e+08, 3, _zoffset);
    configDialog->addTitle(tr("<em>Set here the height above ground of the lowest tree point.\nSet it equal to ExtractSoil step thickness parameter.</em>"));

    configDialog->addEmpty();
    configDialog->addTitle(tr("<b>Allometry parameters - set fact to 100 if no allometry desired</b>"));
    configDialog->addDouble(tr("a"), "", 0, 1000, 4, _a, 1);
    configDialog->addDouble(tr("b"), "", 0, 1000, 4, _b, 1);
    configDialog->addDouble(tr("fact"), "", 0, 100, 2, _fact, 1);
    configDialog->addDouble(tr("minRad"), "m", 0, 0.1 , 5, _minRad, 1);


    configDialog->addEmpty();
    configDialog->addTitle(tr("<b>Do not change parameters (not working) - Optimization parameters - set iterations to 0 if no optimization desired</b>"));
    configDialog->addDouble(tr("criterion"), "", 0, 1000, 4, _criterion, 1);
    configDialog->addInt(tr("iterations"), "", 0, 100, _iterations);
    configDialog->addInt(tr("seeds"), "", 0, 100,  _seeds);
}



void ST_StepDetectTree::compute()
{
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);

    // IN results browsing


    // COPIED results browsing
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    while (itCpy_grp.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();

        const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        create_simple_tree_cloud(itemCpy_cloud_in);
        enrich_cloud(itemCpy_cloud_in,resCpy_res,grpCpy_grp);
    }


            MethodCoefficients method_coefficients;
            method_coefficients.name = "";
            method_coefficients.sphere_radius_multiplier = _sphereRadiusMultiplier;
            method_coefficients.epsilon_cluster_stem = _epsilonClusterStem;
            method_coefficients.epsilon_cluster_branch = _epsilonClusterBranch;
            method_coefficients.epsilon_sphere = _epsilonSphere;
            method_coefficients.minPts_ransac_stem = _minPtsRansacStem;
            method_coefficients.minPts_ransac_branch = _minPtsRansacBranch;
            method_coefficients.minPts_cluster_stem = _minPtsClusterStem;
          //              method_coefficients.minPts_cluster_branch = _minPtsClusterBranch;
          //  method_coefficients.minPts_cluster_branch = _minPtsClusterBranch;
            method_coefficients.min_radius_sphere = _minRadiusSphereStem;
        //    method_coefficients.min_radius_sphere_branch = _minRadiusSphereBranch;

            method_coefficients.a = _a;
            method_coefficients.b = _b;
            method_coefficients.fact = _fact;
            method_coefficients.minRad = _minRad;

            method_coefficients.min_dist = _criterion;
            method_coefficients.max_iterations = _iterations;
            method_coefficients.seeds_per_voxel = _seeds;

//            pcl::console::TicToc tt;
//            tt.tic ();

//            if(method_coefficients.max_iterations>0)
//            {


//                boost::shared_ptr<Optimization> op (new Optimization(method_coefficients.max_iterations,method_coefficients.seeds_per_voxel,
//                                                                     method_coefficients.min_dist));
//                connect(&*op, SIGNAL(emit_progress(int)), this, SLOT(setProgressWrapper(int)));
//                PS_LOG->addInfoMessage(this, tr("Opimization of parameters : "));
//                PS_LOG->addInfoMessage(this, method_coefficients.toQString());

//                op->setCloudPtr(cloud_with_normals);
//                op->setIsStem(isStem);
//                op->setTreeID("this->getControl ()->getTreeID()");
//                op->setCoefficients(method_coefficients);
//                op->optimize();
//                method_coefficients = op->getCoefficients();
//                PS_LOG->addInfoMessage(this, tr("Opimization of parameters : \n ").append(QString::number((tt.toc()/100))).append(" seconds."));
//                PS_LOG->addInfoMessage(this, tr("to : \n "));
//                PS_LOG->addInfoMessage(this, method_coefficients.toQString());

//            }else{
//                PS_LOG->addInfoMessage(this, tr("No optimization desired"));
//            }


















            SphereFollowing sphereFollowing (method_coefficients, cloud);
            sphereFollowing.sphere_following();
    //        Tree tree

            setProgress(80);
            waitForAckIfInDebugMode();

            QVector<pcl::ModelCoefficients> cylinder_coeff = sphereFollowing.get_cylinders();
            BuildTree build(cylinder_coeff);
            Tree tree(build.getRoot_segment());
            QVector<QSharedPointer<Cylinder> > cylinders = tree.get_all_cylinders();

     //       const std::vector<pcl::ModelCoefficients > &spheres = sphereFollowing.getSpheres();
//            std::vector<pcl::ModelCoefficients > &cylindersFirst = sphereFollowing.getCylinders();

          //  PS_LOG->addInfoMessage(this, tr("Created %1 spheres").arg(spheres.size()));

            PS_LOG->addInfoMessage(this, tr("Tree structure"));

//            boost::shared_ptr<simpleTree::Tree> tree_ptr = boost::make_shared<simpleTree::Tree> ( cylindersFirst, cloud_with_normals,
//                                                                                                  "", true );

            setProgress(95);

//            if(_fact != 100)
//            {
//                simpleTree::Allometry allom;
//                allom.setTree(tree_ptr);
//                allom.setCoefficients(method_coefficients.a,method_coefficients.b);
//                allom.setFac(method_coefficients.fact);
//                allom.setMinRad(method_coefficients.minRad);
//                allom.improveTree();
//                setProgress(97);
//            }
        //    simpleTree::Tree tree = *tree_ptr;

            //CT_TTreeGroup *ctTree = constructTopology(min, res_r, tree_ptr);

//            if(ctTree != NULL)
//                treeGroup->addGroup(ctTree);

 //           std::vector<boost::shared_ptr<simpleTree::Cylinder> > cylinders = tree.getCylinders();

            //            for(int i = 0; i < cylinders.size(); i++)
            //            {
            //                boost::shared_ptr<simpleTree::Cylinder> cylinder = cylinders.at(i);

            //                cylinder->values.at(0) += cylinder->values.at(3)/2;
            //                cylinder->values.at(1) += cylinder->values.at(4)/2;
            //                cylinder->values.at(2) += cylinder->values.at(5)/2;

            //            }
//            std::vector<boost::shared_ptr<simpleTree::Cylinder> >::const_iterator it = cylinders.begin();
//            std::vector<boost::shared_ptr<simpleTree::Cylinder> >::const_iterator end = cylinders.end();

            QVectorIterator<QSharedPointer<Cylinder> >  it(cylinders);

       //     PS_LOG->addInfoMessage(this, tr("Computree data creation"));

            while(it.hasNext()) {

                QSharedPointer<Cylinder> cylinderPtr = it.next();

//                boost::shared_ptr<simpleTree::Cylinder> cylin = (*it);

                CT_StandardItemGroup* cylinderGroup = new CT_StandardItemGroup(_outCylinderGroupModelName.completeName(), res_r);
                treeGroup->addGroup(cylinderGroup);

//                int branchID = cylin->getSegment()->branchID;
//                int branchOrder = cylin->getSegment()->branchOrder;
//                int segmentID = cylin->getSegment()->segmentID;
//                int parentSegmentID = -1;
//                if (cylin->getSegment()->getParent()!=0)
//                {
//                    parentSegmentID = cylin->getSegment()->getParent()->segmentID;
//                }
//                float growthVolume = tree.getGrowthVolume(cylin);

                QSharedPointer<PointS> center = cylinderPtr->get_center();
                QSharedPointer<PointS> start = cylinderPtr->get_start();
                QSharedPointer<PointS> end   = cylinderPtr->get_end();


                CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x, center->y, center->z),
                                                            Eigen::Vector3d(end->x-start->x, end->y-start->y, end->z-start->z),
                                                            cylinderPtr->get_radius(),
                                                            cylinderPtr->get_length());



                CT_Cylinder* cylinder = new CT_Cylinder(_outCylinderModelName.completeName(), res_r, data);
                cylinderGroup->addItemDrawable(cylinder);

//                cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchIDModelName.completeName(), CT_AbstractCategory::DATA_ID, res_r, branchID));
//                cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchOrderModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, res_r, branchOrder));
//                cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_segmentIDModelName.completeName(), CT_AbstractCategory::DATA_ID, res_r, segmentID));
//                cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_parentSegmentIDModelName.completeName(), CT_AbstractCategory::DATA_ID, res_r, parentSegmentID));
//                cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(_growthVolumeModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, res_r, growthVolume));


            }

//            std::vector<pcl::ModelCoefficients >::const_iterator itS = spheres.begin();
//            std::vector<pcl::ModelCoefficients >::const_iterator endS = spheres.end();
//            while (itS != endS)
//            {
//                const pcl::ModelCoefficients &sph = (*itS);

//                CT_StandardItemGroup* sphereGroup = new CT_StandardItemGroup(_outSphereGroupModelName.completeName(), res_r);
//                treeGroup->addGroup(sphereGroup);

//                CT_SphereData *data = new CT_SphereData(Eigen::Vector3d(sph.values[0] + min(0), sph.values[1] + min(1), sph.values[2] + min(2) - _zoffset), sph.values[3]);

//                CT_Sphere* item_c = new CT_Sphere(_outSphereModelName.completeName(), res_r, data);

//                sphereGroup->addItemDrawable(item_c);

//                ++itS;
//            }

//            CT_AttributesList *attrList = new CT_AttributesList(_outTreeAttributesModelName.completeName(), res_r);
//            treeGroup->addItemDrawable(attrList);

//            attrList->addItemAttribute(new CT_StdItemAttributeT<float>(_totalVolumeModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, res_r, tree.getVolume()));
//            attrList->addItemAttribute(new CT_StdItemAttributeT<float>(_solidVolumeModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, res_r, tree.getSolidVolume()));
//            attrList->addItemAttribute(new CT_StdItemAttributeT<float>(_totalStemVolumeModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, res_r, tree.getStemVolume()));
//            attrList->addItemAttribute(new CT_StdItemAttributeT<float>(_heightModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, res_r, tree.getHeight()));
//            attrList->addItemAttribute(new CT_StdItemAttributeT<float>(_lengthModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, res_r, tree.getLength()));
//            attrList->addItemAttribute(new CT_StdItemAttributeT<float>(_DBHModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, res_r, tree.getDBH()));
//            attrList->addItemAttribute(new CT_StdItemAttributeT<float>(_baseDiameterModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, res_r, tree.getBaseDiameter()));
//            attrList->addItemAttribute(new CT_StdItemAttributeT<float>(_heightAboveGroundModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, res_r, tree.getHeightAboveGround()));
//            attrList->addItemAttribute(new CT_StdItemAttributeT<float>(_volumeUntilFirstBranchModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, res_r, tree.getRootSegmentVolume()));
//            attrList->addItemAttribute(new CT_StdItemAttributeT<float>(_volumeUntilCrownModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, res_r, tree.getVolumeToRoot(tree.getFirstCrownSegment())));
//            attrList->addItemAttribute(new CT_StdItemAttributeT<float>(_crownVolumeModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, res_r, tree.crown->volume));
//            attrList->addItemAttribute(new CT_StdItemAttributeT<float>(_crownProjectionAreaModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, res_r, tree.crown->crownProjectionArea));
//            attrList->addItemAttribute(new CT_StdItemAttributeT<float>(_crownSurfaceAreaModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, res_r, tree.crown->area));


        }



    }
}

void ST_StepDetectTree::setProgressWrapper(int ix)
{
    setProgress(ix);
}

//CT_TTreeGroup* ST_StepDetectTree::constructTopology(const Eigen::Vector3d &min, const CT_AbstractResult *res_r, boost::shared_ptr<simpleTree::Tree> tree_ptr)
//{
//    simpleTree::Tree &tree = *tree_ptr;
//    boost::shared_ptr<Segment> topSegment = tree.getRootSegment();

//    CT_TTreeGroup *topology = new CT_TTreeGroup(_topologyGroup.completeName(), res_r);

//    CT_TNodeGroup *root = new CT_TNodeGroup(_rootTreeGroup.completeName(), res_r);
//    topology->setRootNode(root);

//    setCylinders(min, res_r, root, topSegment,tree_ptr);
//            constructTopologyRecursively(min,res_r,root, topSegment, tree_ptr);

////    std::vector<boost::shared_ptr<Segment> > &childrens = topSegment->getChildren();

////    std::vector<boost::shared_ptr<simpleTree::Segment> >::const_iterator it = childrens.begin();
////    std::vector<boost::shared_ptr<simpleTree::Segment> >::const_iterator end = childrens.end();

////    while(it != end) {
////        boost::shared_ptr<Segment> childSegment = *it;

////        CT_TNodeGroup* branchGroup = new CT_TNodeGroup(_branchGroup.completeName(), res_r);
////        root->addBranch(branchGroup);

////        setCylinders(min, res_r, branchGroup, childSegment,tree_ptr);
////        constructTopologyRecursively(min,res_r,branchGroup, childSegment, tree_ptr);

////        ++it;
////    }

//    return topology;
//}

void ST_StepDetectTree::enrich_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp)
{
    const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();

    size_t size = index->size();
    CT_NormalCloudStdVector *normalCloud = new CT_NormalCloudStdVector( size );
    CT_StandardCloudStdVectorT<float> *curvaturesCloud = new CT_StandardCloudStdVectorT<float>(size);
    CT_StandardCloudStdVectorT<float> *eigen1Cloud = new CT_StandardCloudStdVectorT<float>(size);
    CT_StandardCloudStdVectorT<float> *eigen2Cloud = new CT_StandardCloudStdVectorT<float>(size);
    CT_StandardCloudStdVectorT<float> *eigen3Cloud = new CT_StandardCloudStdVectorT<float>(size);
    CT_StandardCloudStdVectorT<float> *stemCloud = new CT_StandardCloudStdVectorT<float>(size);

    for(size_t i =0; i < size; i ++)
    {
        PointS p = _cloud->points[i];
        float n1 = p.normal_x;
        float n2 = p.normal_y;
        float n3 = p.normal_z;
        float eigen1 = p.eigen1;
        float eigen2 = p.eigen2;
        float eigen3 = p.eigen3;

        CT_Normal &ctNormal = normalCloud->normalAt(i);
        ctNormal.x() = n1;
        ctNormal.y() = n2;
        ctNormal.z() = n3;
        ctNormal.w() = p.curvature;

        curvaturesCloud->tAt(i) =p.curvature;
        eigen1Cloud->tAt(i) = eigen1;
        eigen2Cloud->tAt(i) = eigen2;
        eigen3Cloud->tAt(i) = eigen3;

    }
    CT_PointsAttributesNormal * normals = new CT_PointsAttributesNormal(_cloud_out_normals.completeName(), resCpy_res, itemCpy_cloud_in->getPointCloudIndexRegistered(),normalCloud);
    CT_PointsAttributesScalarTemplated<float> * curvatures =  new CT_PointsAttributesScalarTemplated<float>
            (_cloud_out_curvature.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(), curvaturesCloud);
    CT_PointsAttributesScalarTemplated<float> * eigenvalues1 =  new CT_PointsAttributesScalarTemplated<float>
            (_cloud_out_eigen1.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(), eigen1Cloud);
    CT_PointsAttributesScalarTemplated<float> * eigenvalues2 =  new CT_PointsAttributesScalarTemplated<float>
            (_cloud_out_eigen2.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(), eigen2Cloud);
    CT_PointsAttributesScalarTemplated<float> * eigenvalues3 =  new CT_PointsAttributesScalarTemplated<float>
            (_cloud_out_eigen3.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(), eigen3Cloud);
    CT_PointsAttributesScalarTemplated<float> * stem =  new CT_PointsAttributesScalarTemplated<float>
            (_cloud_out_stem.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(),stemCloud);


    grpCpy_grp->addItemDrawable(normals);
    grpCpy_grp->addItemDrawable(curvatures);
    grpCpy_grp->addItemDrawable(eigenvalues1);
    grpCpy_grp->addItemDrawable(eigenvalues2);
    grpCpy_grp->addItemDrawable(eigenvalues3);
    grpCpy_grp->addItemDrawable(stem);
}

void ST_StepDetectTree::create_simple_tree_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in)
{
    if (itemCpy_cloud_in != NULL)
    {
        const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();

        size_t size = 0;
        size = index->size();
        _cloud.reset(new PointCloudS);
        _cloud->width = size;
        _cloud->height = 1;
        if(size > 0) {
            _cloud->points.resize(size);
            size_t i = 0;
            CT_PointIterator it (index);
            while(it.hasNext())
            {
                const CT_PointData &internalPoint = it.next().currentConstInternalPoint();
                PointS p;
                p.x =internalPoint[0];
                p.y =internalPoint[1];
                p.z =internalPoint[2];
                _cloud->points[i] = p;
                ++i;
            }
        }
        EnrichCloud enrich(_cloud, _knn, _range, _use_knn);
    }


}

