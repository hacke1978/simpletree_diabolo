#include "st_stepcompletemodellingfolder.h"
#include "ct_itemdrawable/ct_fileheader.h"



// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"
#define DEFin_header "header"



// Constructor : initialization of parameters
ST_StepCompleteModellingFolder::ST_StepCompleteModellingFolder(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _knn = 4;
    _sphereRadiusMultiplier = 2;
    _epsilonClusterStem = 0.02;
    _epsilonClusterBranch = 0.02;
    _epsilonSphere = 0.035;
    _minPtsRansacStem = 50;
    _minPtsRansacBranch = 50;
    _minPtsClusterStem = 3;
    _minPtsClusterBranch = 3;
    _minRadiusSphere = 0.07;
    _iterationNumber = 1;
    _zoffset = .1;
    _a = 35.671;
    _b = 2.204;
    _fact = 2;
    _minRad = 0.0025;
    _criterion = 0.0001;
    _iterations = 2;
    _seeds = 21;

    _species = "unknown";
    _id      = "unknown_001";



    _sceneList = new QList<CT_Scene*>();

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

ST_StepCompleteModellingFolder::~ST_StepCompleteModellingFolder()
{
    _sceneList->clear();
    delete _sceneList;
}

// Step description (tooltip of contextual menu)
QString ST_StepCompleteModellingFolder::getStepDescription() const
{
    return tr("Complete Modelling for one folder.");
}

// Step detailled description
QString ST_StepCompleteModellingFolder::getStepDetailledDescription() const
{
    return tr("See for now SimpleTree homepage." );
}

// Step URL
QString ST_StepCompleteModellingFolder::getStepURL() const
{
    //return tr("STEP URL HERE");
    return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepCompleteModellingFolder::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepCompleteModellingFolder(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepCompleteModellingFolder::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Isolated Tree"));
    resIn_res->addItemModel(DEFin_grp, DEFin_header, CT_FileHeader::staticGetType(), tr("File Header"));

}

// Creation and affiliation of OUT models
void ST_StepCompleteModellingFolder::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addGroupModel(DEFin_grp, _topologyGroup_allom, new CT_TTreeGroup(), tr("Topology_allom"));
        resCpy_res->addGroupModel(_topologyGroup_allom, _rootTreeGroup_allom, new CT_TNodeGroup(), tr("Root_allom"));
        //        resCpy_res->addGroupModel(_topologyGroup_allom, _branchGroup_allom, new CT_TNodeGroup(), tr("Branch_allom"));
        resCpy_res->addGroupModel(_topologyGroup_allom, _stemGroup_allom, new CT_TNodeGroup(), tr("Stem_allom"));
        resCpy_res->addItemModel(_stemGroup_allom, _stemCylinders_allom, new CT_Cylinder(), tr("Stem cylinders_allom"));


        resCpy_res->addGroupModel(DEFin_grp, _topologyGroup_fit, new CT_TTreeGroup(), tr("Topology_fit"));
        resCpy_res->addGroupModel(_topologyGroup_fit, _rootTreeGroup_fit, new CT_TNodeGroup(), tr("Root_fit"));
        //        resCpy_res->addGroupModel(_topologyGroup_fit, _branchGroup_fit, new CT_TNodeGroup(), tr("Branch_fit"));
        resCpy_res->addGroupModel(_topologyGroup_fit, _stemGroup_fit, new CT_TNodeGroup(), tr("Stem_fit"));
        resCpy_res->addItemModel(_stemGroup_fit, _stemCylinders_fit, new CT_Cylinder(), tr("Stem cylinders_fit"));


        //        resCpy_res->addGroupModel(DEFin_grp, _outCylinderGroupModelName, new CT_StandardItemGroup(), tr("Cylinder group"));
        //        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName, new CT_Cylinder(), tr("Cylinder"));







        resCpy_res->addItemModel(DEFin_grp, _cloud_out_normals, new CT_PointsAttributesNormal(),tr("Normals"));
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_curvature, new CT_PointsAttributesScalarTemplated<float>(),tr("Curvature"));
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_eigen1, new CT_PointsAttributesScalarTemplated<float>(),tr("Eigenvalue1"));
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_eigen2, new CT_PointsAttributesScalarTemplated<float>(),tr("Eigenvalue2"));
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_eigen3, new CT_PointsAttributesScalarTemplated<float>(),tr("Eigenvalue3"));
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_stem, new CT_PointsAttributesScalarTemplated<float>(),tr("Stem flag"));
        //resCpy_res->addItemModel(DEFin_grp, _cloud_out_ID, new CT_PointsAttributesScalarTemplated<int>(),tr("TreeID"));


        resCpy_res->addGroupModel(DEFin_grp, _outCylinderGroupModelName, new CT_StandardItemGroup(), tr("Cylinder group"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_unimproved, new CT_Cylinder(), tr("Cylinder_unimproved"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_removed_false_cylinders, new CT_Cylinder(), tr("Cylinder__removed_false_cylinder"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_improved_branch_junctions, new CT_Cylinder(), tr("Cylinder_corrected_branch_junctions"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_improved_by_merge, new CT_Cylinder(), tr("Cylinder_merged"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_removed_improved_by_median, new CT_Cylinder(), tr("Cylinder_improved_by_median"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_improved_by_fit, new CT_Cylinder(), tr("Cylinder_improved_by_fit"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_improved_by_allometry, new CT_Cylinder(), tr("Cylinder_by_allometry"));



        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_fit, _branchIDModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("branch_ID"));


        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_fit, _branchOrderModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("branch_order"));


        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_fit, _segmentIDModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("segment_ID"));


        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_fit, _parentSegmentIDModelName,
                                          new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_ID),
                                          tr("parent_segment_ID"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_fit, _growthVolumeModelName,
                                          new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("growth_volume"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_fit, _tree_species,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_VALUE),
                                          tr("tree_species"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_fit, _tree_id,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_VALUE),
                                          tr("tree_id"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_fit, _detection_type,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("detection_method"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_fit, _improvement_type,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("improvement_method"));






        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _branchIDModelName2,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("branch_ID"));


        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _branchOrderModelName2,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("branch_order"));


        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _segmentIDModelName2,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("segment_ID"));


        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _parentSegmentIDModelName2,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("parent_segment_ID"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _growthVolumeModelName2,
                                          new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("growth_volume"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _tree_species2,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_VALUE),
                                          tr("tree_species"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _tree_id2,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_VALUE),
                                          tr("tree_id"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _detection_type2,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("detection_method"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _improvement_type2,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("improvement_method"));
    }
}

void ST_StepCompleteModellingFolder::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();

    configDialog->addEmpty();
    configDialog->addEmpty();
    configDialog->addEmpty();
    configDialog->addFileChoice(tr("Species, ID, etc"), CT_FileChoiceButton::OneExistingFile, "Extension (*.csv)", _file_name_list);
    configDialog->addEmpty();
    configDialog->addEmpty();
    configDialog->addEmpty();
}

void ST_StepCompleteModellingFolder::compute()
{
    // DONT'T FORGET TO ADD THIS STEP TO THE PLUGINMANAGER !!!!!

    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);



    // IN results browsing


    // COPIED results browsing
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    QString file = _file_name_list.at(0);
    ReadCSV read_csv (file);
    QMap<QString, FileCoefficients> map;
    map = read_csv.get_map();
    while (itCpy_grp.hasNext() && !isStopped())
    {
        qDebug() << "start";
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();


        const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        const CT_FileHeader * itemCpy_header = (CT_FileHeader*)grpCpy_grp->firstItemByINModelName(this, DEFin_header);
        QString cloud_name = itemCpy_header->getFileName();
        FileCoefficients coeff_file = map.value(cloud_name);
        _species = coeff_file.species;

        _id      = coeff_file.file;

        _use_allom = coeff_file.use_allom;
        _out_put_path = coeff_file.output_path;


        _knn =16;
        create_simple_tree_cloud(itemCpy_cloud_in);
        // enrich_cloud(itemCpy_cloud_in,resCpy_res,grpCpy_grp);
        build_tree_model(resCpy_res,grpCpy_grp);
        qDebug() << "fini";


    }
}

void ST_StepCompleteModellingFolder::add_cylinder_data(Tree tree, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp, QString string)
{
    QVector<QSharedPointer<Cylinder> > cylinders = tree.get_all_cylinders();

    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);

    PS_LOG->addInfoMessage(this, tr("Computree data creation"));

    while(it.hasNext()) {
        QSharedPointer<Cylinder> cylin = it.next();
        int branchID = cylin->get_segment()->get_branch_id();
        int branchOrder = cylin->get_segment()->get_branch_order();
        int segmentID = cylin->get_segment()->get_id();
        int parentSegmentID = -1;
        if (!(cylin->get_segment()->is_root()))
        {
            parentSegmentID = cylin->get_segment()->get_parent_segment()->get_id();
        }
        float growthVolume = tree.get_growth_volume(cylin);

        int detection = cylin->get_detection();

        int improvement = cylin->get_improvement();


        CT_StandardItemGroup* cylinderGroup = new CT_StandardItemGroup(_outCylinderGroupModelName.completeName(), resCpy_res);
        grpCpy_grp->addGroup(cylinderGroup);


        QSharedPointer<PointS> center = cylin->get_center();
        QSharedPointer<PointS> start = cylin->get_start();
        QSharedPointer<PointS> stop = cylin->get_end();


        CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                    Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
                                                    cylin->get_radius(),
                                                    cylin->get_length());



        CT_Cylinder* cylinder = new CT_Cylinder(string, resCpy_res, data);
        cylinderGroup->addItemDrawable(cylinder);
        //qDebug() << "in loop";
        //qDebug() << string;
        //qDebug() << _outCylinderModelName_improved_by_fit.completeName();
        if(string == _outCylinderModelName_improved_by_fit.completeName())
        {
            //  qDebug() << "01";
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, branchID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchOrderModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, branchOrder));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_segmentIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, segmentID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_parentSegmentIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, parentSegmentID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(_growthVolumeModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, growthVolume));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<QString>(_tree_species.completeName(), CT_AbstractCategory::DATA_VALUE, resCpy_res, _species));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<QString>(_tree_id.completeName(), CT_AbstractCategory::DATA_VALUE, resCpy_res, _id));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_detection_type.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, detection));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_improvement_type.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, improvement));

        }
        else if (string == _outCylinderModelName_improved_by_allometry.completeName()) {
            //    qDebug() << "02";
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchIDModelName2.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, branchID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchOrderModelName2.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, branchOrder));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_segmentIDModelName2.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, segmentID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_parentSegmentIDModelName2.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, parentSegmentID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(_growthVolumeModelName2.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, growthVolume));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<QString>(_tree_species2.completeName(), CT_AbstractCategory::DATA_VALUE, resCpy_res, _species));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<QString>(_tree_id2.completeName(), CT_AbstractCategory::DATA_VALUE, resCpy_res, _id));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_detection_type2.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, detection));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_improvement_type2.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, improvement));
        }

    }
}

CT_TTreeGroup *ST_StepCompleteModellingFolder::constructTopology(const CT_AbstractResult *res_r, QSharedPointer<Tree> tree, QString string)
{
    QSharedPointer<Segment> segment = tree->get_root_segment();

    CT_TTreeGroup *topology = new CT_TTreeGroup(string, res_r);
    if(string == _topologyGroup_allom.completeName())
    {
        CT_TNodeGroup *root = new CT_TNodeGroup(_stemGroup_allom.completeName(), res_r);
        topology->setRootNode(root);
        setCylinders(res_r, root, segment,tree, string);
        constructTopologyRecursively(res_r,root, segment, tree,string);

    }
    else if (string == _topologyGroup_fit.completeName())
    {
        CT_TNodeGroup *root = new CT_TNodeGroup(_stemGroup_fit.completeName(), res_r);
        topology->setRootNode(root);
        setCylinders(res_r, root, segment,tree, string);
        constructTopologyRecursively(res_r,root, segment, tree,string);
    }
    return topology;
}

void ST_StepCompleteModellingFolder::constructTopologyRecursively(const CT_AbstractResult *res_r, CT_TNodeGroup *parent, QSharedPointer<Segment> segment, QSharedPointer<Tree> tree, QString string)
{
    QVector<QSharedPointer<Segment> > children = segment->get_child_segments();
    QVectorIterator<QSharedPointer<Segment> > it (children);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment_child = it.next();
        if(string == _topologyGroup_allom.completeName())
        {
            CT_TNodeGroup* branchGroup = new CT_TNodeGroup(_stemGroup_allom.completeName(), res_r);
            parent->addBranch(branchGroup);
            setCylinders(res_r, branchGroup, segment_child, tree, string);
            constructTopologyRecursively(res_r,branchGroup, segment_child,tree,string);

        }
        else if (string == _topologyGroup_fit.completeName())
        {
            CT_TNodeGroup* branchGroup = new CT_TNodeGroup(_stemGroup_fit.completeName(), res_r);
            parent->addBranch(branchGroup);
            setCylinders(res_r, branchGroup, segment_child, tree, string);
            constructTopologyRecursively(res_r,branchGroup, segment_child,tree,string);
        }
    }
}

void ST_StepCompleteModellingFolder::setCylinders(const CT_AbstractResult *res_r, CT_TNodeGroup *root, QSharedPointer<Segment> segment, QSharedPointer<Tree> tree, QString string)
{

    QVector<QSharedPointer<Cylinder> >cylinders = segment->get_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylin = it.next();

        if(string == _topologyGroup_allom.completeName())
        {
            CT_TNodeGroup* cylinderGroup = new CT_TNodeGroup(_stemGroup_allom.completeName(), res_r);
            root->addComponent(cylinderGroup);
            QSharedPointer<PointS> center = cylin->get_center();
            QSharedPointer<PointS> start = cylin->get_start();
            QSharedPointer<PointS> stop = cylin->get_end();
            CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                        Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
                                                        cylin->get_radius(),
                                                        cylin->get_length());
            CT_Cylinder* cylinder = new CT_Cylinder(_stemCylinders_allom.completeName(), res_r, data);
            cylinderGroup->addItemDrawable(cylinder);

        }
        else if (string == _topologyGroup_fit.completeName())
        {
            CT_TNodeGroup* cylinderGroup = new CT_TNodeGroup(_stemGroup_fit.completeName(), res_r);
            root->addComponent(cylinderGroup);
            QSharedPointer<PointS> center = cylin->get_center();
            QSharedPointer<PointS> start = cylin->get_start();
            QSharedPointer<PointS> stop = cylin->get_end();
            CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                        Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
                                                        cylin->get_radius(),
                                                        cylin->get_length());
            CT_Cylinder* cylinder = new CT_Cylinder(_stemCylinders_fit.completeName(), res_r, data);
            cylinderGroup->addItemDrawable(cylinder);

        }
    }
}


void ST_StepCompleteModellingFolder::build_tree_model(CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp)
{
    MethodCoefficients coeff;
    coeff.sphere_radius_multiplier = _sphereRadiusMultiplier;
    coeff.epsilon_cluster_branch = _epsilonClusterBranch;
    coeff.epsilon_cluster_stem = _epsilonClusterStem;
    coeff.epsilon_sphere = _epsilonSphere;
    coeff.min_radius_sphere = _minRadiusSphere;
    coeff.minPts_cluster_branch = _minPtsClusterBranch;
    coeff.minPts_cluster_stem = _minPtsClusterStem;
    coeff.a = _a;
    coeff.b = _b;
    coeff.id = _id;
    coeff.species = _species;

    float z_min = std::numeric_limits<float>::max();
    float z_max = std::numeric_limits<float>::lowest();
    for(size_t i = 0; i < _cloud->points.size(); i++)
    {
        PointS p = _cloud->points.at(i);
        if(p.z < z_min)
        {
            z_min = p.z;
        }

        if(p.z > z_max)
        {
            z_max = p.z;
        }
    }
    ComputeMeanAndStandardDeviation m_sd(_cloud);
    coeff.sd = m_sd._sd;
    coeff.mean = m_sd._mean;
    coeff.epsilon_cluster_branch = 3*(m_sd._mean + 2*m_sd._sd);

    qDebug() << "percent of stem" << _percent_stem;
    if(_percent_stem>15&&_percent_stem<90)
    {
        coeff.optimze_stem = true;
    }

    float height = z_max - z_min;
    QVector<pcl::ModelCoefficients> cylinder_coeff;


    if(height > 10)
    {
        coeff.epsilon_sphere = 0.03;
    } else
    {

    }
    SphereFollowingRecursive spherefollowing( _cloud,coeff, true);
    coeff = spherefollowing.get_coeff();

    cylinder_coeff = spherefollowing.get_cylinders();







    BuildTree builder(cylinder_coeff);

    QSharedPointer<Tree> tree (new Tree(builder.getRoot_segment()));

    add_cylinder_data(*tree, resCpy_res, grpCpy_grp, _outCylinderModelName_unimproved.completeName());

    RemoveFalseCylinders remove(tree);
    add_cylinder_data(*tree, resCpy_res, grpCpy_grp, _outCylinderModelName_removed_false_cylinders.completeName());


    add_cylinder_data(*tree, resCpy_res, grpCpy_grp, _outCylinderModelName_improved_branch_junctions.completeName());


    ReorderTree reorder(tree);
    ImproveByMedian improve_by_median(tree);

    add_cylinder_data(*tree, resCpy_res, grpCpy_grp, _outCylinderModelName_removed_improved_by_median.completeName());
    ImproveByMerge improve_merge(tree);

    add_cylinder_data(*tree, resCpy_res, grpCpy_grp, _outCylinderModelName_improved_by_merge.completeName());

    ImproveByPipeModel pype(tree);


    ImproveFit fit(tree,_cloud,coeff);

    tree = spherefollowing.get_tree();

    ImprovedByAdvancedMedian improve_by_median2(tree);

    ReorderTree reorder4(tree);
    if(_use_allom=="FALSE")
    {
        ExportTree exporttree(tree,coeff,_out_put_path);
    }

    add_cylinder_data(*tree, resCpy_res, grpCpy_grp, _outCylinderModelName_improved_by_fit.completeName());
    CT_TTreeGroup *ctTree  = constructTopology(resCpy_res,tree, _topologyGroup_fit.completeName());
    if(ctTree != NULL)
        grpCpy_grp->addGroup(ctTree);

    ComputeAllometry ca2(tree);
    coeff.a = ca2.get_a();
    coeff.b = ca2.get_b();
    ImproveByAllometry(tree,coeff.a,coeff.b);
    ComputeAllometry ca3(tree,true);
    coeff.a = ca3.get_a();
    coeff.b = ca3.get_b();
    ImproveByAllometry(tree,coeff.a,coeff.b);
    ComputeAllometry ca4(tree,true);
    coeff.a = ca4.get_a();
    coeff.b = ca4.get_b();
    ImproveByAllometry(tree,coeff.a,coeff.b,1.3f,true);

    add_cylinder_data(*tree, resCpy_res, grpCpy_grp, _outCylinderModelName_improved_by_allometry.completeName());
    CT_TTreeGroup *ctTree2 = constructTopology(resCpy_res,tree, _topologyGroup_allom.completeName());
    if(ctTree2 != NULL)
        grpCpy_grp->addGroup(ctTree);


    if(_use_allom=="TRUE")
    {
        ExportTree exporttree(tree,coeff,_out_put_path);
    }
}


void ST_StepCompleteModellingFolder::enrich_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp)
{

    const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();

    size_t size = index->size();
    CT_NormalCloudStdVector *normalCloud = new CT_NormalCloudStdVector( size );
    CT_StandardCloudStdVectorT<float> *curvaturesCloud = new CT_StandardCloudStdVectorT<float>(size);
    CT_StandardCloudStdVectorT<float> *eigen1Cloud = new CT_StandardCloudStdVectorT<float>(size);
    CT_StandardCloudStdVectorT<float> *eigen2Cloud = new CT_StandardCloudStdVectorT<float>(size);
    CT_StandardCloudStdVectorT<float> *eigen3Cloud = new CT_StandardCloudStdVectorT<float>(size);
    CT_StandardCloudStdVectorT<float> *stemCloud = new CT_StandardCloudStdVectorT<float>(size);
    //  CT_StandardCloudStdVectorT<int> *IDCloud = new CT_StandardCloudStdVectorT<int>(size);

    pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS>);
    tree->setInputCloud (_cloud);



    CT_PointIterator it (index);





    for(size_t i =0; i < size; i ++)
    {

        const CT_PointData &internalPoint =  it.next().currentConstInternalPoint();
        PointS query;
        query.x =internalPoint[0];
        query.y =internalPoint[1];
        query.z =internalPoint[2];

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        tree->nearestKSearch(query,1,pointIdxRadiusSearch,pointRadiusSquaredDistance);


        PointS p = _cloud->points[pointIdxRadiusSearch[0]];
        float n1 = p.normal_x;
        float n2 = p.normal_y;
        float n3 = p.normal_z;
        float eigen1 = p.eigen1;
        float eigen2 = p.eigen2;
        float eigen3 = p.eigen3;
        float stem = p.is_stem;
        //   int ID = p.treeID;

        CT_Normal &ctNormal = normalCloud->normalAt(i);
        ctNormal.x() = n1;
        ctNormal.y() = n2;
        ctNormal.z() = n3;
        ctNormal.w() = p.curvature;

        curvaturesCloud->tAt(i) =p.curvature;
        eigen1Cloud->tAt(i) = eigen1;
        eigen2Cloud->tAt(i) = eigen2;
        eigen3Cloud->tAt(i) = eigen3;
        stemCloud->tAt(i) = stem;

    }
    CT_PointsAttributesNormal * normals = new CT_PointsAttributesNormal(_cloud_out_normals.completeName(), resCpy_res, itemCpy_cloud_in->getPointCloudIndexRegistered(),normalCloud);
    CT_PointsAttributesScalarTemplated<float> * curvatures =  new CT_PointsAttributesScalarTemplated<float>
            (_cloud_out_curvature.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(), curvaturesCloud);
    CT_PointsAttributesScalarTemplated<float> * eigenvalues1 =  new CT_PointsAttributesScalarTemplated<float>
            ( _cloud_out_eigen1.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(), eigen1Cloud);
    CT_PointsAttributesScalarTemplated<float> * eigenvalues2 =  new CT_PointsAttributesScalarTemplated<float>
            (_cloud_out_eigen2.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(), eigen2Cloud);
    CT_PointsAttributesScalarTemplated<float> * eigenvalues3 =  new CT_PointsAttributesScalarTemplated<float>
            (_cloud_out_eigen3.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(), eigen3Cloud);
    CT_PointsAttributesScalarTemplated<float> * stem =  new CT_PointsAttributesScalarTemplated<float>
            (_cloud_out_stem.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(),stemCloud);
    //    CT_PointsAttributesScalarTemplated<int> * ids =  new CT_PointsAttributesScalarTemplated<int>
    //            (_cloud_out_ID.completeName(), resCpy_res,itemCpy_cloud_in->getPointCloudIndexRegistered(),IDCloud);


    grpCpy_grp->addItemDrawable(normals);
    grpCpy_grp->addItemDrawable(curvatures);
    grpCpy_grp->addItemDrawable(eigenvalues1);
    grpCpy_grp->addItemDrawable(eigenvalues2);
    grpCpy_grp->addItemDrawable(eigenvalues3);
    grpCpy_grp->addItemDrawable(stem);
    // grpCpy_grp->addItemDrawable(ids);
}

void ST_StepCompleteModellingFolder::create_simple_tree_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in)
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
//        VoxelGridFilter voxelgrid (_cloud, 0.005f);
//        voxelgrid.compute();
//        _cloud = voxelgrid.get_cloud_out();
        EnrichCloud enrich(_cloud, _knn, 0.03, true);
        StemPointDetection stempts (0,0.1,0.35,1,0,0.8,0.035,_cloud,1) ;
        stempts.compute();
        _percent_stem = stempts.get_percentage();

    }


}



