#include "st_stepabstractmodellingmt.h"




QVector<SimpleTreeMT>  ST_StepAbstractModellingMT::st_mt_vec;
QMutex ST_StepAbstractModellingMT::lock;

// Constructor : initialization of parameters
ST_StepAbstractModellingMT::ST_StepAbstractModellingMT(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _knn = 4;
    _sceneList = new QList<CT_Scene*>();

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

void ST_StepAbstractModellingMT::update_vec(int i, MethodCoefficients coeff, QVector<pcl::ModelCoefficients> cylinder_coeff)
{
    lock.lock();
    st_mt_vec[i].coeff = coeff;
    lock.unlock();
    PointCloudS::Ptr cloud = st_mt_vec[i].cloud;

    CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in = st_mt_vec[i].itemCpy_cloud_in;
    CT_ResultGroup* resCpy_res = st_mt_vec[i].resCpy_res;
    CT_StandardItemGroup* grpCpy_grp = st_mt_vec[i].grpCpy_grp;
    FileCoefficients file_coeff = st_mt_vec[i].file_coeff;
    coeff.species = file_coeff.species;
    coeff.id = file_coeff.file;
    QString outputpath = file_coeff.output_path;

    SphereFollowing2 sphere (coeff,cloud,true);
    sphere.sphere_following();
    cylinder_coeff = sphere.get_cylinders();

    ImproveByAttractor ia (cloud,sphere.get_remaining_points(),coeff,cylinder_coeff);
    cylinder_coeff = ia.get_cylinders();
        lock.lock();
    st_mt_vec[i].cylinder_coeff = cylinder_coeff;
lock.unlock();

    BuildTree builder(cylinder_coeff);

    QSharedPointer<Tree> tree (new Tree(builder.getRoot_segment(), coeff.id));
    PS_LOG->addInfoMessage(this, tr("Computree data creation"));

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


    ImproveFit fit(tree,cloud,coeff);


    ImprovedByAdvancedMedian improve_by_median2(tree);
    ImproveBranchJunctions improve_junctions(tree);
    ReorderTree reorder4(tree);
    lock.lock();
    if(file_coeff.use_allom=="FALSE"||(!coeff.use_allom))
    {
        ExportTree exporttree(tree,coeff,outputpath);
        add_cylinder_data(*tree, resCpy_res, grpCpy_grp, _outCylinderModelName_improved_by_allometry.completeName());
        CT_TTreeGroup *ctTree2 = constructTopology(resCpy_res,tree, _topologyGroup.completeName());
        if(ctTree2 != NULL)
            grpCpy_grp->addGroup(ctTree2);

    }
    else
    {
        ExportTree exporttree(tree,coeff,outputpath,true);
        add_cylinder_data(*tree, resCpy_res, grpCpy_grp, _outCylinderModelName_improved_by_fit.completeName());
    }

    lock.unlock();

    ComputeAllometry ca2(tree);
    coeff.a = ca2.get_a();
    coeff.b = ca2.get_b();
    ImproveByAllometry(tree,coeff,coeff.a,coeff.b);

    ComputeAllometry ca3(tree,true);
    coeff.a = ca3.get_a();
    coeff.b = ca3.get_b();
    ImproveByAllometry(tree,coeff,coeff.a,coeff.b);

    ComputeAllometry ca4(tree,true);
    coeff.a = ca4.get_a();
    coeff.b = ca4.get_b();
    ImproveByAllometry(tree,coeff,coeff.a,coeff.b,1.3f);

lock.lock();
    if(file_coeff.use_allom=="TRUE"&&coeff.use_allom)
    {
        ExportTree exporttree(tree,coeff,outputpath);
        add_cylinder_data(*tree, resCpy_res, grpCpy_grp, _outCylinderModelName_improved_by_allometry.completeName());
        CT_TTreeGroup *ctTree2 = constructTopology(resCpy_res,tree, _topologyGroup.completeName());
        if(ctTree2 != NULL)
            grpCpy_grp->addGroup(ctTree2);

    }
    else
    {
        ExportTree exporttree(tree,coeff,outputpath,true);
        add_cylinder_data(*tree, resCpy_res, grpCpy_grp, _outCylinderModelName_improved_by_fit.completeName());
    }


    lock.unlock();
}

void ST_StepAbstractModellingMT::receive_counter_finished()
{
    _modelled_trees ++;
    int percentage = (((float)_modelled_trees)/((float)_number_trees)*100.0f);
    setProgress(percentage);
}

ST_StepAbstractModellingMT::~ST_StepAbstractModellingMT()
{
    _sceneList->clear();
    delete _sceneList;
}

void ST_StepAbstractModellingMT::receive_qstring_abstract_modelling(QString qstring)
{
    QString displayed_str = "__";
    displayed_str.append(QTime::currentTime().toString("hh:mm:ss"));
    qDebug() << QTime::currentTime().toString("hh:mm:ss");
    displayed_str.append("__ : ");
    displayed_str.append(qstring);
    PS_LOG->addInfoMessage(this, displayed_str);

}

void ST_StepAbstractModellingMT::receive_counter_abstract_modelling(int counter)
{
    setProgress(counter);
}

//// Step description (tooltip of contextual menu)
//QString ST_StepAbstractModelling::getStepDescription() const
//{
//    return tr("Complete Modelling for one folder.");
//}

//// Step detailled description
//QString ST_StepAbstractModelling::getStepDetailledDescription() const
//{
//    return tr("See for now SimpleTree homepage." );
//}

//// Step URL
//QString ST_StepAbstractModelling::getStepURL() const
//{
//    //return tr("STEP URL HERE");
//    return CT_AbstractStep::getStepURL(); //by default URL of the plugin
//}

// Step copy method
//CT_VirtualAbstractStep* ST_StepAbstractModelling::createNewInstance(CT_StepInitializeData &dataInit)
//{
//    return new ST_StepAbstractModelling(dataInit);
//}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepAbstractModellingMT::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Isolated Tree"));
    resIn_res->addItemModel(DEFin_grp, DEFin_header, CT_FileHeader::staticGetType(), tr("File Header"));

}

// Creation and affiliation of OUT models
void ST_StepAbstractModellingMT::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addGroupModel(DEFin_grp, _topologyGroup, new CT_TTreeGroup(), tr("Topology"));
        resCpy_res->addGroupModel(_topologyGroup, _stemGroup, new CT_TNodeGroup(), tr("Stem"));
        resCpy_res->addItemModel(_stemGroup, _stemCylinders, new CT_Cylinder(), tr("Stem cylinders"));


        resCpy_res->addItemModel(DEFin_grp, _cloud_out_normals, new CT_PointsAttributesNormal(),tr("Normals"));
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_curvature, new CT_PointsAttributesScalarTemplated<float>(),tr("Curvature"));
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_eigen1, new CT_PointsAttributesScalarTemplated<float>(),tr("Eigenvalue1"));
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_eigen2, new CT_PointsAttributesScalarTemplated<float>(),tr("Eigenvalue2"));
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_eigen3, new CT_PointsAttributesScalarTemplated<float>(),tr("Eigenvalue3"));
        resCpy_res->addItemModel(DEFin_grp, _cloud_out_stem, new CT_PointsAttributesScalarTemplated<float>(),tr("Stem flag"));


        resCpy_res->addGroupModel(DEFin_grp, _outCylinderGroupModelName, new CT_StandardItemGroup(), tr("Cylinder group"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_unimproved, new CT_Cylinder(), tr("Cylinder_unimproved"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_removed_false_cylinders, new CT_Cylinder(), tr("Cylinder__removed_false_cylinder"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_improved_branch_junctions, new CT_Cylinder(), tr("Cylinder_corrected_branch_junctions"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_improved_by_merge, new CT_Cylinder(), tr("Cylinder_merged"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_removed_improved_by_median, new CT_Cylinder(), tr("Cylinder_improved_by_median"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_improved_by_fit, new CT_Cylinder(), tr("Cylinder_improved_by_fit"));
        resCpy_res->addItemModel(_outCylinderGroupModelName, _outCylinderModelName_improved_by_allometry, new CT_Cylinder(), tr("Cylinder_by_allometry"));



        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _branchIDModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("branch_ID"));


        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _branchOrderModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("branch_order"));


        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _segmentIDModelName,
                                          new CT_StdItemAttributeT<int>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_ID), NULL, 0),
                                          tr("segment_ID"));


        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _parentSegmentIDModelName,
                                          new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_ID),
                                          tr("parent_segment_ID"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _growthVolumeModelName,
                                          new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
                                          tr("growth_volume"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _tree_species,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_VALUE),
                                          tr("tree_species"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _tree_id,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_VALUE),
                                          tr("tree_id"));

        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _detection_type,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("detection_method"));
        resCpy_res->addItemAttributeModel(_outCylinderModelName_improved_by_allometry, _improvement_type,
                                          new CT_StdItemAttributeT<QString>(CT_AbstractCategory::DATA_NUMBER),
                                          tr("improvement_method"));
    }
}



void ST_StepAbstractModellingMT::add_cylinder_data(Tree tree, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp, QString string)
{
    QVector<QSharedPointer<Cylinder> > cylinders = tree.get_all_cylinders();

    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);



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


        QSharedPointer<PointS> center = cylin->get_center_ptr();
        QSharedPointer<PointS> start = cylin->get_start_ptr();
        QSharedPointer<PointS> stop = cylin->get_end_ptr();


        CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                    Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
                                                    cylin->get_radius(),
                                                    cylin->get_length());



        CT_Cylinder* cylinder = new CT_Cylinder(string, resCpy_res, data);
        cylinderGroup->addItemDrawable(cylinder);
        if (string == _outCylinderModelName_improved_by_allometry.completeName())
        {
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, branchID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchOrderModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, branchOrder));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_segmentIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, segmentID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_parentSegmentIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, parentSegmentID));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(_growthVolumeModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, growthVolume));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_detection_type.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, detection));
            cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_improvement_type.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, improvement));
        }

    }
}

CT_TTreeGroup *ST_StepAbstractModellingMT::constructTopology(const CT_AbstractResult *res_r, QSharedPointer<Tree> tree, QString string)
{
    QSharedPointer<Segment> segment = tree->get_root_segment();

    CT_TTreeGroup *topology = new CT_TTreeGroup(string, res_r);
    if(string == _topologyGroup.completeName())
    {
        CT_TNodeGroup *root = new CT_TNodeGroup(_stemGroup.completeName(), res_r);
        topology->setRootNode(root);
        setCylinders(res_r, root, segment,tree, string);
        constructTopologyRecursively(res_r,root, segment, tree,string);

    }
    return topology;
}

void ST_StepAbstractModellingMT::constructTopologyRecursively(const CT_AbstractResult *res_r, CT_TNodeGroup *parent, QSharedPointer<Segment> segment, QSharedPointer<Tree> tree, QString string)
{
    QVector<QSharedPointer<Segment> > children = segment->get_child_segments();
    QVectorIterator<QSharedPointer<Segment> > it (children);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment_child = it.next();
        if(string == _topologyGroup.completeName())
        {
            CT_TNodeGroup* branchGroup = new CT_TNodeGroup(_stemGroup.completeName(), res_r);
            parent->addBranch(branchGroup);
            setCylinders(res_r, branchGroup, segment_child, tree, string);
            constructTopologyRecursively(res_r,branchGroup, segment_child,tree,string);

        }
    }
}

void ST_StepAbstractModellingMT::setCylinders(const CT_AbstractResult *res_r, CT_TNodeGroup *root, QSharedPointer<Segment> segment, QSharedPointer<Tree> tree, QString string)
{

    QVector<QSharedPointer<Cylinder> >cylinders = segment->get_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylin = it.next();

        if(string == _topologyGroup.completeName())
        {
            CT_TNodeGroup* cylinderGroup = new CT_TNodeGroup(_stemGroup.completeName(), res_r);
            root->addComponent(cylinderGroup);
            QSharedPointer<PointS> center = cylin->get_center_ptr();
            QSharedPointer<PointS> start = cylin->get_start_ptr();
            QSharedPointer<PointS> stop = cylin->get_end_ptr();
            CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                        Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
                                                        cylin->get_radius(),
                                                        cylin->get_length());
            CT_Cylinder* cylinder = new CT_Cylinder(_stemCylinders.completeName(), res_r, data);
            cylinderGroup->addItemDrawable(cylinder);
        }
    }
}

QMap<QString, FileCoefficients> ST_StepAbstractModellingMT::get_map()
{
    QMap<QString, FileCoefficients> map;
    if(!_file_name_list.empty())
    {
        QString file = _file_name_list.at(0);
        ReadCSV read_csv (file);
        map = read_csv.get_map();
        QString success = "Loaded the Map file successful and crate a map with ";
        success.append(QString::number(map.size()));
        success.append("entries.");
        PS_LOG->addInfoMessage(this, success);
    }
    else
    {
        QString path = "D:/Data/output/GT_102.csv";
        QFileInfo check_file(path);
        if(check_file.exists() && check_file.isFile())
        {
            ReadCSV read_csv (path);
            map = read_csv.get_map();
            QString success = "Loaded the Map file successful and crate a map with ";
            success.append(QString::number(map.size()));
            success.append("entries.");

            PS_LOG->addInfoMessage(this, success);

        } else
        {
            qDebug() << "Could not load Map file, please rerun the step.";
            PS_LOG->addInfoMessage(this, tr("Could not load Map file, please rerun the step."));
        }
    }

    return map;
}



void ST_StepAbstractModellingMT::build_tree_model()
{

    HelperForMT mt(st_mt_vec,this);
    mt.start_mt();
    size_t size = st_mt_vec.size();


}


void ST_StepAbstractModellingMT::enrich_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, CT_ResultGroup* resCpy_res, CT_StandardItemGroup* grpCpy_grp)
{

    const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();

    size_t size = index->size();
    CT_NormalCloudStdVector *normalCloud = new CT_NormalCloudStdVector( size );
    CT_StandardCloudStdVectorT<float> *curvaturesCloud = new CT_StandardCloudStdVectorT<float>(size);
    CT_StandardCloudStdVectorT<float> *eigen1Cloud = new CT_StandardCloudStdVectorT<float>(size);
    CT_StandardCloudStdVectorT<float> *eigen2Cloud = new CT_StandardCloudStdVectorT<float>(size);
    CT_StandardCloudStdVectorT<float> *eigen3Cloud = new CT_StandardCloudStdVectorT<float>(size);
    CT_StandardCloudStdVectorT<float> *stemCloud = new CT_StandardCloudStdVectorT<float>(size);

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


    grpCpy_grp->addItemDrawable(normals);
    grpCpy_grp->addItemDrawable(curvatures);
    grpCpy_grp->addItemDrawable(eigenvalues1);
    grpCpy_grp->addItemDrawable(eigenvalues2);
    grpCpy_grp->addItemDrawable(eigenvalues3);
    grpCpy_grp->addItemDrawable(stem);
}

void ST_StepAbstractModellingMT::create_simple_tree_cloud(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in)
{
    if (itemCpy_cloud_in != NULL)
    {
        _number_trees++;
        const CT_AbstractPointCloudIndex* index =itemCpy_cloud_in->getPointCloudIndex();

        size_t size = 0;
        size = index->size();
        _cloud.reset(new PointCloudS);
        _cloud->width = (int) size;
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
        EnrichCloud enrich(_cloud, _knn, 0.03, true);
        StemPointDetection stempts (0,0.15,0.3,1,0,0.8,0.035,_cloud,1) ;
        stempts.compute();
        _percent_stem = stempts.get_percentage();


        PS_LOG->addInfoMessage(this, tr("Cloud computations successfull."));
    }

}



