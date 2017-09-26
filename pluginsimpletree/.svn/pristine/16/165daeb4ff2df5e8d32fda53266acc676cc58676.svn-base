#include "st_stepcomputecrown.h"

ST_StepComputeCrown::ST_StepComputeCrown(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

ST_StepComputeCrown::~ST_StepComputeCrown()
{
}


// Step description (tooltip of contextual menu)
QString ST_StepComputeCrown::getStepDescription() const
{
    return tr("Calculates a crown model.");
}

// Step detailled description
QString ST_StepComputeCrown::getStepDetailledDescription() const
{
    return tr("Calculates a 3d and 2d crown model" );
}

// Step URL
QString ST_StepComputeCrown::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
}

// Step copy method
ST_StepComputeCrown* ST_StepComputeCrown::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepComputeCrown(dataInit);
}

QString ST_StepComputeCrown::getStepName() const
{
    return "ST_Compute_Crown";
}






void ST_StepComputeCrown::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addText("Dummy Step, had to be deactivated for now.");
}


// Creation and affiliation of IN models
void ST_StepComputeCrown::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("QSM result model"));
    resIn_res->setZeroOrMoreRootGroup();

    resIn_res->addGroupModel("", DEFin_grp,  CT_AbstractItemGroup::staticGetType(), tr("QSM group"));
    resIn_res->addItemModel(DEFin_grp, DEFin_model_in, ST_Tree::staticGetType(), tr("QSM"));
    resIn_res->addItemModel(DEFin_grp, DEFin_coeff, ST_Coefficients::staticGetType(), tr("Method coefficients"));
    resIn_res->addItemModel(DEFin_grp, DEFin_header, CT_FileHeader::staticGetType(), tr("File Header"));

}

// Creation and affiliation of OUT models
void ST_StepComputeCrown::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);
    resCpy_res->addItemModel(DEFin_grp,_tree_out, new ST_Tree(), tr("Tree with crown"));
    resCpy_res->addItemModel(DEFin_grp,_coeff_out, new ST_Coefficients(), tr("Tree coefficients with crown parameters"));
    resCpy_res->addItemModel(DEFin_grp,_mesh_3d, new CT_MeshModel(), tr("Crown hull"));
    resCpy_res->addItemModel(DEFin_grp,_mesh_2d, new CT_MeshModel(), tr("Crown projection area"));
    //   resCpy_res->addItemModel(DEFin_grp, _crown_out, new ST_Crown(), tr("crown"));
}

void ST_StepComputeCrown::compute()
{
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    QString path;

    if(!_file_name_list.empty())
    {
        path = _file_name_list.at(0);
    }

    while (itCpy_grp.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();


        CT_FileHeader * itemCpy_header =
                (CT_FileHeader*)grpCpy_grp->firstItemByINModelName(this, DEFin_header);

        ST_Tree * tree_model = (ST_Tree*) grpCpy_grp->firstItemByINModelName(this,DEFin_model_in);

        ST_Coefficients * tree_coeff = (ST_Coefficients*) grpCpy_grp->firstItemByINModelName(this, DEFin_coeff);

        MethodCoefficients coeff = tree_coeff->get_coeff();

        QSharedPointer<Tree> tree = tree_model->getTree();

        QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();

        pcl::PointCloud<pcl::PointXYZ>::Ptr end_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        {
            QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
            while(it.hasNext())
            {
                QSharedPointer<Cylinder> cylinder = it.next();
                PointS ps = cylinder->get_end();
                pcl::PointXYZ p (ps.x,ps.y,ps.z);

                end_point_cloud->push_back(p);
            }

            pcl::PolygonMesh mesh;

            pcl::ConvexHull<pcl::PointXYZ> hull;
            hull.setInputCloud (end_point_cloud);
            hull.setComputeAreaVolume (true);
            hull.reconstruct (mesh);

            QString desktop_folder = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
            desktop_folder.append("/convex.obj");
            std::string desktop_folder_str= desktop_folder.toStdString();
            desktop_folder_str.append("/convex.obj");
            CT_Reader_OBJ m_reader{};
            m_reader.setLoadAsPointCloud(false);
            m_reader.setFilePath(desktop_folder);
            pcl::io::saveOBJFile ( desktop_folder_str, mesh);


            if(((m_reader.readHeader()) != NULL) && m_reader.readFile())
            {
                CT_AbstractSingularItemDrawable* item = m_reader.takeFirstItemDrawableOfModel(DEF_CT_Reader_OBJ_meshOrSceneOut, resCpy_res, _mesh_3d.completeName());
                grpCpy_grp->addItemDrawable(item);
            }

            float volume = hull.getTotalVolume ();
            float area = hull.getTotalArea ();
            coeff.area_3d = area;
            coeff.volume_3d = volume;

            QString _str = "";
            _str.append("The volume of the crown is ").append(
                        QString::number(volume)).append(" in m^3, the crown surface area is ").append(
                        QString::number(area)).append("m^2.\n");
            PS_LOG->addInfoMessage(this, _str);
        }

        {
            QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
            while(it.hasNext())
            {
                QSharedPointer<Cylinder> cylinder = it.next();
                PointS ps = cylinder->get_end();
                pcl::PointXYZ p (ps.x,ps.y,0);

                end_point_cloud->push_back(p);
            }

            pcl::PolygonMesh mesh;
            pcl::ConvexHull<pcl::PointXYZ> hull;
            hull.setInputCloud (end_point_cloud);
            hull.setComputeAreaVolume (true);
            hull.reconstruct (mesh);

            QString desktop_folder = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
            desktop_folder.append("/convex.obj");
            std::string desktop_folder_str= desktop_folder.toStdString();
            desktop_folder_str.append("/convex.obj");
            CT_Reader_OBJ m_reader{};
            m_reader.setLoadAsPointCloud(false);
            m_reader.setFilePath(desktop_folder);
            pcl::io::saveOBJFile ( desktop_folder_str, mesh);


            if(((m_reader.readHeader()) != NULL) && m_reader.readFile())
            {
                CT_AbstractSingularItemDrawable* item = m_reader.takeFirstItemDrawableOfModel(DEF_CT_Reader_OBJ_meshOrSceneOut, resCpy_res, _mesh_2d.completeName());
                grpCpy_grp->addItemDrawable(item );
            }
;
            float area = hull.getTotalArea ();
            coeff.area_2d = area;
            QString _str = "";
            _str.append("The crown projection area of the crown is ").append(
                        QString::number(area)).append(" in m^3.");
            PS_LOG->addInfoMessage(this, _str);
        }

        ST_Tree * st_tree = new ST_Tree(_tree_out.completeName(),resCpy_res,tree->clone());
        grpCpy_grp->addItemDrawable(st_tree);
        ST_Coefficients *  st_coefficients = new ST_Coefficients(_coeff_out.completeName(), resCpy_res,coeff);
        grpCpy_grp->addItemDrawable(st_coefficients);




    }
}
