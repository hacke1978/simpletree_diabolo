/****************************************************************************

 Copyright (C) 2016-2017 INRA (Institut National de la Recherche Agronomique, France) and IGN (Institut National de l'information Géographique et forestière, France)
 All rights reserved.

 Contact : jan.hackenberg@posteo.de

 Developers : Jan Hackenberg

 This file is part of Simpletree plugin Version 4 for Computree.

 Simpletree plugin is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Simpletree plugin is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Simpletree plugin.  If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/

#include "st_stepexportallEuroSDR.h"





ST_StepExportAllEuroSDR::ST_StepExportAllEuroSDR(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

ST_StepExportAllEuroSDR::~ST_StepExportAllEuroSDR()
{
}


// Step description (tooltip of contextual menu)
QString ST_StepExportAllEuroSDR::getStepDescription() const
{
    return tr("Exports EuroSDR benchmarking formated output.");
}

// Step detailled description
QString ST_StepExportAllEuroSDR::getStepDetailledDescription() const
{
    return tr("Output is generated according to EUROSDR standard format." );
}

// Step URL
QString ST_StepExportAllEuroSDR::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
}

// Step copy method
ST_StepExportAllEuroSDR* ST_StepExportAllEuroSDR::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepExportAllEuroSDR(dataInit);
}





void ST_StepExportAllEuroSDR::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addText("This step can create different output files. Please unselect if you want to print less information");
    configDialog->addEmpty();
    configDialog->addBool("Save one detailed CSV file per tree","","",_save_st_tree);
    //    configDialog->addBool("Save one Amap file per tree (not working at the moment)","","",_save_amap_tree);
    configDialog->addBool("Save the computed parameters in one file per pipeline","","",_save_coefficients);
    configDialog->addBool("Save the most important forestry parameters in one file per pipeline","","",_save_all);
    configDialog->addBool("Save one ply file per pipeline","","",_save_ply);
    configDialog->addFileChoice( tr("Select a folder to write the output files."), CT_FileChoiceButton::OneExistingFolder, "",
                                 _file_name_list);
    dialog_simple_tree(configDialog);
}


// Creation and affiliation of IN models
void ST_StepExportAllEuroSDR::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("tree_model_in"));
    resIn_res->setZeroOrMoreRootGroup();



    resIn_res->addGroupModel("",DEFin_grp_header, CT_AbstractItemGroup::staticGetType(), tr("header_and_dtm_group"));
    resIn_res->addItemModel(DEFin_grp_header, DEFin_header_in, CT_FileHeader::staticGetType(), tr("header"));
    resIn_res->addItemModel(DEFin_grp_header, DEFin_DTM_in, CT_Image2D<float>::staticGetType(), tr("dtm"));

    resIn_res->addGroupModel(DEFin_grp_header, DEFin_grp,  CT_AbstractItemGroup::staticGetType(), tr("tree_model_group"));
    resIn_res->addItemModel(DEFin_grp, DEFin_model_in, ST_Tree::staticGetType(), tr("tree"));
    resIn_res->addItemModel(DEFin_grp, DEFin_coeff_in, ST_Coefficients::staticGetType(), tr("coeff"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Tree cloud"));


}

// Creation and affiliation of OUT models
void ST_StepExportAllEuroSDR::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);
}

void ST_StepExportAllEuroSDR::compute()
{
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp_header(resCpy_res, this, DEFin_grp_header);
    QVector<MethodCoefficients> coeff_vec;
    QVector<QSharedPointer<Tree> > tree_vec;
    QVector<PointCloudS::Ptr> cloud_vec;
    QString path;


    CT_Image2D<float>* dtm = NULL;
    CT_FileHeader* header = NULL;
    while (itCpy_grp_header.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp_header.next();

        dtm = (CT_Image2D<float>*) grpCpy_grp->firstItemByINModelName(this, DEFin_DTM_in);
        header = (CT_FileHeader*)grpCpy_grp->firstItemByINModelName(this, DEFin_header_in);

        CT_GroupIterator itCpy_grp_tree(grpCpy_grp, this, DEFin_grp);
        while (itCpy_grp_tree.hasNext() && !isStopped())
        {
            CT_StandardItemGroup* grpCpy_grp2 = (CT_StandardItemGroup*) itCpy_grp_tree.next();
qDebug() << "a";
            ST_Tree* tree_st = (ST_Tree*) grpCpy_grp2->firstItemByINModelName(this, DEFin_model_in);
            ST_Coefficients * coeff_st = (ST_Coefficients*)grpCpy_grp2->firstItemByINModelName(this, DEFin_coeff_in);
            CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in
                    = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp2->firstItemByINModelName(this, DEFin_cloud_in);
            PointCloudS::Ptr cloud = pcl_CT_to_PCL_cloud(itemCpy_cloud_in,this,16, false,false);
qDebug() << "b";
if(coeff_st != NULL)
{
            MethodCoefficients cf = coeff_st->get_coeff();
            QSharedPointer<Tree> tree = tree_st->getTree();

qDebug() << "c";
            float dbh_taper = 200*Stem_Taper::get_DBH_from_taper(tree,cf);
            QSharedPointer<Cylinder> dbh_cyl = tree->get_cylinder_in_height(1.3f,cf);
            if(dbh_cyl != NULL)
            {
                if(dbh_cyl->get_radius()*200> 5&&dbh_taper>5)
                {
                    coeff_vec.push_back(cf);
                    tree_vec.push_back(tree);
                    cloud_vec.push_back(cloud);
                }
            }

qDebug() << "d";
}

        }


    }
    QString file_name = header->getFileName();
    QString plot_id;
    QString scan_design;
    split_file_name(file_name,plot_id, scan_design);
    if(!_file_name_list.empty())
    {
        path = _file_name_list.at(0);
    }
    QString plot_path;
    plot_path = path;
    plot_path.append("/TLS_benchmarking_2016_Plot_");
    plot_path.append(plot_id);
    plot_path.append("_LHD_");
    plot_path.append(scan_design);
    plot_path.append(".txt");

    QString stem_path;
    stem_path = path;
    stem_path.append("/TLS_benchmarking_2016_Plot_");
    stem_path.append(plot_id);
    stem_path.append("_Stem_Curve_");
    stem_path.append(scan_design);
    stem_path.append(".txt");

    QString dtm_path;
    dtm_path = path;
    dtm_path.append("/TLS_benchmarking_2016_Plot_");
    dtm_path.append(plot_id);
    dtm_path.append("_DTM_");
    dtm_path.append(scan_design);
    dtm_path.append(".txt");

    qDebug() << "á";
    write_dtm(dtm,dtm_path);
    qDebug() << "b";
    write_plot(dtm,cloud_vec,tree_vec,coeff_vec,plot_path);
    qDebug() << "c";
    write_taper(dtm,cloud_vec,tree_vec,coeff_vec,stem_path);
    qDebug() << "d";


}

void ST_StepExportAllEuroSDR::write_plot(CT_Image2D<float> *dtm, QVector<PointCloudS::Ptr> clouds, QVector<QSharedPointer<Tree> > qsms,
                                         QVector<MethodCoefficients> cfs, QString path)
{
    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    QFile file(path);
    if(file.open(QIODevice::WriteOnly))
    {

        QTextStream out(&file);
        for(int i = 0; i < clouds.size(); i++ )
        {
            PointCloudS::Ptr cloud = clouds.at(i);
            QSharedPointer<Tree> tree = qsms.at(i);
            MethodCoefficients cf = cfs.at(i);
            int id = i + 1;
            QSharedPointer<Cylinder> dbh_cyl = tree->get_cylinder_in_height(1.3f,cf);
            PointS center = dbh_cyl->get_center();
            float dbh = dbh_cyl->get_radius()*200;
            float x = center.x;
            float y = center.y;
            float zmin = std::numeric_limits<float>::max();
            float zmax = std::numeric_limits<float>::lowest();
            for(size_t j = 0; j < cloud->points.size(); j++)
            {
                PointS p = cloud->points.at(j);
                if(p.z < zmin)
                    zmin = p.z;
                if(p.z > zmax)
                    zmax = p.z;

            }
            float dif = zmax-zmin;
            float height = cf.cut_height + dif;
            QString str;
            str.append(QString::number(id));
            str.append(" ");
            str.append(QString::number(x, 'f', 3));
            str.append(" ");
            str.append(QString::number(y, 'f', 3));
            str.append(" ");
            str.append(QString::number(height, 'f', 3));
            str.append(" ");
            str.append(QString::number(dbh, 'f', 3));
            str.append("\n");
            out << str;

        }
         file.close();
    }

}



void ST_StepExportAllEuroSDR::write_taper(CT_Image2D<float> *dtm, QVector<PointCloudS::Ptr> clouds, QVector<QSharedPointer<Tree> > qsms,
                                         QVector<MethodCoefficients> cfs, QString path)
{
    float max_height = 2;
    for(int i = 0; i < clouds.size(); i++ )
    {
        QSharedPointer<Tree> tree = qsms.at(i);
        MethodCoefficients cf = cfs.at(i);
        QVector<QSharedPointer<Cylinder> > stem_cylinders = tree->get_stem_cylinders();
        QSharedPointer<Cylinder> first = stem_cylinders.at(0);
        QSharedPointer<Cylinder> last = stem_cylinders.last();
        float dif = last->get_end().z - first->get_start().z;
        float height = std::floor(dif+cf.cut_height);
        if(height > max_height)
            max_height = height;
    }
    QVector<float> heights;
    heights.push_back(0.65f);
    heights.push_back(1.3f);
    for(int i = 2; i < max_height; i++)
    {
        heights.push_back(i);
    }

    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    QFile file(path);
    if(file.open(QIODevice::WriteOnly))
    {

        QTextStream out(&file);
        for(int i = 0; i < clouds.size(); i++ )
        {
            QSharedPointer<Tree> tree = qsms.at(i);
            MethodCoefficients cf = cfs.at(i);
            int id = i + 1;
            QString d;
            QString x;
            QString y;
            QString z;
            d.append(QString::number(id));
            d.append(" ");

            x.append(QString::number(id));
            x.append(" ");

            y.append(QString::number(id));
            y.append(" ");

            z.append(QString::number(id));
            z.append(" ");

            for(int j = 0; j < heights.size(); j++)
            {
                float height = heights.at(j);
                QSharedPointer<Cylinder> cyl = tree->get_cylinder_in_height(height,cf);
                if(cyl == NULL)
                {
                    d.append(QString::number(-1));
                    d.append(" ");

                    x.append(QString::number(-1));
                    x.append(" ");

                    y.append(QString::number(-1));
                    y.append(" ");

                    z.append(QString::number(-1));
                    z.append(" ");
                }else {
                    PointS c = cyl->get_center();
                    float dia = cyl->get_radius()*200;
                    d.append(QString::number(dia));
                    d.append(" ");

                    x.append(QString::number(c.x, 'f', 3));
                    x.append(" ");

                    y.append(QString::number(c.y, 'f', 3));
                    y.append(" ");

                    z.append(QString::number(c.z, 'f', 3));
                    z.append(" ");

                }


            }
            d.append("\n");

            x.append("\n");

            y.append("\n");

            z.append("\n");

            out << d;
            out << x;
            out << y;
            out << z;

        }
         file.close();
    }

}

void ST_StepExportAllEuroSDR::write_dtm(CT_Image2D<float> *dtm, QString path, float res)
{
    int number_cells =  dtm->nCells();

    PointCloudS::Ptr cell_cloud (new PointCloudS);
    PointCloudS::Ptr cell_cloud_2D (new PointCloudS);

    for(int i = 0; i < number_cells; i++)
    {
        Eigen::Vector2d bot;
        Eigen::Vector2d top;
        dtm->getCellCoordinates(i,bot,top);
        float centerX = (bot(0)+top(0))/2;
        float centerY = (bot(1)+top(1))/2;
        float value = dtm->valueAtIndex(i);
        if(value != -9999)
        {
            PointS p;
            p.x = centerX;
            p.y = centerY;
            p.z = value;
            cell_cloud->points.push_back(p);
            PointS p2;
            p2.x = centerX;
            p2.y = centerY;
            p2.z = 0;
            cell_cloud_2D->points.push_back(p2);
        }
    }

    float min_x = dtm->minX();
    float max_x = dtm->maxX();
    float min_y = dtm->minY();
    float max_y = dtm->maxY();


    pcl::KdTreeFLANN<PointS> kdtree;
    kdtree.setInputCloud (cell_cloud_2D);

    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    QFile file(path);
    if(file.open(QIODevice::WriteOnly))
    {

        QTextStream out(&file);
        for(float x = min_x; x < max_x ; x+=res)
        {
            for(float y = min_y; y < max_y ; y +=res)
            {
                PointS p;
                p.x = x;
                p.y = y;
                p.z = 0;


                int K = 4;
                std::vector<int> pointIdxNKNSearch(K);
                std::vector<float> pointNKNSquaredDistance(K);
                float dist1;
                float dist2;
                float dist3;
                float dist4;
                int index1;
                int index2;
                int index3;
                int index4;

                if ( kdtree.nearestKSearch (p, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
                {
                    dist1 = std::sqrt(pointNKNSquaredDistance[0]);
                    dist2 = std::sqrt(pointNKNSquaredDistance[1]);
                    dist3 = std::sqrt(pointNKNSquaredDistance[2]);
                    dist4 = std::sqrt(pointNKNSquaredDistance[3]);

                    if(dist1 == 0)
                    {
                        dist1 = 1000000000;
                    } else {
                        dist1 = 1/dist1;
                    }

                    if(dist2 == 0)
                    {
                        dist2 = 1000000000;
                    } else {
                        dist2 = 1/dist2;
                    }

                    if(dist3 == 0)
                    {
                        dist3 = 1000000000;
                    } else {
                        dist3 = 1/dist3;
                    }

                    if(dist4 == 0)
                    {
                        dist4 = 1000000000;
                    } else {
                        dist4 = 1/dist4;
                    }

                    index1 = pointIdxNKNSearch[0];
                    index2 = pointIdxNKNSearch[1];
                    index3 = pointIdxNKNSearch[2];
                    index4 = pointIdxNKNSearch[3];
                }
                float dist_total = dist1 + dist2 + dist3 + dist4;
                float val = cell_cloud->points[index1].z * dist1/dist_total +
                        cell_cloud->points[index2].z * dist2/dist_total +
                        cell_cloud->points[index3].z * dist3/dist_total +
                        cell_cloud->points[index4].z * dist4/dist_total;

                QString str;
                str.append(QString::number(x, 'f', 3));
                str.append(" ");
                str.append(QString::number(y, 'f', 3));
                str.append(" ");
                str.append(QString::number(val, 'f', 3));
                str.append("\n");
                out << str;
            }
        }
        file.close();
    }
}

void ST_StepExportAllEuroSDR::split_file_name(QString &file_name, QString &plot_id, QString &scan_design)
{
    QStringList list_all = file_name.split(".");
    if(list_all.size()<=0)
        PS_LOG->addInfoMessage(this, "no file extension included or too many . used");
    QString file_no_ext = list_all[0];
    QStringList list_no_ext = file_no_ext.split("_");
    if(list_no_ext.size()!=2)
        PS_LOG->addInfoMessage(this, "not the right EUROSDR file naming convention");
    plot_id = list_no_ext[0];
    scan_design = list_no_ext[1];

}
