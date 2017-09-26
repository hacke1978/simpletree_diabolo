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

#include "export.h"

Export::Export()
{

}

void Export::export_tree_detail(QSharedPointer<Tree> tree, QString path, QString file_name, MethodCoefficients coeff)
{
    if(tree!=NULL)
    {
        QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
        QStringList file_list = file_name.split(".");

        QString file_id = file_list.at(0);
        file_id.append("_detailed.csv");
        path.append("/detailed/");
        QDir dir(path);
        if(!dir.exists())
        {
            dir.mkpath(".");
        }
        path.append(file_id);
        QFile file(path);


        if(file.open(QIODevice::WriteOnly))
        {
            QTextStream out(&file);
            QString  total_id = tree->getTreeID();
            float total_vol = tree->get_volume();
            float total_len = tree->get_length(coeff);
            float total_height = tree->get_height(coeff);

            QVector<QSharedPointer<Cylinder> > cylinders2 = tree->get_stem_cylinders();
            QSharedPointer<Cylinder> dbh_cylinder = cylinders2.at(0);
            QSharedPointer<Cylinder> dbh_cylinder2 = cylinders2.at(0);
            float cut_height = coeff.cut_height;
            float dbh_z = dbh_cylinder->get_start().z + (1.3f - cut_height);
            {
                QVectorIterator<QSharedPointer<Cylinder> > git(cylinders2);
                while(git.hasNext())
                {
                    QSharedPointer<Cylinder>  cyl = git.next();
                    float start_z = cyl->get_start().z;
                    float end_z = cyl->get_end().z;
                    if(start_z <= dbh_z && end_z >=dbh_z)
                    {
                        dbh_cylinder = cyl;
                    }
                }
            }
            //            float dbh = 0;
            //if(dbh_cylinder!=dbh_cylinder2)
            //                dbh = dbh_cylinder->get_radius()*200;
            float total_dbh =  dbh_cylinder->get_radius()*200;
            float total_dbh_taper = 200*Stem_Taper::get_DBH_from_taper(tree,coeff);
            float total_cut_height = (coeff.cut_height);
            QSharedPointer<Cylinder> root = tree->get_all_cylinders().at(0);
            float total_x = root->get_start().x;
            float total_y = root->get_start().y;





            out << "branch_ID;branch_order;segment_ID;parent_segment_ID;growth_volume;growth_length;detection;improvement;startX;startY;startZ;endX;endY;endZ;"
                   "radius;length;species;ID;length_to_leave; inverse_branch_order; length_of_segment;branch_order_cum; cylinder_ID; cylinder_parent_ID; reverse_pipe_order;Allometric_improved;"
                   "length_to_root;tree_ID; volume (m^3); length (m); height (m); DBH (cm); DBH_taper_linear (cm); cut_height (m);x_position;y_position; "
                   "cylinder_children_id;segment_children_id\n";
            QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
            QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
            while(it.hasNext())
            {
                QSharedPointer<Cylinder>  cyl = it.next();
                QString str;
                str.append(QString::number(cyl->get_segment()->get_branch_id())).append(";");
                str.append(QString::number(cyl->get_segment()->get_branch_order())).append(";");
                str.append(QString::number(cyl->get_segment()->get_id())).append(";");

                if(cyl->get_segment()->get_parent_segment())
                {
                    str.append(QString::number(cyl->get_segment()->get_parent_segment()->get_id())).append(";");
                }
                else
                {
                    str.append("-1;");
                }

                str.append(QString::number(tree->get_growth_volume(cyl))).append(";");
                str.append(QString::number(tree->get_growth_length(cyl))).append(";");

                {
                    int detection  = cyl->get_detection();
                    switch (detection) {
                    case 0:
                        str.append("spherefollowing");
                        break;
                    case 1:
                        str.append("attractor");
                        break;
                    case 2:
                        str.append("taper");
                        break;
                    default:
                        str.append("spherefollowing");
                        break;
                    }
                    str.append(";");
                }

                {
                    int improvement  = cyl->get_improvement();
                    switch (improvement) {
                    case 0:
                        str.append("RANSAC");
                        break;
                    case 1:
                        str.append("MEDIAN");
                        break;
                    case 2:
                        str.append("NO");
                        break;
                    default:
                        str.append("NO");
                        break;
                    }
                    str.append(";");
                }

                str.append(QString::number(cyl->get_start().x)).append(";");
                str.append(QString::number(cyl->get_start().y)).append(";");
                str.append(QString::number(cyl->get_start().z)).append(";");
                str.append(QString::number(cyl->get_end().x)).append(";");
                str.append(QString::number(cyl->get_end().y)).append(";");
                str.append(QString::number(cyl->get_end().z)).append(";");
                str.append(QString::number(cyl->get_radius())).append(";");
                str.append(QString::number(cyl->get_length())).append(";");
                str.append(coeff.species).append(";");


                str.append(coeff.id).append(";");

                str.append(QString::number(tree->get_length_to_leave(cyl))).append(";");
                str.append(QString::number(tree->get_branch_order_from_leave(cyl))).append(";");

                str.append(QString::number(cyl->get_segment()->get_length())).append(";");
                str.append(QString::number(tree->get_branch_order_cumulative(cyl->get_segment()))).append(";");
                str.append(QString::number(cyl->getID())).append(";");
                if(cyl==tree->get_parent(cyl))
                {
                    str.append("-1;");
                } else {
                    str.append(QString::number(tree->get_parent(cyl)->getID())).append(";");
                }
                str.append(QString::number(cyl->get_segment()->getReverse_pipe_order())).append(";");
                {
                    int improvement  = cyl->get_allometry_improvement();
                    switch (improvement) {
                    case 0:
                        str.append("NO_ALLOM");
                        break;
                    case 1:
                        str.append("ALLOM_VOL");
                        break;
                    case 2:
                        str.append("ALLOM_LEN");
                        break;
                    case 3:
                        str.append("ALLOM_TAPER");
                        break;
                    case 4:
                        str.append("ALLOM_PIPE");
                        break;
                    case 5:
                        str.append("ALLOM_MEDIAN_INSIDE_SEGMENT");
                        break;
                    case 6:
                        str.append("ALLOM_PIPE_MEDIAN_OVER_SEGMENT");
                        break;
                    default:
                        str.append("ALLOM_UNNOWN");
                        break;
                    }
                    str.append(";");
                }
                str.append(QString::number(tree->get_length_to_root(cyl,coeff))).append(";");

                str.append(total_id).append(";");
                str.append(QString::number(total_vol)).append(";");
                str.append(QString::number(total_len)).append(";");
                str.append(QString::number(total_height)).append(";");
                str.append(QString::number(total_dbh)).append(";");
                str.append(QString::number(total_dbh_taper)).append(";");
                str.append(QString::number(total_cut_height)).append(";");
                str.append(QString::number(total_x)).append(";");
                str.append(QString::number(total_y)).append(";");
                QVector<QSharedPointer<Cylinder> > children = tree->get_children_non_recursive(cyl);
                for(size_t i = 0; i < children.size(); i++)
                {
                    QSharedPointer<Cylinder> child = children.at(i);
                    str.append(QString::number(child->getID())).append("_");
                }
                str.append(";");
                QVector<QSharedPointer<Segment> > children2 = cyl->get_segment()->get_child_segments();
                for(size_t i = 0; i < children2.size(); i++)
                {
                    QSharedPointer<Segment> child = children2.at(i);
                    str.append(QString::number(child->get_id())).append("_");
                }

                str.append("\n");

                out << str;


            }
            file.close();
        }
    }
}

void Export::export_coefficients(QString path, QVector<MethodCoefficients> coeff_list)
{
    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    path.append("/result_coeff.csv");
    QFile file(path);

    if(file.open(QIODevice::WriteOnly))
    {
        QTextStream out(&file);

        out << "min_fitted_distance; sphere_radius_multiplier; epsilon_cluster_stem; epsilon_cluster_branch; epsilon_sphere;"
               " minPts_improve_cylinder; minPts_ransac_branch; minPts_cluster_stem; minPts_cluster_branch; min_radius_sphere;"
               " height_start_sphere; ransac_circle_type; ransac_circle_inlier_distance; cut_height; number_clusters_for_spherefollowing;"
               " use_dhs; clustering_distance; tree_height; tree_circumference; tree_predicted_volume; tree_max_angle; percentage_for_attractor;"
               " use_allom; a; b; a_length; b_length; minRad; ransac_type; ransac_inlier_distance; ransac_iterations; ransac_median_factor; min_dist;"
               " factor; sd; mean; sd_mult; optimze_stem;"
               " radius_multiplier_cylinder_test; times_cluster_extension; max_times_cluster_extension; min_ratio_pype; max_ratio_pype;"
               " min_radius_for_pype_test; max_number_failure_segments; id; species; outputpath; number_merges\n";
        QVectorIterator<MethodCoefficients> it(coeff_list);
        while(it.hasNext())
        {
            MethodCoefficients  coefficients = it.next();
            QString str;

            str.append(QString::number(coefficients.min_fitted_distance));
            str.append(";");
            str.append(QString::number(coefficients.sphere_radius_multiplier));
            str.append(";");
            str.append(QString::number(coefficients.epsilon_cluster_stem));
            str.append(";");
            str.append(QString::number(coefficients.epsilon_cluster_branch));
            str.append(";");
            str.append(QString::number(coefficients.epsilon_sphere));
            str.append(";");

            str.append(QString::number(coefficients.minPts_improve_cylinder));
            str.append(";");
            str.append(QString::number(coefficients.minPts_ransac_branch));
            str.append(";");
            str.append(QString::number(coefficients.minPts_cluster_stem));
            str.append(";");
            str.append(QString::number(coefficients.minPts_cluster_branch));
            str.append(";");
            str.append(QString::number(coefficients.min_radius_sphere));
            str.append(";");

            str.append(QString::number(coefficients.height_start_sphere));
            str.append(";");
            str.append(QString::number(coefficients.ransac_circle_type));
            str.append(";");
            str.append(QString::number(coefficients.ransac_circle_inlier_distance));
            str.append(";");
            str.append(QString::number(coefficients.cut_height));
            str.append(";");
            str.append(QString::number(coefficients.number_clusters_for_spherefollowing));
            str.append(";");

            str.append(QString::number(coefficients.use_dhs));
            str.append(";");
            str.append(QString::number(coefficients.clustering_distance));
            str.append(";");
            str.append(QString::number(coefficients.tree_height));
            str.append(";");
            str.append(QString::number(coefficients.tree_circumference));
            str.append(";");
            str.append(QString::number(coefficients.tree_predicted_volume));
            str.append(";");
            str.append(QString::number(coefficients.tree_max_angle));
            str.append(";");
            str.append(QString::number(coefficients.percentage_for_attractor));
            str.append(";");

            str.append(QString::number(coefficients.use_allom));
            str.append(";");
            str.append(QString::number(coefficients.a));
            str.append(";");
            str.append(QString::number(coefficients.b));
            str.append(";");
            str.append(QString::number(coefficients.length_a));
            str.append(";");
            str.append(QString::number(coefficients.length_b));
            str.append(";");
            str.append(QString::number(coefficients.minRad));
            str.append(";");
            str.append(QString::number(coefficients.ransac_type));
            str.append(";");
            str.append(QString::number(coefficients.ransac_inlier_distance));
            str.append(";");
            str.append(QString::number(coefficients.ransac_iterations));
            str.append(";");
            str.append(QString::number(coefficients.ransac_median_factor));
            str.append(";");
            str.append(QString::number(coefficients.min_dist));
            str.append(";");

            str.append(QString::number(coefficients.factor));
            str.append(";");
            str.append(QString::number(coefficients.sd));
            str.append(";");
            str.append(QString::number(coefficients.mean));
            str.append(";");
            str.append(QString::number(coefficients.sd_mult));
            str.append(";");
            str.append(QString::number(coefficients.optimze_stem));
            str.append(";");

            str.append(QString::number(coefficients.radius_multiplier_cylinder_test));
            str.append(";");
            str.append(QString::number(coefficients.times_cluster_extension));
            str.append(";");
            str.append(QString::number(coefficients.max_times_cluster_extension));
            str.append(";");
            str.append(QString::number(coefficients.min_ratio_pype));
            str.append(";");
            str.append(QString::number(coefficients.max_ratio_pype));
            str.append(";");

            str.append(QString::number(coefficients.min_radius_for_pype_test));
            str.append(";");
            str.append(QString::number(coefficients.max_number_failure_segments));
            str.append(";");
            str.append(coefficients.id);
            str.append(";");
            str.append(coefficients.species);
            str.append(";");
            str.append(coefficients.outputpath);
            str.append(";");
            str.append(QString::number(coefficients.number_of_merges));
            str.append("\n");

            out << str;


        }

        file.close();

    }
}

void Export::export_ply(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution)
{
    if(tree!=NULL)
    {
        QString path2 = path;
        path2.append("/ply/");
        QDir dir(path2);
        if(!dir.exists())
        {
            dir.mkpath(".");
        }
        export_ply_color(path, file_name, tree, resolution);
        export_ply_good(path, file_name, tree, resolution);
        export_ply_bad(path, file_name, tree, resolution);
    }
}

void Export::export_ply_stem(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution)
{
    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    QStringList file_list = file_name.split(".");

    QString file_id = file_list.at(0);
    file_id.append(".ply");
    path.append("/ply/stem/");
    QDir dir(path);
    if(!dir.exists())
    {
        dir.mkpath(".");
    }
    path.append(file_id);


    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_stem_cylinders_save();
    int number_cylinders = cylinders.size();
    int number_points =  number_cylinders * resolution * 2;
    int number_faces  =  number_cylinders * resolution * 2;




    QFile file(path);

    if(file.open(QIODevice::WriteOnly))
    {
        QTextStream out(&file);


        out << "ply\n"
               "format ascii 1.0\n"
               "comment author: Jan Hackenberg\n"
               "comment object: SimpleTree calculated tree model\n";
        out << "element vertex ";
        out << QString::number(number_points).append("\n");
        out << "property float x\n"
               "property float y\n"
               "property float z\n"
               "property uchar red\n"
               "property uchar green\n"
               "property uchar blue\n";
        out << "element face ";
        out << QString::number(number_faces).append("\n");
        out << "property list uchar int vertex_index\n"
               "end_header\n";
        QVector<QSharedPointer<Cylinder> > cylinders = tree->get_stem_cylinders_save();
        PointCloudS::Ptr cloud = generate_points_from_cylinder_list(cylinders,resolution);
        for(int i = 0; i < cloud->points.size(); i++)
        {
            PointS p = cloud->points.at(i);
            out << QString::number(p.x);
            out << " ";
            out << QString::number(p.y);
            out << " ";
            out << QString::number(p.z);
            out << " ";
            if(p.is_stem == 0)
            {
                out << "255 0 0\n";
            } else
            {
                out << "0 255 0\n";
            }
        }
        for(int i = 0; i < cylinders.size(); i++)
        {
            int x = 2*i;
            for(int j = 0; j < resolution -1; j++)
            {
                out << "3 ";
                out << QString::number(x*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j+1);
                out << "\n";


                out << "3 ";
                out << QString::number((x+1)*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j);
                out << "\n";

            }

            out << "3 ";
            out << QString::number(x*resolution+resolution -1);
            out << " ";
            out << QString::number(x*resolution);
            out << " ";
            out << QString::number((x+1)*resolution);
            out << "\n";

            out << "3 ";
            out << QString::number((x+1)*resolution);
            out << " ";
            out << QString::number((x+1)*resolution + resolution -1);
            out << " ";
            out << QString::number(x*resolution+resolution -1);
            out << "\n";
        }


        file.close();

    }
}


void Export::export_ply_stem_all(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution)
{
    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    QStringList file_list = file_name.split(".");

    QString file_id = file_list.at(0);
    file_id.append(".ply");
    path.append("/ply/stem_total/");
    QDir dir(path);
    if(!dir.exists())
    {
        dir.mkpath(".");
    }
    path.append(file_id);


    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_stem_cylinders();
    int number_cylinders = cylinders.size();
    int number_points =  number_cylinders * resolution * 2;
    int number_faces  =  number_cylinders * resolution * 2;




    QFile file(path);

    if(file.open(QIODevice::WriteOnly))
    {
        QTextStream out(&file);


        out << "ply\n"
               "format ascii 1.0\n"
               "comment author: Jan Hackenberg\n"
               "comment object: SimpleTree calculated tree model\n";
        out << "element vertex ";
        out << QString::number(number_points).append("\n");
        out << "property float x\n"
               "property float y\n"
               "property float z\n"
               "property uchar red\n"
               "property uchar green\n"
               "property uchar blue\n";
        out << "element face ";
        out << QString::number(number_faces).append("\n");
        out << "property list uchar int vertex_index\n"
               "end_header\n";
        QVector<QSharedPointer<Cylinder> > cylinders = tree->get_stem_cylinders_save();
        PointCloudS::Ptr cloud = generate_points_from_cylinder_list(cylinders,resolution);
        for(int i = 0; i < cloud->points.size(); i++)
        {
            PointS p = cloud->points.at(i);
            out << QString::number(p.x);
            out << " ";
            out << QString::number(p.y);
            out << " ";
            out << QString::number(p.z);
            out << " ";
            if(p.is_stem == 0)
            {
                out << "255 0 0\n";
            } else
            {
                out << "0 255 0\n";
            }
        }
        for(int i = 0; i < cylinders.size(); i++)
        {
            int x = 2*i;
            for(int j = 0; j < resolution -1; j++)
            {
                out << "3 ";
                out << QString::number(x*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j+1);
                out << "\n";


                out << "3 ";
                out << QString::number((x+1)*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j);
                out << "\n";

            }

            out << "3 ";
            out << QString::number(x*resolution+resolution -1);
            out << " ";
            out << QString::number(x*resolution);
            out << " ";
            out << QString::number((x+1)*resolution);
            out << "\n";

            out << "3 ";
            out << QString::number((x+1)*resolution);
            out << " ";
            out << QString::number((x+1)*resolution + resolution -1);
            out << " ";
            out << QString::number(x*resolution+resolution -1);
            out << "\n";
        }


        file.close();

    }
}

void Export::export_ply_allom(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution)
{
    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    QStringList file_list = file_name.split(".");

    QString file_id = file_list.at(0);
    file_id.append(".ply");
    path.append("/ply/allom/");
    QDir dir(path);
    if(!dir.exists())
    {
        dir.mkpath(".");
    }
    path.append(file_id);


    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
    int number_cylinders = cylinders.size();
    int number_points =  number_cylinders * resolution * 2;
    int number_faces  =  number_cylinders * resolution * 2;




    QFile file(path);

    if(file.open(QIODevice::WriteOnly))
    {
        QTextStream out(&file);


        out << "ply\n"
               "format ascii 1.0\n"
               "comment author: Jan Hackenberg\n"
               "comment object: SimpleTree calculated tree model\n";
        out << "element vertex ";
        out << QString::number(number_points).append("\n");
        out << "property float x\n"
               "property float y\n"
               "property float z\n"
               "property uchar red\n"
               "property uchar green\n"
               "property uchar blue\n";
        out << "element face ";
        out << QString::number(number_faces).append("\n");
        out << "property list uchar int vertex_index\n"
               "end_header\n";
        QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
        PointCloudS::Ptr cloud = generate_points_from_cylinder_list(cylinders,resolution);
        for(int i = 0; i < cloud->points.size(); i++)
        {
            PointS p = cloud->points.at(i);
            out << QString::number(p.x);
            out << " ";
            out << QString::number(p.y);
            out << " ";
            out << QString::number(p.z);
            out << " ";
            int index_cylinder = std::floor(i/(2.0f*resolution));
            QSharedPointer<Cylinder> cylinder = cylinders.at(index_cylinder);
            if(cylinder->get_allometry_improvement() == AllometryImproved::NOALLOM)
            {
                out << "255 0 0\n";
            } else
            {
                out << "0 255 0\n";
            }
        }
        for(int i = 0; i < cylinders.size(); i++)
        {
            int x = 2*i;
            for(int j = 0; j < resolution -1; j++)
            {
                out << "3 ";
                out << QString::number(x*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j+1);
                out << "\n";


                out << "3 ";
                out << QString::number((x+1)*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j);
                out << "\n";

            }

            out << "3 ";
            out << QString::number(x*resolution+resolution -1);
            out << " ";
            out << QString::number(x*resolution);
            out << " ";
            out << QString::number((x+1)*resolution);
            out << "\n";

            out << "3 ";
            out << QString::number((x+1)*resolution);
            out << " ";
            out << QString::number((x+1)*resolution + resolution -1);
            out << " ";
            out << QString::number(x*resolution+resolution -1);
            out << "\n";
        }


        file.close();

    }
}

void Export::export_ply_detection(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution)
{
    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    QStringList file_list = file_name.split(".");

    QString file_id = file_list.at(0);
    file_id.append(".ply");
    path.append("/ply/detection/");
    QDir dir(path);
    if(!dir.exists())
    {
        dir.mkpath(".");
    }
    path.append(file_id);


    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
    int number_cylinders = cylinders.size();
    int number_points =  number_cylinders * resolution * 2;
    int number_faces  =  number_cylinders * resolution * 2;




    QFile file(path);

    if(file.open(QIODevice::WriteOnly))
    {
        QTextStream out(&file);


        out << "ply\n"
               "format ascii 1.0\n"
               "comment author: Jan Hackenberg\n"
               "comment object: SimpleTree calculated tree model\n";
        out << "element vertex ";
        out << QString::number(number_points).append("\n");
        out << "property float x\n"
               "property float y\n"
               "property float z\n"
               "property uchar red\n"
               "property uchar green\n"
               "property uchar blue\n";
        out << "element face ";
        out << QString::number(number_faces).append("\n");
        out << "property list uchar int vertex_index\n"
               "end_header\n";
        QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
        PointCloudS::Ptr cloud = generate_points_from_cylinder_list(cylinders,resolution);
        for(int i = 0; i < cloud->points.size(); i++)
        {
            PointS p = cloud->points.at(i);
            out << QString::number(p.x);
            out << " ";
            out << QString::number(p.y);
            out << " ";
            out << QString::number(p.z);
            out << " ";
            int index_cylinder = std::floor(i/(2.0f*resolution));
            QSharedPointer<Cylinder> cylinder = cylinders.at(index_cylinder);
            if(cylinder->get_detection() == DetectionType::SPHEREFOLLOWING)
            {
                out << "255 0 0\n";
            } else
            {
                out << "0 255 0\n";
            }
        }
        for(int i = 0; i < cylinders.size(); i++)
        {
            int x = 2*i;
            for(int j = 0; j < resolution -1; j++)
            {
                out << "3 ";
                out << QString::number(x*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j+1);
                out << "\n";


                out << "3 ";
                out << QString::number((x+1)*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j);
                out << "\n";

            }

            out << "3 ";
            out << QString::number(x*resolution+resolution -1);
            out << " ";
            out << QString::number(x*resolution);
            out << " ";
            out << QString::number((x+1)*resolution);
            out << "\n";

            out << "3 ";
            out << QString::number((x+1)*resolution);
            out << " ";
            out << QString::number((x+1)*resolution + resolution -1);
            out << " ";
            out << QString::number(x*resolution+resolution -1);
            out << "\n";
        }


        file.close();

    }
}

void Export::export_ply_good(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution)
{
    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    QStringList file_list = file_name.split(".");

    QString file_id = file_list.at(0);
    file_id.append(".ply");
    path.append("/ply/good/");
    QDir dir(path);
    if(!dir.exists())
    {
        dir.mkpath(".");
    }
    path.append(file_id);


    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();

    QVector<QSharedPointer<Cylinder> > good_cylinders;
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cyl = it.next();

        if( (cyl->get_detection()== DetectionType::SPHEREFOLLOWING) && (cyl->get_allometry_improvement() == AllometryImproved::NOALLOM ))
        {
            good_cylinders.push_back(cyl);
        }
    }

    cylinders = good_cylinders;
    int number_cylinders = cylinders.size();
    int number_points =  number_cylinders * resolution * 2;
    int number_faces  =  number_cylinders * resolution * 2;




    QFile file(path);

    if(file.open(QIODevice::WriteOnly))
    {
        QTextStream out(&file);


        out << "ply\n"
               "format ascii 1.0\n"
               "comment author: Jan Hackenberg\n"
               "comment object: SimpleTree calculated tree model\n";
        out << "element vertex ";
        out << QString::number(number_points).append("\n");
        out << "property float x\n"
               "property float y\n"
               "property float z\n"
               "property uchar red\n"
               "property uchar green\n"
               "property uchar blue\n";
        out << "element face ";
        out << QString::number(number_faces).append("\n");
        out << "property list uchar int vertex_index\n"
               "end_header\n";


        PointCloudS::Ptr cloud = generate_points_from_cylinder_list(cylinders,resolution);
        for(int i = 0; i < cloud->points.size(); i++)
        {
            PointS p = cloud->points.at(i);
            out << QString::number(p.x);
            out << " ";
            out << QString::number(p.y);
            out << " ";
            out << QString::number(p.z);
            out << " ";
            int index_cylinder = std::floor(i/(2.0f*resolution));
            QSharedPointer<Cylinder> cylinder = cylinders.at(index_cylinder);
            if(cylinder->get_improvement() == ImprovementType::RANSAC)
            {
                out << "255 0 0\n";
            } else
            {
                out << "0 255 0\n";
            }
        }
        for(int i = 0; i < cylinders.size(); i++)
        {
            int x = 2*i;
            for(int j = 0; j < resolution -1; j++)
            {
                out << "3 ";
                out << QString::number(x*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j+1);
                out << "\n";


                out << "3 ";
                out << QString::number((x+1)*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j);
                out << "\n";

            }

            out << "3 ";
            out << QString::number(x*resolution+resolution -1);
            out << " ";
            out << QString::number(x*resolution);
            out << " ";
            out << QString::number((x+1)*resolution);
            out << "\n";

            out << "3 ";
            out << QString::number((x+1)*resolution);
            out << " ";
            out << QString::number((x+1)*resolution + resolution -1);
            out << " ";
            out << QString::number(x*resolution+resolution -1);
            out << "\n";
        }


        file.close();

    }
}

void Export::export_ply_bad(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution)
{
    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    QStringList file_list = file_name.split(".");

    QString file_id = file_list.at(0);
    file_id.append(".ply");
    path.append("/ply/bad/");
    QDir dir(path);
    if(!dir.exists())
    {
        dir.mkpath(".");
    }
    path.append(file_id);

    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();

    QVector<QSharedPointer<Cylinder> > bad_cylinders;
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cyl = it.next();
        if( (cyl->get_detection()== DetectionType::SPHEREFOLLOWING) && (cyl->get_allometry_improvement() == AllometryImproved::NOALLOM ))
        {

        } else {
            bad_cylinders.push_back(cyl);
        }
    }

    cylinders = bad_cylinders;
    int number_cylinders = cylinders.size();
    int number_points =  number_cylinders * resolution * 2;
    int number_faces  =  number_cylinders * resolution * 2;




    QFile file(path);

    if(file.open(QIODevice::WriteOnly))
    {
        QTextStream out(&file);


        out << "ply\n"
               "format ascii 1.0\n"
               "comment author: Jan Hackenberg\n"
               "comment object: SimpleTree calculated tree model\n";
        out << "element vertex ";
        out << QString::number(number_points).append("\n");
        out << "property float x\n"
               "property float y\n"
               "property float z\n"
               "property uchar red\n"
               "property uchar green\n"
               "property uchar blue\n";
        out << "element face ";
        out << QString::number(number_faces).append("\n");
        out << "property list uchar int vertex_index\n"
               "end_header\n";


        PointCloudS::Ptr cloud = generate_points_from_cylinder_list(cylinders,resolution);
        for(int i = 0; i < cloud->points.size(); i++)
        {
            PointS p = cloud->points.at(i);
            out << QString::number(p.x);
            out << " ";
            out << QString::number(p.y);
            out << " ";
            out << QString::number(p.z);
            out << " ";
            int index_cylinder = std::floor(i/(2.0f*resolution));
            QSharedPointer<Cylinder> cylinder = cylinders.at(index_cylinder);
            if(cylinder->get_improvement() == ImprovementType::RANSAC)
            {
                out << "255 0 0\n";
            } else
            {
                out << "0 255 0\n";
            }
        }
        for(int i = 0; i < cylinders.size(); i++)
        {
            int x = 2*i;
            for(int j = 0; j < resolution -1; j++)
            {
                out << "3 ";
                out << QString::number(x*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j+1);
                out << "\n";


                out << "3 ";
                out << QString::number((x+1)*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j);
                out << "\n";

            }

            out << "3 ";
            out << QString::number(x*resolution+resolution -1);
            out << " ";
            out << QString::number(x*resolution);
            out << " ";
            out << QString::number((x+1)*resolution);
            out << "\n";

            out << "3 ";
            out << QString::number((x+1)*resolution);
            out << " ";
            out << QString::number((x+1)*resolution + resolution -1);
            out << " ";
            out << QString::number(x*resolution+resolution -1);
            out << "\n";
        }


        file.close();

    }
}

void Export::export_ply_leave(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution)
{
    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    QStringList file_list = file_name.split(".");

    QString file_id = file_list.at(0);
    file_id.append(".ply");
    path.append("/ply/leave/");
    QDir dir(path);
    if(!dir.exists())
    {
        dir.mkpath(".");
    }
    path.append(file_id);


    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();


    QVector<QSharedPointer<Cylinder> > leave_cylinders;
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cyl = it.next();
        if( (cyl->get_segment()->is_leave() ))
        {
            leave_cylinders.push_back(cyl);
        }
    }

    cylinders = leave_cylinders;


    int number_cylinders = cylinders.size();
    int number_points =  number_cylinders * resolution * 2;
    int number_faces  =  number_cylinders * resolution * 2;
    QFile file(path);

    if(file.open(QIODevice::WriteOnly))
    {
        QTextStream out(&file);


        out << "ply\n"
               "format ascii 1.0\n"
               "comment author: Jan Hackenberg\n"
               "comment object: SimpleTree calculated tree model\n";
        out << "element vertex ";
        out << QString::number(number_points).append("\n");
        out << "property float x\n"
               "property float y\n"
               "property float z\n"
               "property uchar red\n"
               "property uchar green\n"
               "property uchar blue\n";
        out << "element face ";
        out << QString::number(number_faces).append("\n");
        out << "property list uchar int vertex_index\n"
               "end_header\n";


        PointCloudS::Ptr cloud = generate_points_from_cylinder_list(cylinders,resolution);
        for(int i = 0; i < cloud->points.size(); i++)
        {
            PointS p = cloud->points.at(i);
            out << QString::number(p.x);
            out << " ";
            out << QString::number(p.y);
            out << " ";
            out << QString::number(p.z);
            out << " ";
            int index_cylinder = std::floor(i/(2.0f*resolution));
            QSharedPointer<Cylinder> cylinder = cylinders.at(index_cylinder);
            if(cylinder->get_improvement() == ImprovementType::RANSAC)
            {
                out << "255 0 0\n";
            } else
            {
                out << "0 255 0\n";
            }
        }
        for(int i = 0; i < cylinders.size(); i++)
        {
            int x = 2*i;
            for(int j = 0; j < resolution -1; j++)
            {
                out << "3 ";
                out << QString::number(x*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j+1);
                out << "\n";


                out << "3 ";
                out << QString::number((x+1)*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j);
                out << "\n";

            }

            out << "3 ";
            out << QString::number(x*resolution+resolution -1);
            out << " ";
            out << QString::number(x*resolution);
            out << " ";
            out << QString::number((x+1)*resolution);
            out << "\n";

            out << "3 ";
            out << QString::number((x+1)*resolution);
            out << " ";
            out << QString::number((x+1)*resolution + resolution -1);
            out << " ";
            out << QString::number(x*resolution+resolution -1);
            out << "\n";
        }


        file.close();

    }
}

void Export::export_ply_no_leave(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution)
{
    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    QStringList file_list = file_name.split(".");

    QString file_id = file_list.at(0);
    file_id.append(".ply");
    path.append("/ply/no_leave/");
    QDir dir(path);
    if(!dir.exists())
    {
        dir.mkpath(".");
    }
    path.append(file_id);


    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();

    int number_cylinders = cylinders.size();
    int number_points =  number_cylinders * resolution * 2;
    int number_faces  =  number_cylinders * resolution * 2;


    QVector<QSharedPointer<Cylinder> > leave_cylinders;
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cyl = it.next();
        if( (!(cyl->get_segment()->is_leave()) ))
        {
            leave_cylinders.push_back(cyl);
        }
    }
    cylinders = leave_cylinders;

    QFile file(path);

    if(file.open(QIODevice::WriteOnly))
    {
        QTextStream out(&file);


        out << "ply\n"
               "format ascii 1.0\n"
               "comment author: Jan Hackenberg\n"
               "comment object: SimpleTree calculated tree model\n";
        out << "element vertex ";
        out << QString::number(number_points).append("\n");
        out << "property float x\n"
               "property float y\n"
               "property float z\n"
               "property uchar red\n"
               "property uchar green\n"
               "property uchar blue\n";
        out << "element face ";
        out << QString::number(number_faces).append("\n");
        out << "property list uchar int vertex_index\n"
               "end_header\n";




        PointCloudS::Ptr cloud = generate_points_from_cylinder_list(cylinders,resolution);
        for(int i = 0; i < cloud->points.size(); i++)
        {
            PointS p = cloud->points.at(i);
            out << QString::number(p.x);
            out << " ";
            out << QString::number(p.y);
            out << " ";
            out << QString::number(p.z);
            out << " ";
            int index_cylinder = std::floor(i/(2.0f*resolution));
            QSharedPointer<Cylinder> cylinder = cylinders.at(index_cylinder);
            if(cylinder->get_improvement() == ImprovementType::RANSAC)
            {
                out << "255 0 0\n";
            } else
            {
                out << "0 255 0\n";
            }
        }
        for(int i = 0; i < cylinders.size(); i++)
        {
            int x = 2*i;
            for(int j = 0; j < resolution -1; j++)
            {
                out << "3 ";
                out << QString::number(x*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j+1);
                out << "\n";


                out << "3 ";
                out << QString::number((x+1)*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j);
                out << "\n";

            }

            out << "3 ";
            out << QString::number(x*resolution+resolution -1);
            out << " ";
            out << QString::number(x*resolution);
            out << " ";
            out << QString::number((x+1)*resolution);
            out << "\n";

            out << "3 ";
            out << QString::number((x+1)*resolution);
            out << " ";
            out << QString::number((x+1)*resolution + resolution -1);
            out << " ";
            out << QString::number(x*resolution+resolution -1);
            out << "\n";
        }


        file.close();

    }
}

void Export::export_ply_improvement(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution)
{
    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    QStringList file_list = file_name.split(".");

    QString file_id = file_list.at(0);
    file_id.append(".ply");
    path.append("/ply/improvement/");
    QDir dir(path);
    if(!dir.exists())
    {
        dir.mkpath(".");
    }
    path.append(file_id);


    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
    int number_cylinders = cylinders.size();
    int number_points =  number_cylinders * resolution * 2;
    int number_faces  =  number_cylinders * resolution * 2;




    QFile file(path);

    if(file.open(QIODevice::WriteOnly))
    {
        QTextStream out(&file);


        out << "ply\n"
               "format ascii 1.0\n"
               "comment author: Jan Hackenberg\n"
               "comment object: SimpleTree calculated tree model\n";
        out << "element vertex ";
        out << QString::number(number_points).append("\n");
        out << "property float x\n"
               "property float y\n"
               "property float z\n"
               "property uchar red\n"
               "property uchar green\n"
               "property uchar blue\n";
        out << "element face ";
        out << QString::number(number_faces).append("\n");
        out << "property list uchar int vertex_index\n"
               "end_header\n";
        QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
        PointCloudS::Ptr cloud = generate_points_from_cylinder_list(cylinders,resolution);
        for(int i = 0; i < cloud->points.size(); i++)
        {
            PointS p = cloud->points.at(i);
            out << QString::number(p.x);
            out << " ";
            out << QString::number(p.y);
            out << " ";
            out << QString::number(p.z);
            out << " ";
            int index_cylinder = std::floor(i/(2.0f*resolution));
            QSharedPointer<Cylinder> cylinder = cylinders.at(index_cylinder);
            if(cylinder->get_improvement() == ImprovementType::RANSAC)
            {
                out << "255 0 0\n";
            }
            else
            {
                out << "0 255 0\n";
            }
        }
        for(int i = 0; i < cylinders.size(); i++)
        {
            int x = 2*i;
            for(int j = 0; j < resolution -1; j++)
            {
                out << "3 ";
                out << QString::number(x*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j+1);
                out << "\n";


                out << "3 ";
                out << QString::number((x+1)*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j);
                out << "\n";

            }

            out << "3 ";
            out << QString::number(x*resolution+resolution -1);
            out << " ";
            out << QString::number(x*resolution);
            out << " ";
            out << QString::number((x+1)*resolution);
            out << "\n";

            out << "3 ";
            out << QString::number((x+1)*resolution);
            out << " ";
            out << QString::number((x+1)*resolution + resolution -1);
            out << " ";
            out << QString::number(x*resolution+resolution -1);
            out << "\n";
        }


        file.close();

    }
}



void Export::export_ply_color(QString path, QString file_name, QSharedPointer<Tree> tree, int resolution)
{
    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    QStringList file_list = file_name.split(".");

    QString file_id = file_list.at(0);
    file_id.append(".ply");
    path.append("/ply/color/");
    QDir dir(path);
    if(!dir.exists())
    {
        dir.mkpath(".");
    }
    path.append(file_id);


    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
    int number_cylinders = cylinders.size();
    int number_points =  number_cylinders * resolution * 2;
    int number_faces  =  number_cylinders * resolution * 2;




    QFile file(path);

    if(file.open(QIODevice::WriteOnly))
    {
        QTextStream out(&file);


        out << "ply\n"
               "format ascii 1.0\n"
               "comment author: Jan Hackenberg\n"
               "comment object: SimpleTree calculated tree model\n";
        out << "element vertex ";
        out << QString::number(number_points).append("\n");
        out << "property float x\n"
               "property float y\n"
               "property float z\n"
               "property uchar red\n"
               "property uchar green\n"
               "property uchar blue\n";
        out << "element face ";
        out << QString::number(number_faces).append("\n");
        out << "property list uchar int vertex_index\n"
               "end_header\n";
        QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
        PointCloudS::Ptr cloud = generate_points_from_cylinder_list(cylinders,resolution);
        for(int i = 0; i < cloud->points.size(); i++)
        {
            PointS p = cloud->points.at(i);
            out << QString::number(p.x);
            out << " ";
            out << QString::number(p.y);
            out << " ";
            out << QString::number(p.z);
            out << " ";
            int index_cylinder = std::floor(i/(2.0f*resolution));
            QSharedPointer<Cylinder> cylinder = cylinders.at(index_cylinder);
            if(cylinder->get_segment()->get_branch_order() == 0)
            {
                out << "70 35 9\n";
            } else
            {
                out << "107 142 35\n";
            }
        }
        for(int i = 0; i < cylinders.size(); i++)
        {
            int x = 2*i;
            for(int j = 0; j < resolution -1; j++)
            {
                out << "3 ";
                out << QString::number(x*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j+1);
                out << "\n";


                out << "3 ";
                out << QString::number((x+1)*resolution+j+1);
                out << " ";
                out << QString::number((x+1)*resolution+j);
                out << " ";
                out << QString::number(x*resolution+j);
                out << "\n";

            }

            out << "3 ";
            out << QString::number(x*resolution+resolution -1);
            out << " ";
            out << QString::number(x*resolution);
            out << " ";
            out << QString::number((x+1)*resolution);
            out << "\n";

            out << "3 ";
            out << QString::number((x+1)*resolution);
            out << " ";
            out << QString::number((x+1)*resolution + resolution -1);
            out << " ";
            out << QString::number(x*resolution+resolution -1);
            out << "\n";
        }


        file.close();

    }
}

PointCloudS::Ptr Export::generate_points_from_cylinder_list(QVector<QSharedPointer<Cylinder> > cylinders, float resolution)
{
    PointCloudS::Ptr cloud  (new PointCloudS);
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        PointCloudS::Ptr cylinder_cloud = generate_points_from_cylinder(cylinder,resolution);
        *cloud += *cylinder_cloud;

    }
    return cloud;
}

PointCloudS::Ptr Export::generate_points_from_cylinder(QSharedPointer<Cylinder> cylinder, float resolution)
{
    double inc = (2.0 * M_PI) / ((float)resolution);
    double theta = 0;

    PointS center = cylinder->get_center();


    PointCloudS::Ptr untransformed_cloud (new PointCloudS);
    PointCloudS::Ptr transformed_cloud (new PointCloudS);
    float length = cylinder->get_length();
    float radius = cylinder->get_radius();
    for(int i = 0; i < resolution; i++)
    {
        float x = radius * (std::cos(theta));
        float y = radius * (std::sin(theta));
        float z = -length/2;
        PointS p (x,y,z);
        float branchorder = cylinder->get_segment()->get_branch_order();
        if(branchorder == 0)
        {
            p.is_stem = 0;
        } else {
            p.is_stem = 1;
        }
        untransformed_cloud->push_back(p);
        theta += inc;
    }
    theta = 0;

    for(int i = 0; i < resolution; i++)
    {
        float x = radius * (std::cos(theta));
        float y = radius * (std::sin(theta));
        float z = length/2;
        PointS p (x,y,z);
        float branchorder = cylinder->get_segment()->get_branch_order();
        if(branchorder == 0)
        {
            p.is_stem = 0;
        } else {
            p.is_stem = 1;
        }
        untransformed_cloud->push_back(p);
        theta += inc;
    }
    using namespace Eigen;
    using namespace std;

    Vector3f z_axis (0,0,1);
    Vector3f cylinder_axis (cylinder->values[3],cylinder->values[4],cylinder->values[5]);
    cylinder_axis = cylinder_axis.normalized();
    Vector3f rotation_axis= z_axis.cross(cylinder_axis);
    rotation_axis = rotation_axis.normalized();
    float angle = std::acos(z_axis.dot(cylinder_axis));

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(angle,rotation_axis));
    transform.translation() << center.x, center.y, center.z;
    pcl::transformPointCloud(*untransformed_cloud,*transformed_cloud, transform);

    return transformed_cloud;
}


void Export::export_tree_list(QVector<QSharedPointer<Tree> > tree_list, QString path, QVector<MethodCoefficients> coeff_list)
{
    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
    path.append("/result_list.csv");
    QFile file(path);
    int index = 0;
    if(file.open(QIODevice::WriteOnly))
    {

        QTextStream out(&file);
        out << "ID; volume (m^3); length (m); height (m); DBH (cm); DBH_taper_linear (cm); cut_height (m);x_position;y_position \n";
        QVectorIterator<QSharedPointer<Tree> > it(tree_list);
        while(it.hasNext())
        {

            QSharedPointer<Tree>  tree = it.next();
            if(true)
            {
                MethodCoefficients cf = coeff_list.at(index);
                index++;

                float cut_height = cf.cut_height;


                QString str;
                str.append(tree->getTreeID()).append(";");
                str.append(QString::number(tree->get_volume())).append(";");
                str.append(QString::number(tree->get_length(cf))).append(";");
                str.append(QString::number(tree->get_height(cf))).append(";");

                QVector<QSharedPointer<Cylinder> > cylinders = tree->get_stem_cylinders();
                QSharedPointer<Cylinder> dbh_cylinder = cylinders.at(0);
                QSharedPointer<Cylinder> dbh_cylinder2 = cylinders.at(0);
                float dbh_z = dbh_cylinder->get_start().z + (1.3f - cut_height);

                QVectorIterator<QSharedPointer<Cylinder> > git(cylinders);
                while(git.hasNext())
                {
                    QSharedPointer<Cylinder>  cyl = git.next();
                    float start_z = cyl->get_start().z;
                    float end_z = cyl->get_end().z;
                    if(start_z <= dbh_z && end_z >=dbh_z)
                    {
                        dbh_cylinder = cyl;
                    }

                }
                float dbh = 0;
                if(dbh_cylinder!=dbh_cylinder2)
                    dbh = dbh_cylinder->get_radius()*200;
                str.append(QString::number(dbh)).append(";");
                str.append(QString::number(200*Stem_Taper::get_DBH_from_taper(tree,cf))).append(";");
                QSharedPointer<Cylinder> root = tree->get_all_cylinders().at(0);
                str.append(QString::number(cf.cut_height)).append(";");
                str.append(QString::number(root->get_start().x)).append(";");
                str.append(QString::number(root->get_start().y));
                str.append("\n");
                out << str;
            }
        }


        file.close();

    }
}
