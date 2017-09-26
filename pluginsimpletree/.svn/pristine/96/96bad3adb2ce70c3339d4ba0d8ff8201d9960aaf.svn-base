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

#include "computeallometry_length.h"

float ComputeAllometryLength::get_a() const
{
    return _a_length;
}

void ComputeAllometryLength::set_a(float a)
{
    _a_length = a;
}

float ComputeAllometryLength::get_b() const
{
    return _b_length;
}

void ComputeAllometryLength::set_b(float b)
{
    _b_length = b;
}

Eigen::MatrixXd ComputeAllometryLength::generate_Jacobian(QVector<float> &rad)
{
    Eigen::MatrixXd jacobian;
    jacobian.resize(rad.size(),2);
    for(size_t i = 0; i < rad.size(); i++)
    {
        double xi1 = pow(rad.at(i),_b_length);
        double xi2 = _a_length*log(rad.at(i))*pow(rad.at(i),_b_length);
        jacobian(i,0) = xi1;
        jacobian(i,1) = xi2;
    }
    return jacobian;
}

Eigen::MatrixXd ComputeAllometryLength::generate_value_matrix(QVector<float>& rad, QVector<float>& vol)
{
    Eigen::MatrixXd  matrix;
    matrix.resize(rad.size(),1);
    for(size_t i = 0; i < rad.size(); i++)
    {
        double val = (_a_length*pow(rad.at(i), _b_length))-vol.at(i);
        matrix(i,0) = -val;
    }
    return matrix;
}

void ComputeAllometryLength::gauss_newton(QVector<float> &rad, QVector<float> &vol)
{
    int k = 0;
    float update_a;
    float update_b;
    while(k < 100)
    {

        Eigen::MatrixXd jacobian = generate_Jacobian(rad);
        Eigen::MatrixXd values   = generate_value_matrix(rad,vol);
        Eigen::FullPivHouseholderQR<Eigen::MatrixXd> x2(jacobian);
        Eigen::MatrixXd update = x2.solve(values);


        float _update_a = update(0,0);
        float _update_b = update(1,0);
        if(update_a == _update_a&&update_b == update_b)
        {
            k = 100;
        }
        update_a = _update_a;
        update_b = _update_b;
        _a_length += _update_a;
        _b_length += _update_b;
        k++;
    }

}

void ComputeAllometryLength::compute()
{
    QVector<float> vec_log_rad;
    QVector<float> vec_rad;

    QVector<float> vec_log_length;
    QVector<float> vec_length;

    generate_vectors(vec_log_rad, vec_log_length,vec_rad, vec_length);


    if(vec_log_rad.size()!=0 || vec_log_length.size()!=0 || vec_rad.size()!=0 || vec_length.size()!=0)
    {
        PointCloudS::Ptr cloud (new PointCloudS);
        for(size_t i = 0; i < vec_log_rad.size(); i++)
        {
            PointS p;
            p.x = vec_log_rad.at(i);
            p.y = vec_log_length.at(i);
            p.z = 0;
            cloud->points.push_back(p);
        }
        try

        {
            pcl::SACSegmentationFromNormals<PointS, PointS> seg;
            pcl::PointIndices::Ptr inliers_line (new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients_line (new pcl::ModelCoefficients);
            pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS> ());
            pcl::NormalEstimation<PointS, PointS> ne;

            ne.setSearchMethod (tree);
            ne.setInputCloud (cloud);
            ne.setKSearch (50);
            ne.compute (*cloud);

            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_LINE);
            seg.setNormalDistanceWeight (0.1);
            seg.setMethodType (pcl::SAC_MLESAC);
            seg.setMaxIterations (100);
            seg.setDistanceThreshold (1);
            seg.setInputCloud (cloud);
            seg.setInputNormals (cloud);

            // Obtain the plane inliers and coefficients
            seg.segment (*inliers_line, *coefficients_line);

            float a_x = coefficients_line->values.at(0);
            float a_y = coefficients_line->values.at(1);
            float b_x = coefficients_line->values.at(3);
            float b_y = coefficients_line->values.at(4);

            float fac;
            fac = a_x/b_x;
            float log_a = a_y - fac*(b_y);
            _a_length = exp(log_a);
            _b_length = b_y/b_x;

            gauss_newton(vec_rad,vec_length);

        } catch(...)
        {
            _a_length = 117.4;
            _b_length = 2.595;
            qDebug() << "allometry error in compute";
        }
    } else {
        _a_length = 117.4;
        _b_length = 2.595;
        qDebug() << "allometry error in compute 2";
    }

    if(_b_length < 1 || _b_length > 2.5)
    {
        //C:\Users\Jan Hackenberg\Documents\R\Standard_allom_length.R


        _b_length = 1.50203;
        _a_length = 5324;
    }
}



void ComputeAllometryLength::generate_vectors(QVector<float> & rad, QVector<float> & vol, QVector<float> &rad_2, QVector<float> &vol_2)
{
    rad.clear();
    vol.clear();
    rad_2.clear();
    vol_2.clear();

    float _min_radius = 0;
    float _min_volume = 0;
    QVector<QSharedPointer<Cylinder> > cylinders = _tree->get_all_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > tit(cylinders);
    while(tit.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = tit.next();
        float vol = _tree->get_growth_volume(cylinder);
        float radius = cylinder->get_radius();
        if(vol>_min_volume&&radius>_min_radius)
        {
            _min_volume = vol;
            _min_radius = radius;
        }
    }
    _min_volume = _min_volume;
    _min_radius = _min_radius;


    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {

        QSharedPointer<Cylinder> cylinder = it.next();
        if (cylinder->get_radius()>(_min_radius/_division_factor) && _tree->get_growth_volume(cylinder)>(_min_volume/_division_factor))
        {
            if(!cylinder->get_segment()->is_root())
            {
                if(!cylinder->get_segment()->is_leave())
                {
                    float growth_length = _tree->get_growth_length(cylinder);
                    vol_2.push_back(growth_length);
                    rad_2.push_back(cylinder->get_radius());

                    rad.push_back(log(cylinder->get_radius()));
                    vol.push_back(log(growth_length));
                }
            }
        }


//        if(cylinder->get_allometry_improvement()==AllometryImproved::NOALLOM && cylinder->get_detection()== DetectionType::SPHEREFOLLOWING)
//        {
//            float growth_length = _tree->get_growth_length(cylinder);
//            vol_2.push_back(growth_length);
//            rad_2.push_back(cylinder->get_radius());

//            rad.push_back(log(cylinder->get_radius()));
//            vol.push_back(log(growth_length));
//        }
    }
}

ComputeAllometryLength::ComputeAllometryLength(QSharedPointer<Tree> tree, float division_factor)
{
    _division_factor = 9;
    _tree = tree;
    compute();
}
