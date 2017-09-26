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

#include "computeallometry_length_RC.h"

float ComputeAllometryLengthRC::get_a() const
{
    return _a_length;
}

void ComputeAllometryLengthRC::set_a(float a)
{
    _a_length = a;
}

float ComputeAllometryLengthRC::get_b() const
{
    return _b_length;
}

void ComputeAllometryLengthRC::set_b(float b)
{
    _b_length = b;
}

float ComputeAllometryLengthRC::get_c() const
{
    return _c_length;
}

void ComputeAllometryLengthRC::set_c(float c_length)
{
    _c_length = c_length;
}

Eigen::MatrixXd ComputeAllometryLengthRC::generate_Jacobian(QVector<float> &rad)
{
    Eigen::MatrixXd jacobian;
    jacobian.resize(rad.size(),3);
    for(size_t i = 0; i < rad.size(); i++)
    {
        double xi1 = std::pow( (1 - std::pow( EulerConstant,(-_b_length*rad.at(i)) ) ),_c_length);

        double xi2 = _a_length*rad.at(i)*_c_length*(std::pow(EulerConstant,(-_b_length*rad.at(i))))*
                (std::pow( (1 - std::pow( EulerConstant,(-_b_length*rad.at(i)) ) ),(_c_length-1) ));
        double xi3 = _a_length*log(1-( std::pow( EulerConstant,(-_b_length*rad.at(i)) ) ) ) * std::pow(1-( std::pow( EulerConstant,(-_b_length*rad.at(i)) ) ),_c_length);
                jacobian(i,0) = xi1;
        jacobian(i,1) = xi2;
        jacobian(i,2) = xi3;
    }
    return jacobian;
}

Eigen::MatrixXd ComputeAllometryLengthRC::generate_value_matrix(QVector<float>& rad, QVector<float>& vol)
{
    Eigen::MatrixXd  matrix;
    matrix.resize(rad.size(),1);
    for(size_t i = 0; i < rad.size(); i++)
    {
//        double val = (_a_length*pow(rad.at(i), _b_length))-vol.at(i);
        double val = ((_a_length* std::pow( (1-  std::pow( EulerConstant,(-_b_length*rad.at(i)) ) ),_c_length) ) - vol.at(i));
        matrix(i,0) = -val;
    }
    return matrix;
}

void ComputeAllometryLengthRC::gauss_newton(QVector<float> &rad, QVector<float> &vol)
{
    int k = 0;
    float update_a;
    float update_b;
    float update_c;
    while(k < 100)
    {

        Eigen::MatrixXd jacobian = generate_Jacobian(rad);
        Eigen::MatrixXd values   = generate_value_matrix(rad,vol);
        Eigen::FullPivHouseholderQR<Eigen::MatrixXd> x2(jacobian);
        Eigen::MatrixXd update = x2.solve(values);


        float _update_a = update(0,0);
        float _update_b = update(1,0);
        float _update_c = update(2,0);
        if(update_a == _update_a&&update_b == update_b)
        {
            k = 100;
        }
        update_a = _update_a;
        update_b = _update_b;
        update_c = _update_c;
        _a_length += _update_a;
        _b_length += _update_b;
        _c_length += _update_c;
        k++;
    }

}

void ComputeAllometryLengthRC::compute()
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

            float max_r = SimpleMath<float>::get_max(vec_rad);
            float max_l = SimpleMath<float>::get_max(vec_length);
            _a_length = max_l;
            _b_length = std::log(0.001)/(-max_r);
            _c_length = 25;




            gauss_newton(vec_rad,vec_length);

        } catch(...)
        {
            float max_r = SimpleMath<float>::get_max(vec_rad);
            float max_l = SimpleMath<float>::get_max(vec_length);
            _a_length = max_l;
            _b_length = std::log(0.001)/(-max_r);
            _c_length = 25;
            qDebug() << "allometry error in compute length_RC";
        }
    } else {
        float max_r = SimpleMath<float>::get_max(vec_rad);
        float max_l = SimpleMath<float>::get_max(vec_length);
        _a_length = max_l;
        _b_length = std::log(0.001)/(-max_r);
        _c_length = 25;
        qDebug() << "allometry error in compute length_RC ";
    }

    if(_c_length < 1 || _c_length > 50)
    {
         qDebug() << "allometry error in compute length_RC ";
        float max_r = SimpleMath<float>::get_max(vec_rad);
        float max_l = SimpleMath<float>::get_max(vec_length);
        _a_length = max_l;
        _b_length = std::log(0.001)/(-max_r);
        _c_length = 25;
    }
}



void ComputeAllometryLengthRC::generate_vectors(QVector<float> & rad, QVector<float> & vol, QVector<float> &rad_2, QVector<float> &vol_2)
{
    rad.clear();
    vol.clear();
    rad_2.clear();
    vol_2.clear();
    QVector<QSharedPointer<Cylinder> > cylinders = _tree->get_all_cylinders();
    float max_len = 0;
    float max_rad = 0;
    QVectorIterator<QSharedPointer<Cylinder> > git(cylinders);
    while(git.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = git.next();
        if(cylinder->get_allometry_improvement()==AllometryImproved::NOALLOM && cylinder->get_detection()== DetectionType::SPHEREFOLLOWING)
        {
            float growth_length = _tree->get_growth_length(cylinder);
            if(growth_length > max_len)
                max_len = growth_length;
            if(cylinder->get_radius()> max_rad)
                max_rad = cylinder->get_radius();
        }
    }
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        if(cylinder->get_allometry_improvement()==AllometryImproved::NOALLOM && cylinder->get_detection()== DetectionType::SPHEREFOLLOWING)
        {
            if(cylinder->get_radius()>=max_rad/2 && _tree->get_growth_length(cylinder)>=max_len/2 )
            {
                float growth_length = _tree->get_growth_length(cylinder);
                vol_2.push_back(growth_length);
                rad_2.push_back(cylinder->get_radius());
                rad.push_back(log(cylinder->get_radius()));
                vol.push_back(log(growth_length));
            }
        }
    }
}

ComputeAllometryLengthRC::ComputeAllometryLengthRC(QSharedPointer<Tree> tree)
{
    _tree = tree;
    compute();
}
