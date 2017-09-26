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

#include "computeallometrySave.h"

float ComputeAllometrySave::get_a() const
{
    return _a;
}

void ComputeAllometrySave::set_a(float a)
{
    _a = a;
}

float ComputeAllometrySave::get_b() const
{
    return _b;
}

void ComputeAllometrySave::set_b(float b)
{
    _b = b;
}

float ComputeAllometrySave::getA_length() const
{
    return _a_length;
}

void ComputeAllometrySave::setA_length(float a_length)
{
    _a_length = a_length;
}

float ComputeAllometrySave::getB_length() const
{
    return _b_length;
}

void ComputeAllometrySave::setB_length(float b_length)
{
    _b_length = b_length;
}

Eigen::MatrixXd ComputeAllometrySave::generate_Jacobian(QVector<float> &rad)
{
    Eigen::MatrixXd jacobian;
    jacobian.resize(rad.size(),2);
    for(size_t i = 0; i < rad.size(); i++)
    {
        double xi1 = pow(rad.at(i),_b);
        double xi2 = _a*log(rad.at(i))*pow(rad.at(i),_b);
        jacobian(i,0) = xi1;
        jacobian(i,1) = xi2;
    }
    return jacobian;
}

Eigen::MatrixXd ComputeAllometrySave::generate_value_matrix(QVector<float>& rad, QVector<float>& vol)
{
    Eigen::MatrixXd  matrix;
    matrix.resize(rad.size(),1);
    for(size_t i = 0; i < rad.size(); i++)
    {
        double val = (_a*pow(rad.at(i), _b))-vol.at(i);
        matrix(i,0) = -val;
    }
    return matrix;
}

void ComputeAllometrySave::gauss_newton(QVector<float> &rad, QVector<float> &vol)
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
        if(update_a == _update_a&&update_b == _update_b)
        {
            k = 100;
        }
        update_a = _update_a;
        update_b = _update_b;
        _a += _update_a;
        _b += _update_b;
        k++;
    }

}

void ComputeAllometrySave::compute()
{
    QVector<float> vec_log_rad;
    QVector<float> vec_rad;

    QVector<float> vec_log_vol;
    QVector<float> vec_vol;
    generate_vectors(vec_log_rad, vec_log_vol,vec_rad, vec_vol);


    if(vec_log_rad.size()>5 || vec_log_vol.size()>5 || vec_rad.size()>5 || vec_vol.size()>5)
    {
        PointCloudS::Ptr cloud (new PointCloudS);
        for(size_t i = 0; i < vec_log_rad.size(); i++)
        {
            PointS p;
            p.y = vec_log_rad.at(i);
            p.x = vec_log_vol.at(i);
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
            _a = exp(log_a);
            _b = b_y/b_x;
//            if(_b > 0.1 && _b < 0.7)
//            {
            gauss_newton(vec_vol,vec_rad);
            correct();
//            }
        } catch (int i){
            ;
        }
    } else if(!_use_growth_volume)
    {
        _use_growth_volume = true;
        compute();
    }
}

void ComputeAllometrySave::correct()
{

    QVector<float> growth_volumina;
    QVector<QSharedPointer<Cylinder> > cylinders = _tree->get_all_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        float y;
        if(_use_growth_volume)
        {
            y = _tree->get_growth_volume(cylinder);
        } else {
            y = _tree->get_growth_length(cylinder);
        }
        growth_volumina.push_back(y);
    }

    if(!(_b > 0.25 && _b < 0.38))
    {
        /**
         *
         * files <- list.files()
a <- c()
b <- c()
for(file in files)
{
  QSM <- read.csv(file, sep = ";")
  x <- QSM$growth_volume
  y <- QSM$radius
  x.log <- log(x)
  y.log <- log(y)
  lm <- lm(y.log ~ x.log)
  sum <- summary(lm)
  cf <- sum$coefficients
  log.a <- cf[1,1]
  log.b <- cf[1,2]
  start.a <- exp(log.a)
  start.b <- log.b
  data2 <- data.frame(x,y)
  nls.vol <- nls(y ~ b1*x**b2,data = data2,start = list(b1 = start.a,b2 = start.b), control = list(maxiter=500,warnOnly = TRUE))
  a <- c(a,coefficients(nls.vol)[1])
  b <- c(b,coefficients(nls.vol)[2])
}
plot(a,b)
         * */

        _b =  0.3211048;
        _a = 0.1547178;
    }

    for(size_t i = 0; i < cylinders.size(); i++)
    {
        QSharedPointer<Cylinder> cylinder = cylinders.at(i);
        float x = growth_volumina.at(i);
        float radius = _a * (std::pow (x,_b));
        if(cylinder->get_allometry_improvement()!=AllometryImproved::NOALLOM)
        {
            cylinder->set_radius(radius);
            if(_use_growth_volume)
            {
                cylinder->set_allometry_improvement(AllometryImproved::ALLOM);
            }
            else
            {
                _a_length = _a;
                _b_length = _b;
                cylinder->set_allometry_improvement(AllometryImproved::ALLOM_LEN);
            }
        }
    }
}



void ComputeAllometrySave::generate_vectors(QVector<float> & rad, QVector<float> & vol, QVector<float> &rad_2, QVector<float> &vol_2)
{
    rad.clear();
    vol.clear();
    rad_2.clear();
    vol_2.clear();
    if(_use_growth_volume)
    {
        QVector<QSharedPointer<Cylinder> > cylinders = _tree->get_all_cylinders();

        QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
        while(it.hasNext())
        {
            QSharedPointer<Cylinder> cylinder = it.next();
            if(!cylinder->get_segment()->is_root())
            {
                if(!cylinder->get_segment()->is_leave())
                {
                    if(cylinder->get_allometry_improvement() == AllometryImproved::NOALLOM)
                    {
                        vol_2.push_back(_tree->get_growth_volume(cylinder));
                        rad_2.push_back(cylinder->get_radius());
                        rad.push_back(log(cylinder->get_radius()));
                        vol.push_back(log(_tree->get_growth_volume(cylinder)));
                    }
                }
            }
        }
    } else {
        QVector<QSharedPointer<Segment> > segments = _tree->get_all_segments();
        QVectorIterator<QSharedPointer<Segment> > it (segments);
        while(it.hasNext())
        {
            QSharedPointer<Segment> segment = it.next();

            if(!segment->is_root())
            {
                if(!segment->is_leave())
                {
                    if(segment->get_cylinders().first()->get_allometry_improvement() == AllometryImproved::NOALLOM)
                    {
                        vol_2.push_back(_tree->get_growth_length(segment->get_cylinders().first()));
                        rad_2.push_back(segment->get_mean_radius());
                        rad.push_back(log(segment->get_mean_radius()));
                        vol.push_back(log(_tree->get_growth_length(segment->get_cylinders().first())));
                    }
                }
            }
        }

    }
}

ComputeAllometrySave::ComputeAllometrySave(QSharedPointer<Tree> tree, bool use_growth_volume)
{
    _tree = tree;
    _use_growth_volume = use_growth_volume;
    compute();
}
