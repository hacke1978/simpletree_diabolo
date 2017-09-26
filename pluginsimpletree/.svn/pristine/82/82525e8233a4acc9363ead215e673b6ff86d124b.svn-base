/****************************************************************************

 Copyright (C) 2016-2017 Jan Hackenberg
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

#ifndef CYLINDER_H
#define CYLINDER_H



#include <assert.h>
#include <cmath>
#include "../typedef.h"

#include <boost/math/constants/constants.hpp>

#include <pcl/common/distances.h>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>

#include <QObject>
#include <QSharedPointer>
#include <QWeakPointer>
#include <QVector>
#include <QDebug>
#include <QEnableSharedFromThis>

class Segment;

enum AllometryImproved {NOALLOM,ALLOM, ALLOM_LEN, ALLOM_TAPER, ALLOM_PIPE, ALLOM_MED, ALLOM_MEDIAN};

enum ImprovementType {RANSAC, MEDIAN, NO};

enum DetectionType {SPHEREFOLLOWING, ATTRACTOR};

enum TaperType {TAPER_APPLIED, NOTAPER_APPLIED};

enum MedianType {MEDIAN_APPLIED, NOMEDIAN_APPLIED};


class Cylinder : public QObject, public QEnableSharedFromThis<Cylinder>, public pcl::ModelCoefficients
{
    Q_OBJECT

private:
    bool well_fitted = false;

    /**
     * @brief segment the segment the cylinder belongs to
     */
    QWeakPointer<Segment> segment;

    /**
     * @brief _max_distance the maximum distance in meter between two cylinder to be treated equal
     */
    const static float _MAX_DISTANCE;

    AllometryImproved allom = AllometryImproved::NOALLOM;



    ImprovementType improvement = ImprovementType::NO;

    DetectionType detection = DetectionType::SPHEREFOLLOWING;

    float dist_on_axis(const PointS point) const;

    /**
     * @brief dist_on_axis Returns the distance of the point's projection to the axis segment
     * @param point the point to compute the distane
     * @return the returned distance
     */
    float
    dist_on_axis_ptr(const QSharedPointer<PointS> point) const;

    float mean_distance = std::numeric_limits<float>::max();

    float mean_distance_sqrd = std::numeric_limits<float>::max();

    float mean_distance_sqrd_angle = std::numeric_limits<float>::max();

    int number_pts = 0;

    int ID = -1;

    int parent_ID = -1;


public:

    QSharedPointer<Cylinder>
    clone();

    explicit Cylinder(pcl::ModelCoefficients coeff, QObject *parent = 0);

    /**
     * @brief Cylinder Copy constructor
     * @param cylinder the cylinder to be copied
     */
    Cylinder(const Cylinder &cylinder);

    /**
     * @brief get_axis returns the axis of this cylinder as Eigen Vector3f
     * @return the axis
     */
    Eigen::Vector3f
    get_axis();




    /** \brief returns the cylinders radius
     * */
    float
    get_radius() const;

    void
    set_radius(const float radius);



    /** \brief returns the axis length of the cylinder
     * */
    float
    get_length() const;

    /** \brief returns half of the axis length
     * */
    float
    get_half_length() const;

    /** \brief returns the maximum distance between a surface point and the center point
     * */
    float
    get_half_size() const;

    float
    get_volume() const;

    const PointS get_center() const;

    const PointS get_start() const;

    const PointS get_end() const;


    pcl::ModelCoefficients get_coeff() const;

    void set_coeff(pcl::ModelCoefficients coeff);


    /** \brief returns the cylinders center point
     * */
    const QSharedPointer<PointS> get_center_ptr() const;

    /** \brief returns the pcl modelcoefficient values of this cylinder
     * */
    std::vector<float>
    get_values ();

    /** \brief sets a weak pointer to the segment containing this cylinder
     * */
    void
    set_segment (QWeakPointer<Segment> segment);

    /** \brief returns a shared pointer to the segment containing this cylinder
     * */
    QSharedPointer<Segment>
    get_segment ();

    /** \brief returns true if cylinders start point, end points and radius are equal
     * */
    bool    operator == ( const QSharedPointer<Cylinder> cylinder2);

    /** \brief returns true if the childs start point is with the defined minimum distance to the cylinders end point
     * */
    bool
    is_parent_of (const QSharedPointer<Cylinder> child) const;

    /** \brief returns true if the parents end point is with the defined minimum distance to the cylinders start point
     * */
    bool
    is_child_of (const  QSharedPointer<Cylinder> parent) const;

    /**
     * @brief set_start_end Resets Start and End point of this cylinder
     * @param start The start point
     * @param end the end point
     */
    void
    set_start_end(const QSharedPointer<PointS> start, const QSharedPointer<PointS> end);

    PointS projection_on_axis(const PointS point) const;


    /**
     * @brief projection_on_axis Projects a point on the cylinder axis
     * @param point the Point to be projected on the axis
     * @return the projection of point on the axis
     */
    QSharedPointer<PointS> projection_on_axis_ptr(const QSharedPointer<PointS> point) const;

    float
    dist_to_axis(const QSharedPointer<PointS> point) const;

    float dist_to_point(const PointS point) const;



    /** \brief returns the distance to a point
     * \param point: the point to which the distance is computed
     * */
    float
    dist_to_point_ptr (const QSharedPointer<PointS> point) const;

    /** \brief
     * \return the cylinders start point
     * */
    QSharedPointer<PointS>
    get_start_ptr () const;

    /** \brief
     * \return the cylinders end point
     * */
    QSharedPointer<PointS>
    get_end_ptr () const;



    /** \brief
     * \return A QString of form (Start.x,start.y,start.z,end.x,end.y,end.z,volume,length,radius)
     * */
    QString
    to_string () const;

    AllometryImproved
    get_allometry_improvement() const;

    void set_allometry_improvement(AllometryImproved value);



    /**
     * @brief getImprovement Returns how the cylinder was improved
     * @return the improvement type
     */
    ImprovementType get_improvement() const;

    /**
     * @brief setImprovement Sets the improvement type
     * @param value the improvement type
     */
    void set_improvement(ImprovementType value);
    /**
     * @brief get_detection Returns the detection method
     * @return the detection method
     */
    DetectionType get_detection() const;

    /**
     * @brief set_detection Sets the detection method
     * @param value the detection method
     */
    void set_detection(const DetectionType &value);

    float getMean_distance() const;
    void setMean_distance(float value);

    float getMean_distance_sqrd() const;
    void setMean_distance_sqrd(float value);

    float getMean_distance_sqrd_angle() const;
    void setMean_distance_sqrd_angle(float value);

    int getNumber_pts() const;
    void setNumber_pts(int value);

    bool getWell_fitted() const;
    void setWell_fitted(bool value);

    int getID() const;
    void setID(int value);

    int getParent_ID() const;
    void setParent_ID(int value);

signals:

public slots:
};

#endif // CYLINDER_H
