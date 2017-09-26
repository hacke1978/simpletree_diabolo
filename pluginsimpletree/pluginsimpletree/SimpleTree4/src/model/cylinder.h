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

class Segment;

enum ImprovementType {RANSAC, MEDIAN, NO};

class Cylinder : public QObject, public pcl::ModelCoefficients
{
    Q_OBJECT

private:
    /**
     * @brief segment the segment the cylinder belongs to
     */
    QWeakPointer<Segment> segment;

    /**
     * @brief _max_distance the maximum distance in meter between two cylinder to be treated equal
     */
    const static float _MAX_DISTANCE;

    /**
     * @brief dist_on_axis Returns the distance of the point's projection to the axis segment
     * @param point the point to compute the distane
     * @return the returned distance
     */
    float
    dist_on_axis(const QSharedPointer<PointI> point) const;

    ImprovementType improvement = ImprovementType::NO;


public:
    explicit Cylinder(pcl::ModelCoefficients coeff, QObject *parent = 0);


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


    /** \brief returns the cylinders center point
     * */
    const QSharedPointer<PointI> get_center() const;

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

//    /**
//     * @brief get_children \brief returns the children of this cylinder which are contained in the list
//     * @param cylinders the list of potential children
//     * @return  all cylinders with a child relation from the input list
//     */

//    const QVector<QSharedPointer<Cylinder> >
//    get_children (QVector<QSharedPointer<Cylinder> >& cylinders) const;


    /**
     * @brief projection_on_axis Projects a point on the cylinder axis
     * @param point the Point to be projected on the axis
     * @return the projection of point on the axis
     */
    QSharedPointer<PointI> projection_on_axis(const QSharedPointer<PointI> point) const;



    /** \brief returns the distance to a point
     * \param point: the point to which the distance is computed
     * */
    float
    dist_to_point (const QSharedPointer<PointI> point) const;

    /** \brief
     * \return the cylinders start point
     * */
    QSharedPointer<PointI>
    get_start () const;

    /** \brief
     * \return the cylinders end point
     * */
    QSharedPointer<PointI>
    get_end () const;

    /** \brief
     * \return A QString of form (Start.x,start.y,start.z,end.x,end.y,end.z,volume,length,radius)
     * */
    QString
    to_string () const;



    /**
     * @brief getImprovement Returns how the cylinder was improved
     * @return the improvement type
     */
    ImprovementType getImprovement() const;

    /**
     * @brief setImprovement Sets the improvement type
     * @param value the improvement type
     */
    void setImprovement(ImprovementType value);

signals:

public slots:
};

#endif // CYLINDER_H
