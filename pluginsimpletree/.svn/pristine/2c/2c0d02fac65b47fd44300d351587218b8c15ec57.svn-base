#ifndef SEGMENT_H
#define SEGMENT_H

#include <QObject>
#include <QSharedPointer>
#include <QVector>
#include <QWeakPointer>
#include <QEnableSharedFromThis>

#include <assert.h>

#include "cylinder.h"
#include "../typedef.h"
#include "src/math/simplemath.h"


class Segment : public QObject, public QEnableSharedFromThis<Segment>
{
    Q_OBJECT

private:
    /**
     * @brief _parent The parent segment
     */
    QWeakPointer<Segment> _parent;
    /**
     * @brief _children A list of all contained children;
     */
    QVector<QSharedPointer<Segment> > _children;

    /**
     * @brief _cylinders A list of all contained cylinders
     */
    QVector<QSharedPointer<Cylinder> > _cylinders;

    /**
     * @brief get_radii Returns a list of the radii of all cylinders
     * @return The list of radii
     */
    QVector<float>
    get_radii() const;




    /**
     * @brief _id The unique ID of this Segment
     */
    int _id = -1;
    /**
     * @brief _branch_id The unique ID of the branch containing this segment
     */
    int _branch_id = -1;

    /**
     * @brief _branch_order The branch order in which the segment is contained
     */
    int _branch_order = -1;

public:
    explicit Segment(QObject *parent = 0);

    /**
     * @brief add_cylinder Adds a cylinder to the segment, if it is not yet contained
     * @param cylinder The cylinder to be added
     * @return True if the cylinder was not contained before, false otherwise
     */
    bool
    add_cylinder(const QSharedPointer<Cylinder> cylinder);

    /**
     * @brief get_median_radius Computes the median radius of all contained cylinders
     * @return The median radius of this Segment
     */
    float
    get_median_radius() const;

    /**
     * @brief get_mean_radius Computes the mean radius of all contained cylinders
     * @return The mean radius of this Segment
     */
    float
    get_mean_radius() const;


    /**
     * @brief remove_child Removes a child segment from this
     * @param child The child segment to be removed
     * @return true if the child segment was contained, false otherwise
     */
    bool
    remove_child(QSharedPointer<Segment> child);

    /**
     * @brief remove Removes this Segment from the tree, i.e. removes the
     * connection to the parent Segment
     */
    bool
    remove();

    /**
     * @brief add_child_segment Adds a child segment if not contained yet
     * @param child The segment to be added
     * @return true if the child was not contained before, false otherwise
     */
    bool
    add_child_segment(QSharedPointer<Segment> child, bool connect_parent = true);




    /**
     * @brief get_start Returns the start point of the first cylinder
     * @return  The start point of the first cylinder
     */
    QSharedPointer<PointI>
    get_start() const;

    /**
     * @brief get_end Returns the end point of the last cylinder
     * @return The End point of the last cylinder
     */
    QSharedPointer<PointI> get_end() const;

    /**
     * @brief get_volume Returns the sum of all contained cylinders' volume
     * @return The total volume of all contained cylinders
     */
    float
    get_volume() const;

    /**
     * @brief get_length Returns the sum of all contained cylinders' length
     * @return The total length of all contained cylinders
     */
    float
    get_length() const;

    /**
     * @brief is_root Returns true if the Segment is the root segment
     * @return True if the Segment is the root segment
     */
    bool
    is_root() const;

    /**
     * @brief is_leave Returns true if the Segment is a leave segment
     * @return  True if the Segment is a leave segment
     */
    bool
    is_leave() const;

    /** \brief
     * \return A QString of form (Start.x,start.y,start.z,end.x,end.y,end.z,volume,length, median radius, ID, BranchID, BranchOrder)
     * */
    QString
    to_string () const;





    /**
     * @brief getBranch_order The getter function for branch order
     * @return The branch order
     */
    int get_branch_order() const;
    /**
     * @brief setBranch_order The setter function of the branch order
     * @param branch_order The branch order to be set
     */
    void set_branch_order(int branch_order);
    /**
     * @brief getBranch_id The getter function for the branch ID
     * @return The branch id to be set
     */
    int get_branch_id() const;
    /**
     * @brief setBranch_id The setter function for the branch ID
     * @param branch_id The branch ID to be set
     */
    void set_branch_id(int branch_id);
    /**
     * @brief getId The getter function for the ID
     * @return The ID of the segment
     */
    int get_id() const;
    /**
     * @brief setId The setter function of the ID
     * @param id The ID to be set
     */
    void set_id(int id);
    /**
     * @brief getChildren The getter function to retrieve the child segments
     * @return A vector of the child segments
     */
    QVector<QSharedPointer<Segment> > get_child_segments() const;
    /**
     * @brief setChildren The setter function of the child segments;
     * @param children the vector of the child segments
     */
    void set_child_segments(const QVector<QSharedPointer<Segment> > new_children);

    /**
     * @brief getCylinders The getter function of the cylinder vector
     * @return The vector of cylinders.
     */
    QVector<QSharedPointer<Cylinder> > get_cylinders() const;
    /**
     * @brief setCylinders The setter function of the cylinder vector
     * @param cylinders A vector of the cylinders
     */
    void set_cylinders(const QVector<QSharedPointer<Cylinder> > cylinders);

    /**
     * @brief getParent The getter function for the parent segment
     * @return The parent segment
     */
    QSharedPointer<Segment> get_parent_segment() const;
    /**
     * @brief setParent The setter function of the parent segment
     * @param parent The parent segment.
     */
    void set_parent_segment(const QSharedPointer<Segment> parent, bool connect_child = true);




    /**
     * @brief merge Merges if possible two cylinders into one;
     */
    void
    merge();

signals:

public slots:
};

#endif // SEGMENT_H
