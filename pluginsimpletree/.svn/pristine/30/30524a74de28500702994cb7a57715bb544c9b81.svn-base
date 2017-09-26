

#include "tree.h"


const QString Tree::getTreeID() const
{
    return _treeID;
}

void Tree::setTreeID(const QString &value)
{
    _treeID = value;
}

QSharedPointer<Segment> Tree::get_root_segment() const
{
    return _root_segment;
}

QVector<pcl::ModelCoefficients> Tree::get_pcl_coefficients() const
{
    QVector<QSharedPointer<Cylinder> > cylinders = get_all_cylinders();
    QVector<pcl::ModelCoefficients> coeff;
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        pcl::ModelCoefficients cf;
        cf.values = cylinder->values;
        coeff.push_back(cf);
    }
    return coeff;
}

const float Tree::get_min_height() const
{
    QVector<QSharedPointer<Cylinder> > cylinders = get_all_cylinders();
    float min_height = std::numeric_limits<float>::max();
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        QSharedPointer<PointS> start = cylinder->get_start_ptr();
        QSharedPointer<PointS> end = cylinder->get_end_ptr();
        if(start->z < min_height)
        {
            min_height = start->z;
        }
        if(end->z < min_height)
        {
            min_height = end->z;
        }

    }
    return min_height;
}

const float Tree::get_max_height() const
{
    QVector<QSharedPointer<Cylinder> > cylinders = get_all_cylinders();
    float max_height = std::numeric_limits<float>::lowest();
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        QSharedPointer<PointS> end = cylinder->get_end_ptr();
        if(end->z > max_height)
        {
            max_height = end->z;
        }
    }
    return max_height;
}

QSharedPointer<Cylinder> Tree::get_cylinder_direction(QVector<QSharedPointer<Cylinder> > stem_cylinders, float save_height)
{
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    QVectorIterator<QSharedPointer<Cylinder> > it (stem_cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        float z = cylinder->get_start().z;
        if(min_z > z)
            min_z = z;
    }
    max_z = min_z + save_height;
    min_z = min_z - 0.1;

    PointS start;
    PointS end;
    float radius = 0;

    float min_found_z = max_z;
    float max_found_z = min_z;


    QVectorIterator<QSharedPointer<Cylinder> > git (stem_cylinders);
    while(git.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = git.next();
        float z = cylinder->get_start().z;
        if(min_found_z > z)
        {
            min_found_z = z;
            start = cylinder->get_start();
            radius = std::max(cylinder->get_radius(),radius);
        }
        if(max_found_z < z)
        {
            max_found_z = z;
            end = cylinder->get_end();
            radius = std::max(cylinder->get_radius(),radius);
        }
    }
    pcl::ModelCoefficients cf;
    cf.values.push_back(start.x);
    cf.values.push_back(start.y);
    cf.values.push_back(start.z);
    cf.values.push_back(end.x - start.x);
    cf.values.push_back(end.y - start.y);
    cf.values.push_back(end.z - start.z);
    cf.values.push_back(radius);

    QSharedPointer<Cylinder>  direction_cylinder (new Cylinder(cf));
    return direction_cylinder;
}

QSharedPointer<Cylinder>  Tree::get_cylinder_in_height(float height, MethodCoefficients cf)
{
    float cut_height = cf.cut_height;

    QVector<QSharedPointer<Cylinder> > cylinders = get_stem_cylinders();
    QSharedPointer<Cylinder> dbh_cylinder = cylinders.at(0);
    QSharedPointer<Cylinder> dbh_cylinder2 = cylinders.at(0);
    QSharedPointer<Cylinder> dbh_cylinder3;
    float dbh_z = dbh_cylinder->get_start().z + (height - cut_height);

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
    if(dbh_cylinder!=dbh_cylinder2|| height < 1.3f)
        return dbh_cylinder;
    return dbh_cylinder3;
}

QSharedPointer<Tree> Tree::clone()
{
    QSharedPointer<Segment> root_segment = _root_segment;
    QSharedPointer<Segment> root_segment_copy = root_segment->clone();
    QSharedPointer<Tree> tree (new Tree(root_segment_copy,_treeID) );
    return tree;
}

const QVector<QSharedPointer<Cylinder> > Tree::get_stem_cylinders() const
{
    QVector<QSharedPointer<Cylinder> > cylinders = get_all_cylinders();
    QVector<QSharedPointer<Cylinder> > stem_cylinders;
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        if(cylinder->get_segment()->get_branch_order()==0)
        {
            stem_cylinders.push_back(cylinder);
        }
    }
    return stem_cylinders;
}

QSharedPointer<Cylinder> Tree::get_parent(QSharedPointer<Cylinder> cyl)
{

    QSharedPointer<Cylinder> parent (cyl);
    QSharedPointer<Segment> segment = cyl->get_segment();
    QVector<QSharedPointer<Cylinder> > cylinders = segment->get_cylinders();
    int index = cylinders.indexOf(cyl);
    if(index > 0)
    {
        return cylinders.at(index-1);
    } else
    {
        if(!segment->is_root())
        {
            QSharedPointer<Segment> parent = segment->get_parent_segment();
            return parent->get_cylinders().last();
        }
    }

    return parent;

}

QSharedPointer<Cylinder> Tree::get_child(QSharedPointer<Cylinder> cyl)
{
    QSharedPointer<Cylinder> child (cyl);
    QSharedPointer<Segment> segment = cyl->get_segment();
    QVector<QSharedPointer<Cylinder> > cylinders = segment->get_cylinders();
    int index = cylinders.indexOf(cyl);
    int size = cylinders.size();
    if(index < size-2)
    {
        return cylinders.at(index+1);
    }
    else
    {
        if(!segment->is_leave())
        {
            QSharedPointer<Segment> child = segment->get_child_segments().first();
            return child->get_cylinders().first();
        }
    }

    return child;
}

QVector<QSharedPointer<Cylinder> > Tree::get_children_non_recursive(QSharedPointer<Cylinder> cyl)
{
    QVector<QSharedPointer<Cylinder> > children;
    QSharedPointer<Segment> segment = cyl->get_segment();
    QVector<QSharedPointer<Cylinder> > cylinders = segment->get_cylinders();
    int index = -1;
    for(size_t i = 0; i < cylinders.size(); i++)
    {
        if(cyl == cylinders.at(i))
        {
            index = i;
        }
    }
    if(index == -1)
    {
        qDebug () << "impoosible Tree:get_children_non_recursive";
    } else {
        if(index < cylinders.size()-1)
        {
            children.push_back(cylinders.at(index + 1));
        } else {
            QVector<QSharedPointer<Segment> > child_segments = segment->get_child_segments();
            QVectorIterator<QSharedPointer<Segment> > it(child_segments);
            while(it.hasNext())
            {
                QSharedPointer<Segment> child_segment = it.next();
                children.push_back(child_segment->get_cylinders().first());
            }
        }
    }
    return children;
}

QVector<QSharedPointer<Cylinder> > Tree::get_neighbor_cylinders(QSharedPointer<Cylinder> cyl, int neighborhoodsize)
{
    QVector<QSharedPointer<Cylinder> > neighborhood;
    neighborhood.push_back(cyl);

    QSharedPointer<Cylinder>  current = cyl;
    for(int i = 0; i < neighborhoodsize; i++)
    {
        current = get_parent(current);
        neighborhood.push_back(current);
    }

    current = cyl;
    for(int i = 0; i < neighborhoodsize; i++)
    {
        current = get_child(current);
        neighborhood.push_back(current);
    }
    return neighborhood;
}

QVector<QSharedPointer<Cylinder> > Tree::get_stem_cylinders_save()
{
    QVector<QSharedPointer<Cylinder> > stem_cylinders = get_stem_cylinders();
    QSharedPointer<Cylinder> direction_cylinder = get_cylinder_direction(stem_cylinders);
    QVector<QSharedPointer<Cylinder> > stem_cylinders_save;
    bool add = true;
    float max_radius = direction_cylinder->get_radius() + 0.2;
    QVectorIterator<QSharedPointer<Cylinder> > it(stem_cylinders);
    while(it.hasNext()&& add == true)
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        QSharedPointer<PointS> start = cylinder->get_start_ptr();
        QSharedPointer<PointS> end   = cylinder->get_end_ptr();
        float dist1  = direction_cylinder->dist_to_axis(start);
        float dist2  = direction_cylinder->dist_to_axis(end);
        if(dist1 > max_radius || dist2 > max_radius)
        {
            add = false;
        }
        else
        {
            stem_cylinders_save.push_back(cylinder);
        }
    }
    return stem_cylinders_save;
}

QString Tree::to_string(const QSharedPointer<Cylinder> &cylinder, const MethodCoefficients &coefficients)
{
    QString str;

    str.append(QString::number(cylinder->get_segment()->get_branch_id()));
    str.append(";");
    str.append(QString::number(cylinder->get_segment()->get_branch_order()));
    str.append(";");
    str.append(QString::number(cylinder->get_segment()->get_id()));
    str.append(";");
    if(!cylinder->get_segment()->is_root())
    {
        str.append(QString::number(cylinder->get_segment()->get_parent_segment()->get_id()));
        str.append(";");
    }
    else
    {
        str.append("-1");
        str.append(";");
    }
    str.append(QString::number(get_growth_volume(cylinder)));
    str.append(";");
    str.append(coefficients.species);
    str.append(";");
    str.append(coefficients.id);
    str.append(";");

    {
        int detection  = cylinder->get_detection();

        switch (detection) {
        case 0:
            str.append("spherefollowing");
            break;
        case 1:
            str.append("attractor");
            break;
        default:
            str.append("spherefollowing");
            break;
        }
        str.append(";");
    }



    {
        int improvement  = cylinder->get_improvement();

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



    str.append(QString::number(cylinder->get_start_ptr()->x));
    str.append(";");

    str.append(QString::number(cylinder->get_start_ptr()->y));
    str.append(";");

    str.append(QString::number(cylinder->get_start_ptr()->z));
    str.append(";");

    str.append(QString::number(cylinder->get_end_ptr()->x));
    str.append(";");

    str.append(QString::number(cylinder->get_end_ptr()->y));
    str.append(";");

    str.append(QString::number(cylinder->get_end_ptr()->z));
    str.append(";");

    str.append(QString::number(cylinder->get_radius()));
    str.append(";");

    str.append(QString::number(cylinder->get_length()));
    str.append(";");



    str.append(QString::number(coefficients.a));
    str.append(";");

    str.append(QString::number(coefficients.b));
    str.append(";");


    str.append(QString::number(coefficients.sd));
    str.append(";");

    str.append(QString::number(coefficients.mean));
    str.append(";");

    str.append(QString::number(coefficients.ransac_circle_inlier_distance));
    str.append(";");

    str.append(QString::number(coefficients.ransac_inlier_distance));
    str.append(";");

    str.append(QString::number(coefficients.ransac_iterations));
    str.append(";");
    {

        int ransac_type = coefficients.ransac_circle_type;

        switch (ransac_type) {
        case 0:
            str.append("RANSAC");
            break;
        case 1:
            str.append("LMEDS");
            break;
        case 2:
            str.append("MSAC");
            break;
        case 3:
            str.append("RRANSAC");
            break;
        case 4:
            str.append("RMSAC");
            break;
        case 5:
            str.append("MLESAC");
            break;
        case 6:
            str.append("PROSAC");
            break;

        default:
            str.append("RANSAC");
            break;
        }
        str.append(";");
    }

    {

        int ransac_type = coefficients.ransac_type;

        switch (ransac_type) {
        case 0:
            str.append("RANSAC");
            break;
        case 1:
            str.append("LMEDS");
            break;
        case 2:
            str.append("MSAC");
            break;
        case 3:
            str.append("RRANSAC");
            break;
        case 4:
            str.append("RMSAC");
            break;
        case 5:
            str.append("MLESAC");
            break;
        case 6:
            str.append("PROSAC");
            break;

        default:
            str.append("RANSAC");
            break;
        }
        str.append(";");
    }

    str.append(QString::number(coefficients.sphere_radius_multiplier));
    str.append(";");

    str.append(QString::number(coefficients.epsilon_cluster_stem));
    str.append(";");

    str.append(QString::number(coefficients.epsilon_cluster_branch));
    str.append(";");

    str.append(QString::number(coefficients.epsilon_sphere));
    str.append(";");

    str.append(QString::number(coefficients.min_radius_sphere));
    str.append(";");

    str.append(QString::number(coefficients.min_dist));
    str.append(";");

    str.append(QString::number(coefficients.tree_height));
    str.append(";");

    str.append(QString::number(coefficients.tree_circumference));
    str.append(";");

    str.append(QString::number(coefficients.tree_predicted_volume));
    str.append("\n");

    return str;
}

const float Tree::get_volume_to_base(const QSharedPointer<Segment> segment, const QSharedPointer<Segment> root) const
{
    float volume = segment->get_volume();
    if(segment!= root)
    {
        if(segment->get_parent_segment())
        {
            volume +=  get_volume_to_base(segment->get_parent_segment(),root);
        }
    }
    return volume;
}

const QVector<QSharedPointer<Segment> > Tree::get_segments_between(const QSharedPointer<Segment> base, const QSharedPointer<Segment> leave) const
{
    QSharedPointer<Segment> current = leave;
    QVector<QSharedPointer<Segment> >  segments;
    while(current!= base)
    {
        segments.push_back(current);
        current = current->get_parent_segment();
    }
    segments.push_back(base);
    return segments;
}

const float Tree::get_sum_length(const QVector<QSharedPointer<Segment> > segments) const
{
    float sum = 0;
    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        sum += segment->get_length();
    }
    return sum;
}

const float Tree::get_length_to_leave(const QSharedPointer<Cylinder> cylinder) const
{
    float length = 0;
    float spherefollowing_length = 0;
    float attractor_length = 0;
    PointS first (0,0,0);
    PointS last (0,0,0);
    QSharedPointer<Segment> segment = cylinder->get_segment();

    bool has_attractor = false;


    QVector<QSharedPointer<Segment> > segments = get_segments_longest_path(segment);
    QVector<QSharedPointer<Segment> > att_segments;
    QVectorIterator<QSharedPointer<Segment> > it(segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> seg = it.next();

        if(seg->get_cylinders().first()->get_detection()!= DetectionType::SPHEREFOLLOWING)
        {
            has_attractor = true;
            att_segments.push_back(seg);
        }
        else
        {
            spherefollowing_length += seg->get_length();
        }
    }
    if(has_attractor)
    {
        first = att_segments.first()->get_cylinders().last()->get_end();
        last = att_segments.last()->get_cylinders().first()->get_start();
        attractor_length =SimpleMath<float>::get_distance(first, last);
    }
    float length_inside_segment = 0;
    QSharedPointer<PointS> start = segment->get_start();
    QSharedPointer<PointS> end   = segment->get_end();
    QSharedPointer<PointS> mid  = cylinder->get_center_ptr();
    float dist_start_mid = SimpleMath<float>::get_distance(*start,*mid);
    float dist_end_mid = SimpleMath<float>::get_distance(*end,*mid);
    if((dist_start_mid || dist_end_mid) == 0)
    {
        qDebug() << "Tree::get_length_to_leave()  critical error, there seems to be zero length cylidners";
    } else {
        float ratio = dist_start_mid/(dist_start_mid+dist_end_mid);
        length_inside_segment = segment->get_length()*ratio;
    }
    length =   spherefollowing_length + attractor_length - length_inside_segment;
//    qDebug() << " Tree::get_length_to_leave(const QSharedPointer<Cylinder> cylinder) const" << spherefollowing_length << ";" << attractor_length << ";" << length_inside_segment;
    if(length <= 0)
    {
        length = length_inside_segment;
       // qDebug() << "critical problem in Tree::get_length_to_leave(const QSharedPointer<Segment> segment)";
    }
    return length;
}

const float Tree::get_length_to_leave_save(const QSharedPointer<Cylinder> cylinder) const
{
    float length = 0;
    QSharedPointer<Segment> segment = cylinder->get_segment();
    bool add_length = true;
    QVector<QSharedPointer<Segment> > segments = get_segments_longest_path(segment);
    QVectorIterator<QSharedPointer<Segment> > it(segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> seg = it.next();
        QVector<QSharedPointer<Cylinder> > cylinders = seg->get_cylinders();
        QVectorIterator<QSharedPointer<Cylinder> > git(cylinders);
        git.toBack();
        while(git.hasPrevious())
        {
            QSharedPointer<Cylinder> cyl = git.previous();
            if(add_length)
            {
                length += cyl->get_length();
                if(cyl == cylinder)
                {
                    add_length = false;
                }
            }
        }
    }
    return length;
}

const int Tree::get_branch_order_from_leave(const QSharedPointer<Cylinder> cylinder) const
{
    int branch_order = cylinder->get_segment()->get_branch_order();
    QSharedPointer<Segment> segment = cylinder->get_segment();
    QVector<QSharedPointer<Segment> > leaves = get_leave_segments(segment);
    int max_branch_order = branch_order;
    QVectorIterator<QSharedPointer<Segment> > it(leaves);
    while(it.hasNext())
    {
        QSharedPointer<Segment>  seg = it.next();
        if(seg->get_branch_order()>max_branch_order)
        {
            max_branch_order = seg->get_branch_order();
        }
    }

    return max_branch_order - branch_order;
}

const int Tree::get_branch_order_cumulative(const QSharedPointer<Segment> segment) const
{
    QVector<QSharedPointer<Segment> > children = segment->get_child_segments();
    if(children.size()==0)
    {
        return 1;
    } else {
        int bo = 1;
        QVectorIterator<QSharedPointer<Segment> > it (children);
        while(it.hasNext())
        {
            QSharedPointer<Segment> seg = it.next();
            bo += get_branch_order_cumulative(seg);
        }
        return bo;
    }
}

const QVector<QSharedPointer<Segment> > Tree::get_segments_longest_path(const QSharedPointer<Segment> base) const
{
    QVector<QSharedPointer<Segment> > longest_path_segments;
    QVector<QSharedPointer<Segment> > leaves = get_leave_segments(base);
    float max_length = 0;
    QVectorIterator<QSharedPointer<Segment> >  it(leaves);

    while(it.hasNext())
    {
        QSharedPointer<Segment> leave = it.next();
        QVector<QSharedPointer<Segment> > path_segments = get_segments_between(base,leave);
        float length = get_sum_length(path_segments);
        if(length>max_length)
        {
            max_length = length;
            longest_path_segments = path_segments;
        }
    }
    return longest_path_segments;

}

const QVector<QSharedPointer<Segment> > Tree::get_leave_segments(const QSharedPointer<Segment> base) const
{
    QVector<QSharedPointer<Segment> > segments = get_child_segments(base);
    QVector<QSharedPointer<Segment> > leave_segments;
    QVectorIterator<QSharedPointer<Segment> > it(segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        if(segment->is_leave())
        {
            leave_segments.push_back(segment);
        }
    }
    return leave_segments;
}

const float Tree::get_height(MethodCoefficients cf) const
{
    return (get_max_height()-get_min_height()+cf.cut_height);
}

const float Tree::get_length(MethodCoefficients cf) const
{
    float length = 0;
    QVector<QSharedPointer<Cylinder> > stem_cylinders = get_stem_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it (stem_cylinders);

    float length_spherefollowing = 0;
    PointS highest_point;
    PointS highest_spherefollowing_point;
    float heighest_z = std::numeric_limits<float>::lowest();
    bool count = true;
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        if(count)
        {
            length_spherefollowing += cylinder->get_length();
            highest_spherefollowing_point = cylinder->get_end();
        }


        if(cylinder->get_detection() != DetectionType::SPHEREFOLLOWING)
        {
            count = false;
        }
        if(heighest_z<cylinder->get_end().z)
        {
            heighest_z = cylinder->get_end().z;
            highest_point = cylinder->get_end();
        }

    }
    float length_attractor = SimpleMath<float>::get_distance(highest_spherefollowing_point, highest_point);
    length = cf.cut_height + length_spherefollowing + length_attractor;
    //  qDebug () << cf.cut_height << " ; " <<  length_spherefollowing << " ; " << length_attractor << ";;" << length;

    return length;

}

const float Tree::get_length_to_root(QSharedPointer<Cylinder> cylinder, MethodCoefficients cf)
{
    float length = 0;
    QVector<QSharedPointer<Cylinder> > path_cylinders;
    path_cylinders.push_back(cylinder);
    QSharedPointer<Cylinder> current = cylinder;
    while(get_parent(current) != current)
    {
        current = get_parent(current);
        path_cylinders.push_back(current);
    }
    QVectorIterator<QSharedPointer<Cylinder> > it (path_cylinders);
    float length_spherefollowing = 0;
    QSharedPointer<Cylinder> first_att = cylinder;
    QSharedPointer<Cylinder> last_att = cylinder;

    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        if(cylinder->get_detection() != DetectionType::ATTRACTOR)
        {
            length_spherefollowing += cylinder->get_length();
        }


        if(cylinder->get_detection() == DetectionType::ATTRACTOR)
        {
            last_att = cylinder;
        }
    }
    PointS first = first_att->get_end();
    PointS last = last_att->get_start();

    float length_attractor = SimpleMath<float>::get_distance(first, last);
    length = cf.cut_height + length_spherefollowing + length_attractor;
    return length;
}

const float Tree::get_volume() const
{
    QVector<QSharedPointer<Cylinder> > cylinders = get_all_cylinders();
    float volume = 0;
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        volume += cylinder->get_volume();
    }
    return volume;
}

const float Tree::get_growth_volume(const QSharedPointer<Cylinder> cylinder) const
{
    QVector<QSharedPointer<Cylinder> >  cylinders = get_child_cylinders(cylinder);
    float growth_volume = 0;
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        growth_volume += cylinder->get_volume();
    }
    return growth_volume;
}

float Tree::get_growth_length(QSharedPointer<Cylinder> cylinder)
{
    QVector<QSharedPointer<Cylinder> >  cylinders = get_child_cylinders(cylinder);
    float growth_length = 0;
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        growth_length += cylinder->get_length();
    }
    return growth_length;
}

const QVector<QSharedPointer<Cylinder> > Tree::get_child_cylinders(const QSharedPointer<Cylinder> base_cylinder) const
{
    QSharedPointer<Segment> base_segment = base_cylinder->get_segment();
    QVector<QSharedPointer<Segment> > segments = get_child_segments(base_segment);
    segments.removeAll(base_segment);

    QVector<QSharedPointer<Cylinder> > cylinders;
    QVector<QSharedPointer<Cylinder> > base_cylinders = base_segment->get_cylinders();
    int index = base_cylinders.indexOf(base_cylinder);
    if(index>=0)
    {
        for(index; index<base_cylinders.size(); index++)
        {
            cylinders.push_back(base_cylinders.at(index));
        }
    }

    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        cylinders.append(segment->get_cylinders());
    }
    return cylinders;
}


const QVector<QSharedPointer<Segment> > Tree::get_child_segments(const QSharedPointer<Segment> base_segment) const
{
    QVector<QSharedPointer<Segment> > segments;
    segments.push_back(base_segment);
    QVector<QSharedPointer<Segment> > children = base_segment->get_child_segments();
    QVectorIterator<QSharedPointer<Segment> > it (children);
    while(it.hasNext())
    {
        QSharedPointer<Segment> child = it.next();
        QVector<QSharedPointer<Segment> > children_next_generation = get_child_segments(child);
        segments.append(children_next_generation);
    }
    return segments;
}

const QVector<QSharedPointer<Cylinder> > Tree::get_all_cylinders() const
{
    QVector<QSharedPointer<Segment> > segments = get_all_segments();
    QVector<QSharedPointer<Cylinder> > cylinders;
    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        cylinders.append(segment->get_cylinders());
    }
    return cylinders;
}

const QVector<QSharedPointer<Segment> > Tree::get_all_segments() const
{
    return get_child_segments(_root_segment);
}

Tree::Tree(QSharedPointer<Segment> root_segment,  QString treeID)
{
    _root_segment = root_segment;
    _treeID = treeID;
}
