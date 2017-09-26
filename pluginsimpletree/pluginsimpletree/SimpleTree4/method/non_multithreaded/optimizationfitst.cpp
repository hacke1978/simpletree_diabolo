#include "optimizationfitst.h"

QMutex OptimizationFitST::lock;

QSharedPointer<Tree> OptimizationFitST::_tree;

float OptimizationFitST::_best_distance = 1;

MethodCoefficients OptimizationFitST::_coeff_end;

QVector<MethodCoefficients> OptimizationFitST::generate_coefficients()
{
    QVector<MethodCoefficients> coefficients;
    MethodCoefficients coeff_a = _coeff;
    for(int i = 0; i < 7; i++)
    {
        MethodCoefficients coeff_b = coeff_a;
        switch (i) {
        case 0:
        {
            coeff_b.ransac_type = pcl::SAC_LMEDS;
            break;
        }
        case 1:
        {
            coeff_b.ransac_type = pcl::SAC_MLESAC;
            break;
        }
        case 2:
        {
            coeff_b.ransac_type = pcl::SAC_MSAC;
            break;
        }
        case 3:
        {
            coeff_b.ransac_type = pcl::SAC_PROSAC;
            break;
        }
        case 4:
        {
            coeff_b.ransac_type = pcl::SAC_RANSAC;
            break;
        }
        case 5:
        {
            coeff_b.ransac_type = pcl::SAC_RMSAC;
            break;
        }
        case 6:
        {
            coeff_b.ransac_type = pcl::SAC_RRANSAC;
            break;
        }
        default:
            break;
        }
        for(int j = 0; j < 5; j++)
        {
            MethodCoefficients coeff_c = coeff_b;
            switch (j) {
            case 0:
                coeff_c.ransac_inlier_distance = 0.01;
                break;
            case 1:
                coeff_c.ransac_inlier_distance = 0.02;
                break;
            case 2:
                coeff_c.ransac_inlier_distance = 0.03;
                break;
            case 3:
                coeff_c.ransac_inlier_distance = 0.04;
                break;
            case 4:
                coeff_c.ransac_inlier_distance = 0.05;
                break;
            default:
                break;
            }
            coefficients.push_back(coeff_c);

        }
    }
    return coefficients;

}

OptimizationFitST::OptimizationFitST(PointCloudS::Ptr cloud, MethodCoefficients coeff,  QVector<pcl::ModelCoefficients> cylinder_coef, bool is_multithreaded)
{
    _cylinder_coeff = cylinder_coef;
    _cloud = cloud;
    _coeff = coeff;
    _coeff_end = coeff;
    _is_multithreaded = is_multithreaded;

}

void OptimizationFitST::update_coeff(MethodCoefficients coeff, float distance, QSharedPointer<Tree> tree)
{
    lock.lock();
    if(_best_distance > distance)
    {
        _best_distance = distance;
        _coeff_end = coeff;
        _tree = tree;
    }
    lock.unlock();
}

void OptimizationFitST::optimize()
{
    _best_distance = 1;
    QVector<MethodCoefficients> coeff = generate_coefficients();
    QVectorIterator<MethodCoefficients> it (coeff);

    while(it.hasNext())
    {
        MethodCoefficients coefficients = it.next();
        BuildTree builder(_cylinder_coeff);
        QSharedPointer<Tree> tree (new Tree(builder.getRoot_segment()));
        RemoveFalseCylinders remove(tree);
        ImproveByMedian improve_by_median(tree);
        ImproveByMerge improve_merge(tree);
        ImproveByPipeModel pype(tree);
        ImproveFit fit(tree,_cloud,coefficients);



        QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
        QVector<pcl::ModelCoefficients> cylinder_coefficients;
        QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
        while(it.hasNext())
        {
            QSharedPointer<Cylinder> cylinder = it.next();
            pcl::ModelCoefficients coefficients;
            coefficients.values = cylinder->values;
            cylinder_coefficients.push_back(coefficients);
        }

        ComputeDistanceCylindersCloud cd (cylinder_coefficients,_cloud);
        float dist = cd.get_distance();
        update_coeff(_coeff, dist,tree);
    }

}
