#include "optimizationspherefollowingst.h"
QMutex OptimizationSphereFollowingST::lock;
float OptimizationSphereFollowingST::_best_distance = 1;
MethodCoefficients OptimizationSphereFollowingST::_coeff_end;
OptimizationSphereFollowingST::OptimizationSphereFollowingST(PointCloudS::Ptr cloud, MethodCoefficients coeff,  bool subdivide_stem_and_branch_points, bool is_multithreaded)
{
    _cloud = cloud;
    _coeff = coeff;
    _coeff_end = coeff;
    _subdivide_stem_and_branch_points = subdivide_stem_and_branch_points;
    _is_multithreaded = is_multithreaded;



}

QVector<MethodCoefficients> OptimizationSphereFollowingST::generate_coefficients()
{
    QVector<MethodCoefficients> coefficients;
    MethodCoefficients coeff_a = _coeff;
    for(int i = 0; i < 5; i++)
    {
        MethodCoefficients coeff_b = coeff_a;
        switch (i) {
        case 0:
        {
            coeff_b.sphere_radius_multiplier = 1.75;
            break;
        }
        case 1:
        {
            coeff_b.sphere_radius_multiplier = 1.8;
            break;
        }
        case 2:
        {
            coeff_b.sphere_radius_multiplier = 1.85;
            break;
        }
        case 3:
        {
            coeff_b.sphere_radius_multiplier = 1.9;
            break;
        }
        case 4:
        {
            coeff_b.sphere_radius_multiplier = 1.95;
            break;
        }

        default:
            break;
        }
        for(int j = 0; j < 3; j++)
        {
            MethodCoefficients coeff_c = coeff_b;
            switch (j) {
            case 0:
                coeff_c.epsilon_cluster_branch = coeff_b.epsilon_cluster_branch*0.75;
                break;
            case 1:
                coeff_c.epsilon_cluster_branch = coeff_b.epsilon_cluster_branch*1;
                break;
            case 2:
                coeff_c.epsilon_cluster_branch = coeff_b.epsilon_cluster_branch*1.25;
                break;
            default:
                break;
            }

            for(int k = 0; k < 4; k++)
            {
                MethodCoefficients coeff_d = coeff_c;
                switch (k) {
                case 0:
                    coeff_d.epsilon_sphere = 0.01;
                    break;
                case 1:
                    coeff_d.epsilon_sphere = 0.02;
                    break;
                case 2:
                    coeff_d.epsilon_sphere = 0.03;
                    break;
                case 3:
                    coeff_d.epsilon_sphere = 0.04;
                    break;

                default:
                    break;
                }
                coefficients.push_back(coeff_d);
                for(int l = 0; l < 3; l++)
                {
                    MethodCoefficients coeff_e = coeff_d;
                    switch (l) {
                    case 0:
                        coeff_e.epsilon_cluster_stem = 0.03;
                        break;
                    case 1:
                        coeff_e.epsilon_cluster_stem = 0.06;
                        break;
                    case 2:
                        coeff_e.epsilon_cluster_stem = 0.09;
                        break;

                    default:
                        break;
                    }
                    coefficients.push_back(coeff_e);
                }
            }
        }
    }
    return coefficients;
}

void OptimizationSphereFollowingST::update_coeff(MethodCoefficients coeff, float distance)
{
    lock.lock();
    if(_best_distance > distance)
    {
        _best_distance = distance;
        _coeff_end = coeff;
    }
    lock.unlock();
}

void OptimizationSphereFollowingST::optimize()
{
    _best_distance = 1;
    QVector<MethodCoefficients> coeff = generate_coefficients();
    QVectorIterator<MethodCoefficients> it (coeff);


    while(it.hasNext())
    {
        MethodCoefficients coefficients = it.next();

        SphereFollowing2 sf(coefficients, _cloud, _subdivide_stem_and_branch_points);
        sf.sphere_following();
        QVector<pcl::ModelCoefficients> coeff = sf.get_cylinders();
        ComputeDistanceCylindersCloud cd (coeff,_cloud,3.0f);
        float dist = cd.get_distance();
        update_coeff(_coeff, dist);
    }


}

