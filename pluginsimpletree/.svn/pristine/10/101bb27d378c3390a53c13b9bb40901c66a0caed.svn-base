#include "helperformt.h"

HelperForMT::HelperForMT(QVector<SimpleTreeMT> st_mt_vec, ST_StepAbstractModellingMT *step)
{

    int numberThreads = QThread::idealThreadCount()-1;
    this->setMaxThreadCount(std::max<int>(numberThreads,1));
    _st_mt_vec = st_mt_vec;
    _step = step;



}

void HelperForMT::start_mt()
{
    size_t size = _st_mt_vec.size();

    for (int i = 0; i < size; i++)
    {
       // qDebug() << "helperstart";
        MethodCoefficients coeff;
        ComputeMeanAndStandardDeviation m_sd(_st_mt_vec[i].cloud);
        coeff.id = _st_mt_vec[i].file_coeff.file;
        coeff.species = _st_mt_vec[i].file_coeff.species;

        coeff.sd = m_sd._sd;
        coeff.mean = m_sd._mean;
        coeff.epsilon_cluster_branch = 3*(m_sd._mean + 2*m_sd._sd);

        if(_st_mt_vec[i].percent>05&&_st_mt_vec[i].percent<90)
        {
            coeff.optimze_stem = true;
        }

        float z_min = std::numeric_limits<float>::max();
        float z_max = std::numeric_limits<float>::lowest();
        for(size_t j = 0; j < _st_mt_vec[i].cloud->points.size(); j++)
        {
            PointS p = _st_mt_vec[i].cloud->points.at(j);
            if(p.z < z_min)
            {
                z_min = p.z;
            }

            if(p.z > z_max)
            {
                z_max = p.z;
            }
        }

        float height = z_max - z_min;


//        if(height > 10)
//        {
//            coeff.epsilon_sphere = 0.03;
//            coeff.allometry_b = 3;
//        } else
//        {
//            coeff.optimze_stem = false;
//            coeff.allometry_b = 2;
//        }
        PredictStableVolume pv (_st_mt_vec[i].cloud, coeff);
        coeff = pv.get_coeff();
        WorkerStepMT * worker (new WorkerStepMT(_st_mt_vec[i].cloud,coeff,true,i,_step));
        start(worker);

    }
    waitForDone();
}
