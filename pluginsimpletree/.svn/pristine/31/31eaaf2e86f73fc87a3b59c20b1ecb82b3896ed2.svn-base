#include "computeprincipaldirection.h"

void ComputePrincipalDirection::compute()
{
    pcl::KdTreeFLANN<PointS>::Ptr tree (new pcl::KdTreeFLANN<PointS>);
    tree->setInputCloud(_cloud_in);
    size_t size = _cloud_in->points.size();
    for(size_t i = 0; i < size; i++)
    {
        PointS query = _cloud_in->points.at(i);
        std::vector<int> indices;
        std::vector<float> squared_distances;
        if(tree->radiusSearch(query,_search_radius,indices,squared_distances))
        {
            Eigen::Matrix3f cov;
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid<PointS> (*_cloud_in, indices, centroid);
            pcl::computeCovarianceMatrix<PointS> (*_cloud_in, indices,cov);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen(cov);
            Eigen::Matrix3f eigen_vectors = eigen.eigenvectors();
            Eigen::Vector3f pd = eigen_vectors.col(1);
            pd.normalize();
            _cloud_in->points.at(i).normal_x = pd[0];
            _cloud_in->points.at(i).normal_y = pd[1];
            _cloud_in->points.at(i).normal_z = pd[2];
        }
    }
}

ComputePrincipalDirection::ComputePrincipalDirection(PointCloudS::Ptr cloud_in)
{
    _cloud_in = cloud_in;
    compute();
}
