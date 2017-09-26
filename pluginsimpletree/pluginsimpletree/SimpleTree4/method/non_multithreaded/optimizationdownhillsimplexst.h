#ifndef OPTIMIZATIONDOWNHILLSIMPLEXST_H
#define OPTIMIZATIONDOWNHILLSIMPLEXST_H

#include <QVector>
#include <QGenericMatrix>
#include <QMutexLocker>
#include <QMutex>
#include <QEnableSharedFromThis>

#include <SimpleTree4/method/spherefollowing2.h>
#include <SimpleTree4/method/non_multithreaded/spherefollowingrecursivest.h>
#include <SimpleTree4/model/tree.h>
#include "SimpleTree4/method/computedistancecylinderscloud.h"
#include <SimpleTree4/method/spherefollowing2.h>

#include <Eigen/Core>

class SphereFollowingRecursiveST;


class OptimizationDownHillSimplexST:  public QObject, public QEnableSharedFromThis<OptimizationDownHillSimplexST>
{
    Q_OBJECT

    const int _NMAX = 200;

    const double TINY = 1.0e-10;

    MethodCoefficients _start;

    MethodCoefficients _end;

    MethodCoefficients _copy;

    PointCloudS::Ptr _cloud;

    double _distance_start;

    double _distance_end;

    bool _subdivide_stem_and_branch_points;

    MethodCoefficients convert_to_coefficients(Eigen::MatrixXd const &matrix, int i);

    const double ftol;

    int nfunc = 0;

    int _mpts;

    int _ndim;

    double fmin;

    QVector<double> y;

    Eigen::MatrixXd _matrix;

    void generate_coeff_from_point(QVector<double> &coeff);

    void evaluate_multithreaded(Eigen::MatrixXd &matrix);

    bool _is_multithreaded = true;

    QVector<double> _results;

    QVector<double> minimize(QVector<double> &point, QVector<double> &factors);

    QVector<double> minimize(MethodCoefficients coeff, const double del);

    QVector<double> minimize(Eigen::MatrixXd &matrix_copy);

    double evaluate(QVector<double> x);

    void swap(double x1, double x2); //check this

    void get_psum(Eigen::MatrixXd & matrix, QVector<double> &psum);

    double amotry(Eigen::MatrixXd & matrix, QVector<double> &y, QVector<double> &psum, const int ihi, const double fac);

    QString get_qstring_from_eigen(Eigen::MatrixXd &matrix_copy);

    QString get_qstring_from_vector(QVector<double> &vec);

    MethodCoefficients get_coefficients_from_matrix(Eigen::MatrixXd &matrix, int i);

signals:

    void emit_qstring(QString qstring);

    void emit_counter(int timer);


public slots:

    void sent_qstring(QString qstring);

    void sent_counter(int counter);

public:

    OptimizationDownHillSimplexST(const double ftoll, MethodCoefficients coeff, PointCloudS::Ptr cloud,
                                bool subdivide_stem_and_branch_points, bool is_multithreaded = true);

    MethodCoefficients get_end_coeff() const;

    void minimize();

    void set_distance(int i, double d);

    void set_ndim(const int dim);


};

#endif // OPTIMIZATIONDOWNHILLSIMPLEXST_H
