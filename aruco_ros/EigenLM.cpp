#include "EigenLM.hpp"


Point4DVector GeneratePoints(const Eigen::MatrixXd M)
{
  Point4DVector points;
  // Model y = 2*x + 5 with some noise (meaning that the resulting minimization should be about (2,5)
  int n = M.rows();
  for(unsigned int i = 0; i < n; ++i)
    {
    Eigen::Vector4d point = (Eigen::Vector4d)M.row(i);
    points.push_back(point);
    }

  return points;
}



int Cal_Q_bc(Eigen::MatrixXf Mct_sub_Mb_f,Eigen::VectorXf& Q_b2c)
{
  Eigen::MatrixXd Md = Mct_sub_Mb_f.cast<double>() ;

  Point4DVector Mct_sub_Mb = GeneratePoints(Md);

  Eigen::VectorXd x = Q_b2c.cast<double>();

  Q_bc_FunctorNumericalDiff Q_bc_functor;
  Q_bc_functor.Mct_sub_Mb = Mct_sub_Mb;
  Eigen::LevenbergMarquardt<Q_bc_FunctorNumericalDiff> lm(Q_bc_functor);
  lm.parameters.maxfev = 3000;
  lm.parameters.xtol = 1.0e-9;

  Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(x);
  x = x/x.norm();
  std::cout << "status: " << status << std::endl;
  //std::cout << "info: " << lm.info() << std::endl;
  std::cout << "x that minimizes the function: " << std::endl << x << std::endl;

  Q_b2c = x.cast<float>();

  return 0;
}

int Cal_T_bc(Eigen::VectorXf t_temp_N,Eigen::MatrixXf R_temp_N,Eigen::VectorXf& t_b2c)
{


    T_bc_FunctorNumericalDiff T_bc_functor;
    T_bc_functor.t_temp_N = t_temp_N.cast<double>();
    T_bc_functor.R_temp_N = R_temp_N.cast<double>();
    Eigen::LevenbergMarquardt<T_bc_FunctorNumericalDiff> lm(T_bc_functor);
    lm.parameters.maxfev = 3000;
    lm.parameters.xtol = 1.0e-9;
    Eigen::VectorXd t_b2c_d;
    t_b2c_d.resize(3);
    t_b2c_d.fill(0.2);
    Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(t_b2c_d);
//    std::cout << "status: " << status << std::endl;

//    std::cout << "T_bc_functor.t_temp_N = " << std::endl << T_bc_functor.t_temp_N << std::endl;

//    std::cout << "t_b2c_d = " << std::endl << t_b2c_d << std::endl;
//    std::cout << "info: " << lm.info() << std::endl;

    t_b2c = t_b2c_d.cast<float>();
    return 0;
}


int Cal_T_rw(Eigen::Affine3f& T_r2w,Affine3fVector& T_r2w_V_f)
{

    T_rw_FunctorNumericalDiff T_rw_functor;

    Eigen::Quaterniond Q_r2w_i;
    Eigen::Vector3d t_r2w_i;
    int N = T_r2w_V_f.size();

    Eigen::Affine3d T_r2w_i;
    for(int i=0;i<N;i++)
    {
        T_r2w_i = T_r2w_V_f.at(i).cast<double>();
//        std::cout << "T_r2w_i" << std::endl<< T_r2w_i.matrix() << std::endl;
        Eigen::Matrix3d R_w2r_i = T_r2w_i.linear().inverse();
        Q_r2w_i = Eigen::Quaterniond ( R_w2r_i );
        NormalizeQ(Q_r2w_i);
        t_r2w_i = T_r2w_i.translation();
        T_rw_functor.Q_r2w_V.push_back(Q_r2w_i);
        T_rw_functor.t_r2w_V.push_back(t_r2w_i);

//        std::cout<<"  Q_r2w_i = " << std::endl << Q_r2w_i.coeffs() << std::endl;
    }

    Eigen::LevenbergMarquardt<T_rw_FunctorNumericalDiff> lm(T_rw_functor);
    lm.parameters.maxfev = 3000;
    lm.parameters.xtol = 1.0e-10;

    Eigen::VectorXd Q_t_r2w;
    Q_t_r2w.resize(7);
    Q_t_r2w.fill(0);
    Q_t_r2w(3)=1;

    Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(Q_t_r2w);

//    std::cout << "Q_t_r2w = " << std::endl << Q_t_r2w << std::endl;

    Eigen::Quaterniond Q_r2w(Q_t_r2w(3),Q_t_r2w(0),Q_t_r2w(1),Q_t_r2w(2));
    Q_r2w.normalize();
    Eigen::Quaternionf Q_r2w_f = Q_r2w.cast<float>();
    Eigen::Matrix3f R_r2w(Q_r2w_f.inverse());
    T_r2w.linear() = R_r2w;
    T_r2w.translation() = Q_t_r2w.block<3,1>(4,0).cast<float>();
    std::cout << "status: " << status << std::endl;

//    std::cout << "LM  Cal_T_rw = " << std::endl;
    std::cout << "Q_r2w_f = "  << Q_r2w_f.coeffs().transpose() << std::endl;
    std::cout << "T_r2w = " << std::endl << T_r2w.matrix() << std::endl;
//    std::cout << "info: " << lm.info() << std::endl;

    return 0;
}
void NormalizeQ(Eigen::Quaterniond &Q)
{
    Q.normalize();
    for(int i=0;i<4;i++)
    {
        Q.coeffs().coeffRef(i) *=-1;
    }
}

void Affine3d_From_Qt( Eigen::Affine3d& T_a2b, Eigen::Quaterniond Q_a2b,Eigen::Vector3d t_a2b )
{
    T_a2b.translation() = t_a2b;
    Eigen::Matrix3d R_a2b(Q_a2b.inverse());
    T_a2b.linear() = R_a2b;
}

int InstallCalib_LM( Eigen::Affine3f &T_b2c,  Eigen::Affine3f &T_r2w, Affine3fVector& T_r2c_V,Affine3fVector& T_w2b_V)
{
    Incal_FunctorNumericalDiff IC_functor;

    for(int i=0;i<T_w2b_V.size();i++)
    {
        Eigen::Affine3d T_r2c_i = T_r2c_V.at(i).cast<double>();
        IC_functor.T_r2c_V.push_back(T_r2c_i);

        Eigen::Affine3d T_w2b_i = T_w2b_V.at(i).cast<double>();
        IC_functor.T_w2b_V.push_back(T_w2b_i);
    }

    Eigen::LevenbergMarquardt<Incal_FunctorNumericalDiff> lm(IC_functor);
    lm.parameters.maxfev = 10000;
    lm.parameters.xtol = 1.0e-9;

    Eigen::VectorXd T_In;

    T_In.resize(32);
    T_In.fill(0);
    T_In(0) = 1;
    T_In(1+4) = 1;
    T_In(2+4*2) = 1;
    T_In(3+4*3) = 1;

    T_In(0 + 16) = 1;
    T_In(1+4 + 16) = 1;
    T_In(2+4*2 + 16) = 1;
    T_In(3+4*3 + 16) = 1;


    Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(T_In);

    Eigen::VectorXd V_r2w,V_c2b;
    V_r2w = T_In.block<16,1>(0,0);
    V_c2b = T_In.block<16,1>(16,0);

    Eigen::Affine3d O_T_c2b,O_T_r2w;
    Vector_To_Affine3d(V_r2w,O_T_r2w);
    Vector_To_Affine3d(V_c2b,O_T_c2b);

    T_r2w = O_T_r2w.cast<float>();
    T_b2c = O_T_c2b.inverse().cast<float>();

    std::cout << "T_r2w = " << std::endl << O_T_r2w.matrix() << std::endl;
//    std::cout << "T_c2b = " << std::endl << O_T_c2b.matrix() << std::endl;
    std::cout << "T_b2c = " << std::endl << T_b2c.matrix() << std::endl;

    std::cout << std::endl;
}

void Affine3d_To_Vector( const Eigen::Affine3d& T,Eigen::VectorXd& V )
{
    Eigen::Matrix4d M = T.matrix();
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            V(i*4+j) = M(i,j);
        }
    }
}
void Vector_To_Affine3d(  const Eigen::VectorXd& V ,Eigen::Affine3d& T)
{
    Eigen::Matrix4d M;
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            M(i,j) = V(i*4+j) ;
        }
    }
    T.matrix() = M;
}
