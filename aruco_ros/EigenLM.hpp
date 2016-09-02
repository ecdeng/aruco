#pragma once
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <unsupported/Eigen/NonLinearOptimization>
//#include <unsupported/Eigen/LevenbergMarquardt>

// LM minimize for the model y = a w + b x + c y + d z
//typedef std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d> > Point4DVector;
typedef std::vector<Eigen::Vector4d > Point4DVector;
typedef std::vector<Eigen::Vector3d > Point3DVector;
typedef std::vector<Eigen::Matrix3d > Matrix3DVector;
typedef std::vector<Eigen::Matrix4d > Matrix4DVector;
typedef std::vector<Eigen::Affine3d > Affine3DVector;
typedef std::vector<Eigen::Affine3f > Affine3fVector;

typedef std::vector<Eigen::Vector3d > Vector3dVector;
typedef std::vector<Eigen::Quaterniond > QuaterniondVector;

typedef Eigen::Matrix<double,7,1> Vector7d ;
typedef std::vector<Vector7d > Vector7dVector;

// Generic functor
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
typedef _Scalar Scalar;
enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
};
typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

int m_inputs, m_values;

Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

int inputs() const { return m_inputs; }
int values() const { return m_values; }

};

void NormalizeQ(Eigen::Quaterniond &Q);
void Affine3d_From_Qt( Eigen::Affine3d& T_a2b, Eigen::Quaterniond Q_a2b,Eigen::Vector3d t_a2b );
void Affine3d_To_Vector( const Eigen::Affine3d& T,Eigen::Matrix<double,16,1>& V );
void Vector_To_Affine3d(  const Eigen::VectorXd& V ,Eigen::Affine3d& T);

struct Q_bc_Functor : Functor<double>
{
  int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
  {
      // 0  = w * p[0] + x * p[1] + y * p[2] + z * p[3]
    for(unsigned int i = 0; i < this->Mct_sub_Mb.size(); ++i)
      {
      fvec(i) = this->Mct_sub_Mb[i](0) * x(0) + this->Mct_sub_Mb[i](1) * x(1) +
              this->Mct_sub_Mb[i](2) * x(2) + this->Mct_sub_Mb[i](3) * x(3) +
              (pow( x(0),2 ) + pow( x(1),2 ) + pow( x(2),2 ) + pow( x(3),2 ) - 1)*1;
      }
    return 0;
  }

  Point4DVector Mct_sub_Mb;

  int inputs() const { return 2; } // There are two parameters of the model
  int values() const { return this->Mct_sub_Mb.size(); } // The number of observations
};

struct Q_bc_FunctorNumericalDiff : Eigen::NumericalDiff<Q_bc_Functor> {};

struct T_bc_Functor : Functor<double>
{
    Eigen::VectorXd t_temp_N;
    Eigen::MatrixXd R_temp_N;

    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
    {
        fvec = t_temp_N - R_temp_N * x;
        return 0;
    }
    int inputs() const { return 2; } // There are two parameters of the model
    int values() const { return t_temp_N.rows(); } // The number of observations
};
struct T_bc_FunctorNumericalDiff : Eigen::NumericalDiff<T_bc_Functor> {};

Point4DVector GeneratePoints(const Eigen::MatrixXd M);


struct T_rw_Functor : Functor<double>
{
    QuaterniondVector Q_r2w_V;
    Vector3dVector t_r2w_V;
    int operator()(const Eigen::VectorXd &Q_t_r2w, Eigen::VectorXd &fvec) const
    {
        for(int i=0;i<Q_r2w_V.size();i++)
        {
            fvec(i*7+0) = Q_r2w_V.at(i).x() - Q_t_r2w(0);
            fvec(i*7+1) = Q_r2w_V.at(i).y() - Q_t_r2w(1);
            fvec(i*7+2) = Q_r2w_V.at(i).z() - Q_t_r2w(2);
            fvec(i*7+3) = Q_r2w_V.at(i).w() - Q_t_r2w(3);


            for(int j=0;j<3;j++)
            {
                fvec(i*7+4+j) = t_r2w_V.at(i)(j) - Q_t_r2w(j+4);
            }
        }
    }
    int inputs() const { return 2; } // There are two parameters of the model
    int values() const { return Q_r2w_V.size()*7; } // The number of observations
};
struct T_rw_FunctorNumericalDiff : Eigen::NumericalDiff<T_rw_Functor> {};

struct InCal_Functor : Functor<double>
{
    Affine3DVector T_w2b_V,T_r2c_V;
    int operator()(const Eigen::VectorXd &T_In, Eigen::VectorXd &fvec) const
    {
//        Eigen::Quaterniond Q_r2w( T_In(3),T_In(0),T_In(1),T_In(2) ) ,Q_b2c( T_In(3+7),T_In(0+7),T_In(1+7),T_In(2+7) );
//        Eigen::Vector3d t_r2w( T_In(4),T_In(5),T_In(6) ) ,t_b2c( T_In(4+7),T_In(5+7),T_In(6+7) );

        Eigen::Affine3d T_r2w,T_c2b;
//        Affine3d_From_Qt(T_r2w,Q_r2w,t_r2w);
//        Affine3d_From_Qt(T_b2c,Q_b2c,t_b2c);
        Eigen::VectorXd V_r2w,V_c2b;
        V_r2w = T_In.block<16,1>(0,0);
        V_c2b = T_In.block<16,1>(16,0);

        Vector_To_Affine3d(V_r2w,T_r2w);
        Vector_To_Affine3d(V_c2b,T_c2b);


        for(int i=0;i<T_w2b_V.size();i++)
        {
            Eigen::Affine3d T_r2b_1 = T_c2b * T_r2c_V.at(i);
            Eigen::Affine3d T_r2b_2 = T_w2b_V.at(i) * T_r2w;

            Eigen::Matrix4d T_err = T_r2b_1.matrix() - T_r2b_2.matrix();

//            Eigen::Affine3d T_r2r =  T_r2b_1.inverse() * T_r2b_2;
//            Eigen::Matrix4d T_err = T_r2r.matrix() - Eigen::Affine3d::Identity().matrix();

            for(int j=0;j<4;j++)
            {
                for(int k=0;k<4;k++)
                {
                    fvec(i*16+j*4+k) = T_err(j,k);
                }
            }
        }
    }
    int inputs() const { return 2; } // There are two parameters of the model
    int values() const { return T_w2b_V.size()*16; } // The number of observations
};
struct Incal_FunctorNumericalDiff : Eigen::NumericalDiff<InCal_Functor> {};



int Cal_Q_bc(Eigen::MatrixXf M,Eigen::VectorXf& xf);
int Cal_T_bc(Eigen::VectorXf t_temp_N,Eigen::MatrixXf R_temp_N,Eigen::VectorXf& t_b2c);
int InstallCalib_LM( Eigen::Affine3f &T_b2c,  Eigen::Affine3f &T_r2w, Affine3fVector& T_r2c_V,Affine3fVector& T_w2b_V);
int Cal_T_rw(Eigen::Affine3f& T_r2w,Affine3fVector& T_r2w_V_f);

