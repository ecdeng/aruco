#pragma once
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <boost/thread.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/Eigenvalues"
#include "EigenLM.hpp"

#include <rosbag/bag.h>

#include <cmath>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


using namespace Eigen;
using namespace std;

ros::Duration d(0.01);

typedef std::deque<geometry_msgs::PoseStamped> PoseVector;
typedef std::vector<Affine3f> Affine3fVector;
enum METHOD { OLD,NEW,WHOLE };

class Listener
{

public:
    // parameters need to be dynamicly adjusted

    // = time_aruco - time_opti(from the realy world to received in this thread)
    // used when pub optiRef
    float m_ArucoDelayCompensate;  // add to opti time
    METHOD m_methodUesd;


private:
    //
    Eigen::Vector3f m_P_aruco, m_P_opti, m_P_arFromOpti;

    geometry_msgs::PoseStamped m_Pose_aruco, m_Pose_opti, m_RefPose,m_Pose_err;
    PoseVector m_OptiPose_V,m_RefPose_V;
    const uint m_RefPosV_N = 120 * 0.2;  // the number stored in RefPose_V
    Affine3f m_T_r2c, m_T_w2b;
    Affine3f m_T_r2w, m_T_w2r, m_T_b2c;
    Affine3f m_T_r2c_Ref;
    Affine3f m_T_r2c_err;

    typedef  Matrix<float,4,8> Matrix48f;
    Matrix<float,4,8> X_r,X_w;


    ros::NodeHandle n;
    ros::Publisher opti_PoseRef_pub;
    ros::Publisher PoseErr_pub,ErrEuler_pub,optiEuler_pub,RefEuler_pub,arucoEuler_pub,RefVelocity_pub ;
    rosbag::Bag m_bag;
    const string m_InstallCalibResultPath = "Aruco_InstallCalibResult.bag";
    const string m_InstallCalibInputPath = "InstallCalibInput.bag";
    const string m_InCal_opti_Topic = "m_Pose_opti";
    const string m_InCal_aruco_Topic = "m_Pose_aruco";

    Vector3i m_aruco_eulerOrder,m_opti_eulerOrder;

    float m_aruco_ROSdelay,m_opti_ROSdelay;     // delay from ros pub to sub



  void CalDataDelay(float& optiDelay, float& arucoDelay);
  bool IsOptiNew(float& optiDelay);
  bool IsArucoNew(float& arucoDelay);

  float Cal_PoseErr();
  int FindClosestTime(PoseVector& PoseV, ros::Time,float&);
  void Cal_OptiRef();

  void Pose_to_Affine(geometry_msgs::PoseStamped Pose_ros_a2b, Affine3f& T_eigen_a2b);
  void Affine_to_Pose(const Affine3f& T_eigen_a2b,geometry_msgs::PoseStamped& Pose_ros_a2b);

  void Calculate_Pr2w(Eigen::Affine3f& Out_T_r2w);
  bool DoInstallCalib(bool IsLoadFromFile);
  void Cal_Install_Old(const int& N_points,const Affine3fVector& T_w2b_Array, const Affine3fVector& T_r2c_Array, Affine3f& Out_T_b2c, Affine3f& Out_T_r2w);
  void Cal_Install_New(const int& N_points,const Affine3fVector& T_w2b_Array, const Affine3fVector& T_r2c_Array, Affine3f& Out_T_b2c, Affine3f& Out_T_r2w);
  bool LoadBag_InstallCalibData( int& N_points,std::vector<Affine3f>& T_w2b_Array,std::vector<Affine3f>& T_r2c_Array);
  void LoadBag_CalErr(const string Path);
  bool GetInstallCalibData_Online( int N_points,std::vector<Affine3f>& T_w2b_Array,std::vector<Affine3f>& T_r2c_Array);
  void CheckCalibErr( const std::vector<Affine3f>& T_w2b_Array, const std::vector<Affine3f>& T_r2c_Array,Affine3f& In_T_b2c,Affine3f& In_T_r2w );

  bool ReadBag_Pose(const string Path, const string topic, std::vector<geometry_msgs::PoseStamped>& PoseVec);
  bool BagRead_InstallCalib(METHOD methodUsed);
  void InitInstallCalib();


  void UpdateInstallCalibBagFile(Affine3f* Out_T_b2c_p,Affine3f* Out_T_r2w_p,int N);
  void WriteBag_T(const Affine3f& T, const string name);
  void AppendBag_T(const Affine3f& T,const string name);

  void GetQM( Quaternionf Q,Matrix4f& M );
  void GetQMt( Quaternionf Q,Matrix4f& Mt );
  void NormalizeQ(Quaternionf &Q);

  void NormalizeAffine(Affine3f& T);

  void PoseMsg_to_EulerMsg(geometry_msgs::PoseStamped pose, geometry_msgs::Vector3Stamped& euler,Vector3i EulerOrder,string eulerFrameid);

  void modifyRosTime(ros::Time& Time,float addSec);



public:

  Listener();
  void chatter_aruco_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void chatter_opti_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void chatter_calibInstall(const std_msgs::Float32ConstPtr& msg)
  {
    cout << "received calib command" << endl;
    float d = msg->data;

    bool IsLoadFromFile = true;
    if(d<0)
    {
        IsLoadFromFile = false; // on line
        m_methodUesd = (METHOD) (-msg->data-1);
    }
    else
    {
        m_methodUesd = (METHOD) (msg->data-1);
    }
    cout << "(chatter_calibInstall) : m_methodUesd = " << m_methodUesd << endl;
    cout << "(chatter_calibInstall) : IsLoadFromFile = " << IsLoadFromFile << endl;

    this->DoInstallCalib(IsLoadFromFile);
  }
  void chatter_arucoDelayAdjust(const std_msgs::Float32ConstPtr& msg)
  {
      m_ArucoDelayCompensate = msg->data;
      cout<<"in chatter_arucoDelayAdjust " << endl << "m_ArucoDelayCompensate = " << m_ArucoDelayCompensate << endl;
  }



};
