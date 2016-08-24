/*

 */

#include "aruco_ros.hpp"
/**
 *
 */



int test();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_node");

  ros::NodeHandle n;

  // publish calib message

  Listener l;

  ros::Subscriber aruco_pose_sub = n.subscribe("aruco_pose_topic", 2, &Listener::chatter_aruco_pose, &l);
  ros::Subscriber opti_pose_sub = n.subscribe("opti_pose_topic", 1, &Listener::chatter_opti_pose, &l);
  ros::Subscriber calibInstall_sub = n.subscribe("calibInstall_topic",1,&Listener::chatter_calibInstall, &l);
  ros::Subscriber arucoDelayAdjust_sub = n.subscribe("arucoDelay_topic",1,&Listener::chatter_arucoDelayAdjust, &l);
  /**
   * The MultiThreadedSpinner object allows you to specify a number of threads to use
   * to call callbacks.  If no explicit # is specified, it will use the # of hardware
   * threads available on your system.  Here we explicitly specify 4 threads.
   */
  ros::MultiThreadedSpinner s(6);
  ros::spin(s);

  return 0;
}

void Listener::chatter_aruco_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
//    m_aruco_delay = ros::Time::now().toSec() - msg->header.stamp.toSec();
//    ROS_INFO("m_aruco_delay = %0.2f ms",m_aruco_delay*1000 );

    m_Pose_aruco.pose = msg->pose;
    m_Pose_aruco.header = msg->header;

//     float optiDelay, arucoDelay;
//     CalDataDelay(optiDelay,  arucoDelay);

//     cout << "arucoDelay = " << arucoDelay << endl;

    Cal_PoseErr();
//      ROS_INFO_STREAM("chatter_aruco_pose: [" << msg->pose.position << "] [thread=" << boost::this_thread::get_id() << "]");
    d.sleep();
}

void Listener::chatter_opti_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
//    m_opti_delay = ros::Time::now().toSec() - msg->header.stamp.toSec();
//    ROS_INFO("m_opti_delay = %0.2f ms",m_opti_delay*1000 );

    m_Pose_opti.pose = msg->pose;
    m_Pose_opti.header = msg->header;

//     float optiDelay = ros::Time::now().toSec() - msg->header.stamp.toSec();
//    cout << "optiDelay = " << optiDelay << endl;

    Cal_OptiRef();

    // pub euler
    geometry_msgs::Vector3Stamped m_Euler_opti;
    PoseMsg_to_EulerMsg(m_Pose_opti,m_Euler_opti,m_opti_eulerOrder,"m_Pose_opti");
    optiEuler_pub.publish(m_Euler_opti);

//     ROS_INFO_STREAM("chatter_opti_pose: [" << msg->pose.position << "] [thread=" << boost::this_thread::get_id() << "]");
    d.sleep();
}

Listener::Listener()
{
    m_ArucoDelayCompensate = 0;
    m_aruco_eulerOrder = Vector3i(1,0,2);
    m_opti_eulerOrder = Vector3i(1,0,2);

    opti_PoseRef_pub = n.advertise<geometry_msgs::PoseStamped>("optiRef_pose_topic", 1000);
    PoseErr_pub = n.advertise<geometry_msgs::PoseStamped>("poseErr_topic", 1000);
    ErrEuler_pub = n.advertise<geometry_msgs::Vector3Stamped>("ErrEuler_topic",100);
    optiEuler_pub = n.advertise<geometry_msgs::Vector3Stamped>("optiEuler_topic",100);

    if(!BagRead_InstallCalib())
        InitInstallCalib();

 //   LoadBag_CalErr(m_InstallCalibInputPath);
}

void Listener::InitInstallCalib()
{
    Calculate_Pr2w();
    m_T_b2c.matrix() <<
                      -0.687401,   0.0893974,    0.720755,   0.0397646,
                       0.132359,   -0.960356,     0.24535, -0.00888724,
                       0.714115,    0.264052,    0.648318,   0.0183151,
                              0,           0,           0,           1;
//    -0.699864  0.0847729   0.709228  0.0392641
//     0.121782  -0.964232   0.235427 -0.0144778
//     0.703818   0.251138   0.664507  0.0158802
//            0          0          0          1

//    -0.699851   0.121815   0.703826  0.0180695
//    0.0847826  -0.964219   0.251186 -0.0212858
//      0.70924   0.235465   0.664481 -0.0350038
//            0          0          0          1
    UpdateInstallCalibBagFile();
}
bool Listener::ReadBag_Pose(const string Path, const string topic, std::vector<geometry_msgs::PoseStamped>& PoseVec)
{
    char str[200];
    rosbag::Bag bag;
    try{
    bag.open(Path, rosbag::bagmode::Read);
    }
    catch(std::exception &ex)
    {
        sprintf(str,"Cannot open %s\n",Path.c_str());
        cerr << str << endl;
        cout << "Exception :" << ex.what() << endl;
        return false;
    }

    std::vector<std::string> topics;
    topics.push_back(topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    geometry_msgs::PoseStamped::ConstPtr posePtr;
    int  i = 0;
    foreach(rosbag::MessageInstance const m, view)
    {
        posePtr = m.instantiate<geometry_msgs::PoseStamped>();
        if (posePtr != NULL)
        {
            PoseVec.push_back(*posePtr);
            i++;
        }
    }
    bag.close();

//    int N = PoseVec.size();
//    for(int j=0;j<N;j++)
//    {
//        cout  << PoseVec.at(j) << endl;
//    }

    return true;
}

bool Listener::BagRead_InstallCalib()
{
    try{
    m_bag.open(m_InstallCalibResultPath, rosbag::bagmode::Read);
    }
    catch(std::exception &ex)
    {
        cerr << "Cannot open Aruco_InstallCalibResult.bag" << endl;
        cout << "Exception :" << ex.what() << endl;
        return false;
    }

    std::vector<std::string> topics;
    topics.push_back(std::string("m_T_r2w"));
    rosbag::View view(m_bag, rosbag::TopicQuery(topics));
    geometry_msgs::PoseStamped::ConstPtr posePtr;
    int  i_m_T_r2w = 0;
    foreach(rosbag::MessageInstance const m, view)
    {
        posePtr = m.instantiate<geometry_msgs::PoseStamped>();
        if (posePtr != NULL)
        {
            Pose_to_Affine(*posePtr,m_T_r2w);
            m_T_w2r = m_T_r2w.inverse();
            cout << "Read m_T_r2w from bag successfully: " << endl << m_T_r2w.matrix() << endl;
            i_m_T_r2w++;
        }
    }
    if(i_m_T_r2w==0)
        cerr << "Cannot find m_T_r2w from bag" << endl;

    topics.clear();
    topics.push_back(std::string("m_T_b2c"));
    rosbag::View view2(m_bag, rosbag::TopicQuery(topics));

    int i_m_T_b2c = 0;
    foreach(rosbag::MessageInstance const m, view2)
    {
        posePtr = m.instantiate<geometry_msgs::PoseStamped>();
        if (posePtr != NULL)
        {
            Pose_to_Affine(*posePtr,m_T_b2c);
            cout << "Read m_T_b2c from bag successfully: " << endl << m_T_b2c.matrix() << endl;
            i_m_T_b2c++;
        }
    }
    if(i_m_T_b2c==0)
        cerr << "Cannot fine i_m_T_b2c from bag" << endl;

    m_bag.close();
    return true;
}

void Listener::CalDataDelay(float& optiDelay, float& arucoDelay)
{
    float t1 = ros::Time::now().toSec();
    float t2 = m_Pose_opti.header.stamp.toSec();
//    cout << "t1=" << t1 << endl;
//    cout << "t2=" << t2 << endl;

    optiDelay = ros::Time::now().toSec() - m_Pose_opti.header.stamp.toSec();
    arucoDelay = ros::Time::now().toSec() - m_Pose_aruco.header.stamp.toSec();
}

bool Listener::IsOptiNew()
{
    float optiDelay = ros::Time::now().toSec() - m_Pose_opti.header.stamp.toSec();
    if(optiDelay < 0.02)
        return true;
    else
        return false;
}
bool Listener::IsArucoNew()
{
    float arucoDelay = ros::Time::now().toSec() - m_Pose_aruco.header.stamp.toSec();
    if(arucoDelay < 0.04)
        return true;
    else
        return false;
}

void Listener::Cal_OptiRef()
{
    Pose_to_Affine( m_Pose_opti,m_T_w2b );
    m_T_r2c_Ref = m_T_b2c * m_T_w2b * m_T_r2w;
    Affine_to_Pose(m_T_r2c_Ref,m_Pose_OptiRef);
    m_Pose_OptiRef.header = m_Pose_opti.header;
// m_Pose_OptiRef.header.stamp
    m_Pose_OptiRef.header.frame_id = "m_Pose_OptiRef";

    modifyRosTime(m_Pose_OptiRef.header.stamp,m_ArucoDelayCompensate);

    opti_PoseRef_pub.publish(m_Pose_OptiRef);

}
void Listener::modifyRosTime(ros::Time& Time,float addSec)
{
    uint32_t nsec = Time.nsec + addSec * 1e9;
    if(nsec >= 1e9)
    {
        int temp = (int)(nsec*1e-9);
        nsec = nsec - temp * 1e9;
        Time.sec += temp;
        Time.nsec = nsec;
    }
    else
    {
        Time.nsec = nsec;
    }
    m_ArucoDelayCompensate;
}

float Listener::Cal_PoseErr()
{
    // calculate Delay
    float delay = m_Pose_aruco.header.stamp.toSec() - m_Pose_OptiRef.header.stamp.toSec();
    if(delay > 0.02)
    {
        cout << "delay = aruco - opti = " << delay << endl << "Cal_PoseErr failed" << endl;
        return delay;
    }
    Pose_to_Affine( m_Pose_aruco,m_T_r2c );
    m_T_r2c_err = m_T_r2c * m_T_r2c_Ref.inverse();
    Affine_to_Pose(m_T_r2c_err,m_Pose_err);
    m_Pose_err.header = m_Pose_opti.header;
    m_Pose_err.header.frame_id = "m_Pose_err";

    PoseErr_pub.publish(m_Pose_err);

    // euler angle err
    geometry_msgs::Vector3Stamped ErrEuler;
    PoseMsg_to_EulerMsg(m_Pose_opti,ErrEuler,m_aruco_eulerOrder,"ErrEuler");

    ErrEuler.vector.z = delay;
    ErrEuler.header.frame_id = delay;

    ErrEuler_pub.publish(ErrEuler);

//    cout  << " , m_T_r2c = " << endl << m_T_r2c.matrix() << endl;
//    cout  << " , m_T_r2c_Ref.inverse() = " << endl << m_T_r2c_Ref.inverse().matrix() << endl;
//    cout  << " , m_T_r2c_err = " << endl << m_T_r2c_err.matrix() << endl;
//    cout << endl;

    using namespace std;
//    cout << "m_Pose_OptiRef = " << endl << m_Pose_OptiRef << endl ;
//    cout << "m_Pose_err = " << endl << m_Pose_err << endl ;
//    cout << endl;
    return delay;
}

void Listener::PoseMsg_to_EulerMsg(geometry_msgs::PoseStamped pose, geometry_msgs::Vector3Stamped& euler,Vector3i EulerOrder,string eulerFrameid)
{
    Quaternionf Q(pose.pose.orientation.w,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z);
    Matrix3f R_inv(Q);
    Vector3f Euler_Eigen = R_inv.eulerAngles(EulerOrder(0),EulerOrder(1),EulerOrder(2)) * 180.0/M_PI;
    euler.vector.x = Euler_Eigen(0);
    euler.vector.y = Euler_Eigen(1);
    euler.vector.z = Euler_Eigen(2);
    euler.header = pose.header;
    euler.header.frame_id = eulerFrameid.c_str();

//    for(int i=0;i<3;i++)
//    {
//        if( euler.vector(i) > 180 )
//            euler.vector(i) = euler.vector(i) - 180;
//        if( euler.vector(i) < -180 )
//            euler.vector(i) = euler.vector(i) + 180;
//    }

 //   euler.vector.z = Q.angularDistance(Quaternionf::Identity())*180.0/M_PI;
}

void Listener::Pose_to_Affine(geometry_msgs::PoseStamped Pose_ros_a2b, Affine3f& T_eigen_a2b)
{
//     cout << "Pose_ros_a2b" << endl << Pose_ros_a2b << endl;

      Quaternionf q_a2b (Pose_ros_a2b.pose.orientation.w,Pose_ros_a2b.pose.orientation.x,Pose_ros_a2b.pose.orientation.y,Pose_ros_a2b.pose.orientation.z);
      Matrix3f R_a2b (q_a2b.inverse());
      Vector3f T_ab_a ( Pose_ros_a2b.pose.position.x,Pose_ros_a2b.pose.position.y,Pose_ros_a2b.pose.position.z);
      Vector3f T_ba_b = -R_a2b * T_ab_a;
      T_eigen_a2b.linear() = R_a2b;
      T_eigen_a2b.translation() = T_ba_b;

      using namespace std;
//       cout << "T_eigen_a2b" << endl << T_eigen_a2b.matrix() << endl;
}

void Listener::Affine_to_Pose(const Affine3f& T_eigen_a2b,geometry_msgs::PoseStamped& Pose_ros_a2b)
{
      Vector3f T_ba_b = T_eigen_a2b.translation();
      Matrix3f R_b2a = T_eigen_a2b.linear().inverse();
      Quaternionf q_a2b(R_b2a);
      Vector3f T_ab_a = - R_b2a * T_ba_b;

      Pose_ros_a2b.pose.position.x = T_ab_a(0);
      Pose_ros_a2b.pose.position.y = T_ab_a(1);
      Pose_ros_a2b.pose.position.z = T_ab_a(2);
      Pose_ros_a2b.pose.orientation.w = q_a2b.w();
      Pose_ros_a2b.pose.orientation.x = q_a2b.x();
      Pose_ros_a2b.pose.orientation.y = q_a2b.y();
      Pose_ros_a2b.pose.orientation.z = q_a2b.z();

      Pose_ros_a2b.header.stamp = ros::Time::now();
}

void Listener::Calculate_Pr2w()
{

  float markerRadius = 0.009;

  Vector4f markerHigh_0( 0,0, markerRadius,0 );

  X_r.col(0) = Vector4f(-0.1,  0.1, 0, 1) + markerHigh_0;
  X_r.col(1) = Vector4f( 0.1,  0.1, 0, 1) + markerHigh_0;
  X_r.col(2) = Vector4f( 0.1, -0.1, 0, 1) + markerHigh_0;
  X_r.col(3) = Vector4f(-0.1, -0.1, 0, 1) + markerHigh_0;


  Vector4f markerHigh_4( 0,markerRadius,0,0 );

  X_r.col(4) = Vector4f ( -4.3972393870353699e-001,-7.7984249591827393e-001, 4.3459945917129517e-001, 1 ) + markerHigh_4;
  X_r.col(5) = Vector4f ( -2.3981997370719910e-001, -7.7381861209869385e-001,4.3605741858482361e-001, 1 ) + markerHigh_4;
  X_r.col(6) = Vector4f (  -2.4125066399574280e-001, -7.7474486827850342e-001, 6.3605016469955444e-001, 1 ) + markerHigh_4;
  X_r.col(7) = Vector4f ( -4.4115462899208069e-001, -7.8076875209808350e-001,6.3459217548370361e-001, 1 ) + markerHigh_4;

//  cout << X_r << endl;

  X_w.col(0) = Vector4f( -1.412468 , 1.609276 , 3.097998, 1 );
  X_w.col(1) = Vector4f( -1.607 , 1.602067 , 3.092549, 1  );
  X_w.col(2) = Vector4f( -1.601349,  1.403421,  3.092784, 1  );
  X_w.col(3) = Vector4f( -1.401656,  1.410452, 3.099109, 1  );

  X_w.col(4) = Vector4f( -1.033751, 0.762876, 2.699498, 1);
  X_w.col(5) = Vector4f( -1.231378, 0.761586, 2.688207, 1 );
  X_w.col(6) = Vector4f( -1.226811, 0.759083, 2.494190, 1 );
  X_w.col(7) = Vector4f( -1.028034, 0.760699, 2.500625, 1 );


  // solve m_T_r2w

  MatrixXf A = X_r.transpose();
  MatrixXf T_w2r =  A.jacobiSvd(ComputeThinU | ComputeThinV).solve( X_w.transpose() ) ;
  m_T_r2w.matrix() = T_w2r.inverse();
//   cout << "m_T_r2w by solving linear least squares = " << endl << m_T_r2w.matrix() <<endl;

  // normorize m_T_r2w as a affine3f
  m_T_r2w.matrix().row(3) = Vector4f(0,0,0,1);
  Quaternionf q_w2r ( m_T_r2w.linear() );
  q_w2r.normalize();
//   cout << "q_w2r =" <<endl<< q_w2r.coeffs() <<endl;
  m_T_r2w.linear() = q_w2r.toRotationMatrix();

  cout << "Normalized m_T_r2w = " << endl << m_T_r2w.matrix() <<endl;

  // check m_T_r2w
  m_T_w2r = m_T_r2w.inverse();
//   cout << " m_T_w2r = " << endl << m_T_w2r.matrix() <<endl;
}

void Listener::UpdateInstallCalibBagFile()
{
    WriteBag_T(m_T_r2w,"m_T_r2w");
    AppendBag_T(m_T_b2c,"m_T_b2c");

    char str[200];
    sprintf(str,"update %s successfuly \n",m_InstallCalibResultPath.c_str());
    cout << str << endl << "m_T_r2w = " << endl << m_T_r2w.matrix() << endl
         << "m_T_b2c = " << endl << m_T_b2c.matrix() << endl;
}

void Listener::WriteBag_T(const Affine3f& T, const string name)
{
        m_bag.open(m_InstallCalibResultPath,rosbag::bagmode::Write);
        cout << "Write open successfully" << endl;


    geometry_msgs::PoseStamped m_T_Pose;
    Affine_to_Pose(T,m_T_Pose);
    m_bag.write(name,ros::Time::now(),m_T_Pose);
    m_bag.close();
}
void Listener::AppendBag_T(const Affine3f& T,const string name)
{
    m_bag.open(m_InstallCalibResultPath,rosbag::bagmode::Append);
    cout << "Append open successfully" << endl;

    geometry_msgs::PoseStamped m_T_Pose;
    Affine_to_Pose(T,m_T_Pose);
    m_bag.write(name,ros::Time::now(),m_T_Pose);
    m_bag.close();
}

bool Listener::GetInstallCalibData_Online( int N_points,std::vector<Affine3f>& T_w2b_Array,std::vector<Affine3f>& T_r2c_Array)
{
    char str[200];

    float optiDelay, arucoDelay;
    CalDataDelay(optiDelay,  arucoDelay);
    if(optiDelay > 0.02 )
    {
      sprintf(str,"optiDelay = %0.3f\n",optiDelay);
      cout << str;
      return false;
    }
    if(arucoDelay > 0.05 )
    {
      sprintf(str,"arucoDelay = %0.3f\n",arucoDelay);
      cout << str;
      return false;
    }


    char key = 0,i = 0;
    Affine3f T_w2b,T_r2c,T_b2c_i;
    cv::Mat image = cv::Mat::zeros( 640,480, CV_8UC3 );

    cv::namedWindow("calib_P_b2c",cv::WINDOW_AUTOSIZE);


    cout << "update P_b2c, press y\n";
    cv::putText(image, string("update P_b2c, press y"), cv::Point(10,30), CV_FONT_HERSHEY_SIMPLEX, 0.8, cvScalar(200,200,250), 1, CV_AA);
    cv::imshow("calib_P_b2c",image);

    rosbag::Bag CalibInstallBag;
    CalibInstallBag.open(m_InstallCalibInputPath, rosbag::bagmode::Write);

    key = cv::waitKey(-1);
   if(key=='y')
   {
      cout << "press a to add a data[0]\npress c to stop and calculate\n";

      cv::putText(image, string("press a to add a data[0]"), cv::Point(10,60), CV_FONT_HERSHEY_SIMPLEX, 0.8, cvScalar(200,200,250), 1, CV_AA);
      cv::imshow("calib_P_b2c",image);
      do{

          while( ( key = cv::waitKey(10) ) == -1);
          if(key=='a')
          {
              CalibInstallBag.write(m_InCal_opti_Topic,ros::Time::now(),m_Pose_opti);
              CalibInstallBag.write(m_InCal_aruco_Topic,ros::Time::now(),m_Pose_aruco);

              Pose_to_Affine(m_Pose_opti,T_w2b);
              Pose_to_Affine(m_Pose_aruco,T_r2c);

              T_w2b_Array.push_back(T_w2b);
              T_r2c_Array.push_back(T_r2c);


//              T_b2c_i = T_r2c * m_T_r2w.inverse() * T_w2b.inverse() ;

//              T_b2c_N.block<4,4>(i*4,0) = T_b2c_i.matrix();
//              I_N.block<4,4>(i*4,0) = Matrix4f::Identity();
              i++;

              /*
              cout << "m_T_r2w.inv = " << endl << m_T_r2w.inverse().matrix() << endl;
              cout << "T_w2b = " << endl << T_w2b.matrix() << endl;
              cout << "T_w2b.inv = " << endl << T_w2b.inverse().matrix() << endl;

              cout << "T_b2c_i= " << T_b2c_i.matrix() << endl;
*/
              sprintf(str,"press a to add data[ %d ]",i);
              string s(str);
              cv::putText(image, s, cv::Point(10,30*i+60), CV_FONT_HERSHEY_SIMPLEX, 0.8, cvScalar(200,200,250), 1, CV_AA);
              cv::imshow("calib_P_b2c",image);
              cv::waitKey(30);
              printf("T_w2b[ %d ] = \n",i);
              cout << T_w2b.matrix() << endl;
              printf("T_r2c[ %d ] = \n",i);
              cout << T_r2c.matrix() << endl;
          }

      }
      while(i<N_points);

      image = cv::Mat::zeros( 640,480, CV_8UC3 );
      cv::putText(image, string("calib data collect ok"), cv::Point(120,60), CV_FONT_HERSHEY_SIMPLEX, 0.8, cvScalar(200,200,250), 1, CV_AA);
      cv::imshow("calib_P_b2c",image);
      cv::waitKey(500);

      cv::destroyWindow("calib_P_b2c");
      CalibInstallBag.close();
   }
   return true;
}

bool Listener::LoadBag_InstallCalibData( int& N_points,std::vector<Affine3f>& T_w2b_Array,std::vector<Affine3f>& T_r2c_Array)
{
    std::vector<geometry_msgs::PoseStamped> ArucoPose_V,OptiPose_V;
    ReadBag_Pose(m_InstallCalibInputPath,m_InCal_aruco_Topic,ArucoPose_V);
    ReadBag_Pose(m_InstallCalibInputPath,m_InCal_opti_Topic,OptiPose_V);

    int N1 = ArucoPose_V.size();
    int N2 = OptiPose_V.size();
    int N = min(N1,N2);
    N_points=  N;
    Affine3f T_w2b,T_r2c;
    for(int i=0;i<N;i++)
    {
        float delay = ArucoPose_V.at(i).header.stamp.toSec() - OptiPose_V.at(i).header.stamp.toSec();
        if(delay > 0.02)
        {
            cout << "delay = " << delay << endl << "LoadBag_InstallCalibData failed" << endl;
            return false;
        }

        Pose_to_Affine(OptiPose_V.at(i),T_w2b);
        Pose_to_Affine(ArucoPose_V.at(i),T_r2c);

        T_w2b_Array.push_back(T_w2b);
        T_r2c_Array.push_back(T_r2c);
    }
}

void Listener::LoadBag_CalErr(const string Path)
{
    std::vector<geometry_msgs::PoseStamped> ArucoPose_V,OptiPose_V;
    ReadBag_Pose(Path,m_InCal_aruco_Topic,ArucoPose_V);
    ReadBag_Pose(Path,m_InCal_opti_Topic,OptiPose_V);

    int N1 = ArucoPose_V.size();
    int N2 = OptiPose_V.size();
    int N = min(N1,N2);
    float delay;
    for(int i=0;i<N;i++)
    {
        m_Pose_opti = OptiPose_V.at(i);
        m_Pose_aruco = ArucoPose_V.at(i);
        Cal_OptiRef();
        delay = Cal_PoseErr();

  //      cout << i << " , m_T_r2c_Ref = " << endl << m_T_r2c_Ref.matrix() << endl;
        cout << i << " , m_T_r2c_err = " << endl << m_T_r2c_err.matrix() << endl;
        cout << "delay = " << delay << endl;
        cout << endl;
    }

    cout<< "LoadBag_CalErr ends"<< endl;
}

bool Listener::Calculate_P_b2c(bool IsLoadFromFile)
{
    // check
    char str[200];


     int N_points = 2*5;
    std::vector<Affine3f> T_w2b_Array,T_r2c_Array;

    Affine3f T_w2b,T_r2c,T_b2c_i;
    Matrix<float,Dynamic,Dynamic> T_b2c_N,I_N;

    if(IsLoadFromFile)
    {
        LoadBag_InstallCalibData(N_points,T_w2b_Array,T_r2c_Array);
    }
    else
    {
        bool flag = GetInstallCalibData_Online(N_points,T_w2b_Array,T_r2c_Array);
        if(!flag)
        {
            cout << "GetInstallCalibData_Online Failed"<< endl;
            return false;
        }
    }
    T_b2c_N.resize(N_points*4,4);
    I_N.resize(N_points*4,4);

        for(int i=0;i<N_points;i++)
        {
            T_w2b = T_w2b_Array.at(i);
            T_r2c = T_r2c_Array.at(i);
            T_b2c_i = T_r2c * m_T_r2w.inverse() * T_w2b.inverse() ;
            T_b2c_N.block<4,4>(i*4,0) = T_b2c_i.matrix();
            I_N.block<4,4>(i*4,0) = Matrix4f::Identity();

//            printf("T_w2b[ %d ] = \n",i);
//            cout << T_w2b.matrix() << endl;
//            printf("T_r2c[ %d ] = \n",i);
//            cout << T_r2c.matrix() << endl;
        }
        cout << "T_b2c_N = " << endl << T_b2c_N << endl;

        MatrixXf A = T_b2c_N;
        MatrixXf T_c2b = A.jacobiSvd(ComputeThinU | ComputeThinV).solve( I_N ) ;
        m_T_b2c.matrix() = T_c2b.inverse();
        NormalizeAffine(m_T_b2c);

        MatrixXf Err = T_b2c_N * T_c2b;
        cout << "Err = " << endl << Err << endl;

        //  -----
 //       UpdateInstallCalibBagFile();

        Matrix3f R_c2b = m_T_b2c.linear().inverse();
        Quaternionf Q_b2c_old(R_c2b);

        //     cout << "T_b2c_N= " << T_b2c_N << endl;
       //      cout << "I_N= " << I_N << endl;
             cout << "old method, m_T_b2c= " << endl<< m_T_b2c.matrix() << endl;
             cout << "old method, Q_b2c_old= " << endl<< Q_b2c_old.coeffs() << endl;

        // new method
        MatrixXf M;
        M.resize(N_points*4,4);
        for(int i=0;i<N_points;i++)
        {
            int j = -1;
            float w_abs_max = 0;
            // search for j which Q.w is biggest
            for(int k=0;k<N_points;k++)
            {
                if(k==i)
                    continue;
                Affine3f dTb_k2i = T_w2b_Array.at(i) * T_w2b_Array.at(k).inverse();
                Matrix3f dR_i2k = dTb_k2i.linear().inverse();
                Quaternionf Qb_k2i (dR_i2k);
                if(abs((float)Qb_k2i.w()) > w_abs_max)
                {
                    w_abs_max = abs((float)Qb_k2i.w());
                    j = k;
                }
            }
            sprintf(str,"max w = %0.2f at [%d]",w_abs_max,j);

          Affine3f dTc_j2i = T_r2c_Array.at(i) * T_r2c_Array.at(j).inverse();
          Matrix3f Rc_i2j = dTc_j2i.linear().inverse();
          Quaternionf Qc_j2i (Rc_i2j);

          Affine3f dTb_j2i = T_w2b_Array.at(i) * T_w2b_Array.at(j).inverse();
          Matrix3f Rb_i2j = dTb_j2i.linear().inverse();
          Quaternionf Qb_j2i (Rb_i2j);

          Matrix4f M_c_j2i, Mt_b_j2i;
          GetQM(Qc_j2i,M_c_j2i);
          GetQMt(Qb_j2i,Mt_b_j2i);
          Matrix4f M_temp = M_c_j2i - Mt_b_j2i;

          MatrixXf mm = M_temp.block<3,4>(0,0);
          MatrixXf ker = mm.fullPivLu().kernel();
          Quaternionf Qi(ker(0),ker(1),ker(2),ker(3));
          Qi.normalize();
          cout << "Qi = " << endl << Qi.coeffs() << endl;

          M.block<4,4>(i*4,0) = M_temp;
        }
        VectorXf ZERO = VectorXf::Zero(N_points*4);

        VectorXf Q_c2b0 = M.jacobiSvd(ComputeThinU | ComputeThinV).solve( ZERO ) ;

        MatrixXf ker = M.fullPivLu().kernel();


        cout << "ker = " << endl << ker << endl;
        cout << "M = " << endl << M << endl;
        cout << "Q_c2b0 = " << endl << Q_c2b0 << endl;

return true;
        // get Q_c2b
        Quaternionf Q_c2b( Q_c2b0(0),Q_c2b0(1),Q_c2b0(2),Q_c2b0(3) );
        Q_c2b.normalize();

         R_c2b = Q_c2b.toRotationMatrix();
        Matrix3f R_b2c = R_c2b.inverse();

        // calculate P_cb_c
        MatrixXf A_N;
        VectorXf b_N;
        A_N.resize(N_points*3,3);
        b_N.resize(N_points);
        for(int i=0;i<N_points;i++)
        {
            int j = -1;
            float D_abs_max = 0;
            // search for j which D is biggest
            for(int k=0;k<N_points;k++)
            {
                if(k==i)
                    continue;
                Affine3f dTb_k2i = T_w2b_Array.at(i) * T_w2b_Array.at(k).inverse();
                float D = dTb_k2i.translation().norm();
                if(D > D_abs_max)
                {
                    D_abs_max = D;
                    j = k;
                }
            }

          Affine3f dTc_j2i = T_r2c_Array.at(i) * T_r2c_Array.at(j).inverse();
          Matrix3f R_c_j2i = dTc_j2i.linear();
          Vector3f P_c_j2i = dTc_j2i.translation();

          Affine3f dTb_j2i = T_w2b_Array.at(i) * T_w2b_Array.at(j).inverse();
          Matrix3f R_b_j2i = dTb_j2i.linear();
          Vector3f P_b_j2i = dTb_j2i.translation();

          b_N.block<3,1>(i*3,0) = P_c_j2i - R_b2c * P_b_j2i;
          A_N.block<3,3>(i*3,0) = Matrix3f::Identity() - R_b2c*R_b_j2i*R_c2b;

          // get P_b2c
          Vector3f P_b2c = A_N.jacobiSvd(ComputeThinU | ComputeThinV).solve( b_N ) ;

          // finally get T_b2c
          m_T_b2c.linear() = R_b2c;
          m_T_b2c.translation() = P_b2c;
        }
        cout << "new method, m_T_b2c= " << m_T_b2c.matrix() << endl;

return true;
}

void Listener::GetQM( Quaternionf Q,Matrix4f& M )
{
    M(0,0) = Q.w();
    Vector3f V(Q.x(),Q.y(),Q.z());
    M.block<1,3>(0,1) = -V.transpose();
    M.block<3,1>(1,0) = V;
    Matrix3f V_cross;
    V_cross.col(0) = V.cross(Vector3f(1,0,0));
    V_cross.col(1) = V.cross(Vector3f(0,1,0));
    V_cross.col(2) = V.cross(Vector3f(0,0,1));
    M.block<3,3>(1,1) = Q.w() * Matrix3f::Identity() + V_cross;

 //   cout <<"M=" << endl << M << endl;
}

void Listener::GetQMt( Quaternionf Q,Matrix4f& Mt )
{
    Mt(0,0) = Q.w();
    Vector3f V(Q.x(),Q.y(),Q.z());
    Mt.block<1,3>(0,1) = -V.transpose();
    Mt.block<3,1>(1,0) = V;
    Matrix3f V_cross;
    V_cross.col(0) = V.cross(Vector3f(1,0,0));
    V_cross.col(1) = V.cross(Vector3f(0,1,0));
    V_cross.col(2) = V.cross(Vector3f(0,0,1));
    Mt.block<3,3>(1,1) = Q.w() * Matrix3f::Identity() - V_cross;
}

void Listener::NormalizeAffine(Affine3f& T)
{
    // normorize m_T_r2w as a affine3f
    T.matrix().row(3) = Vector4f(0,0,0,1);
    Quaternionf q_inv ( T.linear() );
    q_inv.normalize();
 //   cout << "q_0 =" <<endl<< q.coeffs() <<endl;
    T.linear() = q_inv.toRotationMatrix();
}

void ReadBag()
{
    using namespace std;
    rosbag::Bag bag;
    bag.open("test.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("chatter"));
    topics.push_back(std::string("numbers"));
    topics.push_back(std::string("pose"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    geometry_msgs::PoseStamped::ConstPtr pose;
    geometry_msgs::PoseStamped pose1,pose2;

    int pose_i = 0;
    foreach(rosbag::MessageInstance const m, view)
    {
        std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
        if (s != NULL)
            cout << s->data << endl;
        //   ASSERT_EQ(s->data, std::string("foo"));

        std_msgs::Float32::ConstPtr i = m.instantiate<std_msgs::Float32>();
        if (i != NULL)
            cout << i->data << endl;
     //       ASSERT_EQ(i->data, 42);
        pose = m.instantiate<geometry_msgs::PoseStamped>();
        if (pose != NULL)
        {
            cout << pose->header.seq << endl;
            if(pose_i==0)
                pose1 = *pose;
            if(pose_i==1)
                pose2 = *pose;
            pose_i++;
        }
    }

    bag.close();

        cout << "read bag ok" << endl;
}
void WriteBag()
{
    using namespace std;
    rosbag::Bag bag;
    bag.open("test.bag", rosbag::bagmode::Write);

    std_msgs::String str;
    str.data = std::string("foo");

    std_msgs::Float32 i;
    i.data = 42;

    geometry_msgs::PoseStamped pose;
    pose.header.seq=245;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = 4.236;

//    bag.write("chatter", ros::Time::now(), str);
//    bag.write("numbers", ros::Time::now(), i);
//    bag.write("pose",pose.header.stamp,pose);

    pose.header.seq=436576554;
    pose.pose.position.x = -9934436.35;
    bag.write("pose",pose.header.stamp,pose);

    bag.close();

    cout << "write bag ok" << endl;
}

int test()
{

}

