
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>

#include <usv_opt_tracker/usv_opt_tracker.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <mutex>
#include <condition_variable>
#include <Eigen/Eigen>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <thread>

#include <usv_opt_tracker/tunesConfig.h>

#include "traj_opt/traj_min_snap.hpp"

#include "DspFilters/Dsp.h"

using namespace std;
using namespace Eigen;

class ControlInterface
{
public:
  ros::NodeHandle node_;
  ros::Subscriber reference_traj_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber gp_training_data_sub;
  ros::Publisher left_thrust_pub_;
  ros::Publisher right_thrust_pub_;
  ros::Publisher traj_horizon_pub_;
  ros::Publisher traj_predict_pub_;
  ros::Publisher acc_error_pub_;
  ros::Publisher track_error_pub_;
  ros::Publisher solution_time_pub_;
  dynamic_reconfigure::Server<dynamic_tunes::tunesConfig> server_;
  dynamic_reconfigure::Server<dynamic_tunes::tunesConfig>::CallbackType f_;
  ros::ServiceServer gp_model_change_serv;

  double control_frequency;
  double nmpc_pred_hori;
  double time_interval;
  double min_thrust_force;
  double max_thrust_force;
  double max_thrust_force_rate;
  int nmpc_pred_hori_num;
  bool GPUpdateEnable;  // param for whether to update GP online
  bool GPREnable;      // param for whether to use GPR augmented model
  bool loop_track;     // true: cyclic tracking, false: single-pass tracking
  std::string modelName;
  nav_msgs::Path current_ref_traj;
  nav_msgs::Odometry current_odom;
  bool have_odom;
  double traj_ref_vel;
  bool udatepath = true;
  size_t traj_start_id = 0;
  std::unique_ptr<usv_system::UsvOptTracker> opt_tracker_;
  std::unique_ptr<min_snap::SnapOpt> traj_optimizer_;
  min_snap::Trajectory minSnapTraj;


  std::vector<VectorXd> gpTargets_vec;
  int gp_batch_size;

  ControlInterface(ros::NodeHandle& nh) : node_(nh)
  {
    node_.param<double>("ControlFrequency", control_frequency, 20.0);
    node_.param<bool>("GPUpdateEnable", GPUpdateEnable, false);
    node_.param<bool>("GPREnable", GPREnable, false);
    node_.param<bool>("loop_track", loop_track, true);
    node_.param<int>("GPBatchSize", gp_batch_size, 20);
    node_.param<double>("lambda_learning", lamda_learning, 0.1);
    node_.param<std::string>("modelName", modelName, "wamv_nominal");
    reference_traj_sub_ = node_.subscribe<nav_msgs::Path>("/trajectory", 1, &ControlInterface::trajCbk, this,
                                                          ros::TransportHints().tcpNoDelay());
    odom_sub_ = node_.subscribe<nav_msgs::Odometry>("/odom", 1, &ControlInterface::odomCbk, this,
                                                 ros::TransportHints().tcpNoDelay());

    gp_training_data_sub = node_.subscribe<std_msgs::Float64MultiArray>("tracker/acceleration_error", 1, &ControlInterface::gpUpdateDataCbk, this, ros::TransportHints().tcpNoDelay());
    gp_model_change_serv = node_.advertiseService("/tracker/gp/change", &ControlInterface::gpModelChangeServiceCbk, this);
    gpTargets_vec.reserve(gp_batch_size);

    opt_tracker_.reset(new usv_system::UsvOptTracker(GPREnable,modelName));
    traj_optimizer_.reset(new min_snap::SnapOpt());

    int sample_rate;
    node_.param<int>("OdomRate", sample_rate, 100);
    double cutoff_freq;
    node_.param<double>("OdomCutoffFreq", cutoff_freq, 0.25 * sample_rate);
    initFilters(sample_rate, cutoff_freq);

    left_thrust_pub_ = node_.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 1);
    right_thrust_pub_ = node_.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 1);
    traj_horizon_pub_ = node_.advertise<nav_msgs::Path>("tracker/trajectory/horizon", 2);
    traj_predict_pub_ = node_.advertise<nav_msgs::Path>("tracker/trajectory/nmpc_predict", 2);
    acc_error_pub_ = node_.advertise<std_msgs::Float64MultiArray>("tracker/acceleration_error", 2);
    solution_time_pub_ = node_.advertise<std_msgs::Float64MultiArray>("tracker/solution_time", 2);
    track_error_pub_ = node_.advertise<std_msgs::Float64MultiArray>("tracker/track_error", 2);

    nmpc_pred_hori = opt_tracker_->getMpcHorizon();
    time_interval = opt_tracker_->getTimeInterval();
    min_thrust_force = opt_tracker_->getMinInput();
    max_thrust_force = opt_tracker_->getMaxInput();
    max_thrust_force_rate = opt_tracker_->getMaxInputRate();
    nmpc_pred_hori_num = round(nmpc_pred_hori / time_interval);
    traj_ref_vel = opt_tracker_->getMaxVelocity();
    current_ref_traj = nav_msgs::Path();
    have_odom = false;
    control_state.setZero();

    ROS_INFO_STREAM("Trajectory tracking mode: "
                    << (loop_track ? "loop (cyclic)" : "single pass"));

    f_ = boost::bind(&ControlInterface::configCallback, this, _1, _2);
    server_.setCallback(f_);
  }

  void configCallback(dynamic_tunes::tunesConfig& config, uint32_t level)
  {
    traj_ref_vel = config.velocity;
    GPUpdateEnable = config.GPUpdate;
    udatepath = true;
  }

  bool gpModelChangeServiceCbk(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    if(GPUpdateEnable && GPREnable)
    {
      opt_tracker_->callGPModelchange();
      return true;
    }else{
      ROS_WARN("Unable to call GP model change service, please check if GP update and GPREnable are set true");
      return false;
    }
  }

  void trajCbk(const nav_msgs::PathConstPtr& msg)
  {
    if(!udatepath)
      return;
    std::lock_guard<mutex> lck(traj_mtx);
    nav_msgs::Path pathRaw = *msg;
    int length = pathRaw.poses.size();
    if (length == 0)
    {
      current_ref_traj = nav_msgs::Path();
      stopThrusters();
    }
    else if (length == 1)
    {
      current_ref_traj = pathRaw;
    }
    else
    {
      double vel_max = traj_ref_vel;
      current_ref_traj = processPath(pathRaw, vel_max);
      traj_start_id = 0;
    }
    udatepath = false;
  }

  void odomCbk(const nav_msgs::OdometryConstPtr& odom_in)
  {
    std::lock_guard<mutex> lck(odom_mtx);

    // Eigen::Vector3d vel_raw;
    // vel_raw[0] = odom_in->twist.twist.linear.x;
    // vel_raw[1] = odom_in->twist.twist.linear.y;
    // vel_raw[2] = odom_in->twist.twist.angular.z;
    // Eigen::Vector3d vel_filt;
    // getFilterVec(vel_raw,vel_filt);

    current_odom = *odom_in;

    // current_odom.twist.twist.linear.x = vel_filt[0];
    // current_odom.twist.twist.linear.y = vel_filt[1];
    // current_odom.twist.twist.angular.z = vel_filt[2];

    have_odom = true;
  }

  void gpUpdateDataCbk(const std_msgs::Float64MultiArrayConstPtr & msg){
    static size_t input_counts = 0;
    size_t skip = 5;
    if(GPUpdateEnable && GPREnable)
    {
      if (input_counts % skip ==0)
      {
        gpTargets_vec.push_back(Eigen::Map<const VectorXd>(msg->data.data(), msg->data.size()));
      }
      if (gpTargets_vec.size() >= gp_batch_size)
      {
        opt_tracker_->updateGP(gpTargets_vec,lamda_learning);
        input_counts = 0;
        gpTargets_vec.clear();
        gpTargets_vec.reserve(gp_batch_size);
      }
      input_counts++;
    }
  }

  void controller_loop()
  {
    ros::Rate rate(control_frequency);
    ct::core::StateVector<usv::USVDynamicModel::STATE_DIM> x_state_last;
    ct::core::StateVector<usv::USVDynamicModel::STATE_DIM> x_ref_last;
    ct::core::StateVectorArray<usv::USVDynamicModel::STATE_DIM> x_pred_traj;
    ct::core::ControlVector<usv::USVDynamicModel::CONTROL_DIM> control_opt;
    double t_opt_pub = -1;

    while (ros::ok())
    {
      std::lock_guard<mutex> lck(control_mtx);
      if (!have_odom)
        continue;

      if (current_ref_traj.poses.empty())
        continue;

      //获取当前状态
      ct::core::StateVector<usv::USVDynamicModel::STATE_DIM> x_state_now;
      {
        std::lock_guard<mutex> lck1(odom_mtx);
        
        static double last_yaw = 0.0;
        static double yaw_offset = 0.0;
        double cur_yaw_raw = tf::getYaw(current_odom.pose.pose.orientation);
        // unwrap yaw to keep it continuously increasing to avoid discontinuities
        double delta_yaw = cur_yaw_raw - last_yaw;
        if (delta_yaw > M_PI)
          yaw_offset -= 2 * M_PI;
        else if (delta_yaw < -M_PI)
          yaw_offset += 2 * M_PI;
        double cur_yaw = cur_yaw_raw + yaw_offset;
        last_yaw = cur_yaw_raw;

        x_state_now.setZero();
        x_state_now(0) = current_odom.pose.pose.position.x;
        x_state_now(1) = current_odom.pose.pose.position.y;
        x_state_now(4) = cur_yaw;
        x_state_now(2) = cos(x_state_now(4));
        x_state_now(3) = sin(x_state_now(4));
        x_state_now(5) = current_odom.twist.twist.linear.x;
        x_state_now(6) = current_odom.twist.twist.linear.y;
        x_state_now(7) = current_odom.twist.twist.angular.z;
        x_state_now(8) = control_state(0);
        x_state_now(9) = control_state(1);
      }

      //获取当前参考
      ct::core::StateTrajectory<usv::USVDynamicModel::STATE_DIM> x_ref_traj(InterpolationType::LIN);
      ct::core::ControlTrajectory<usv::USVDynamicModel::CONTROL_DIM> u_ref_traj(InterpolationType::LIN);

      Vector3d cmd_pos;
      Vector3d cmd_vel;
      double cmd_yaw_rate = 0.0;

      nav_msgs::Path traj_hori;
      traj_hori.header.frame_id = "odom";
      traj_hori.header.stamp = ros::Time::now();
      traj_hori.poses.reserve(nmpc_pred_hori_num);
      {
        std::lock_guard<mutex> lck2(traj_mtx);

        if (current_ref_traj.poses.empty())
          continue;

        const size_t path_size = current_ref_traj.poses.size();
        if (loop_track)
        {
          // Cyclic mode: wrap the prediction horizon from the last point
          // back to the first point and advance one sample per control cycle.
          traj_start_id %= path_size;
          for (int i = 0; i < nmpc_pred_hori_num; ++i)
          {
            const size_t index = (traj_start_id + static_cast<size_t>(i)) % path_size;
            traj_hori.poses.push_back(current_ref_traj.poses[index]);
          }
          traj_start_id = (traj_start_id + 1) % path_size;
        }
        else
        {
          // Single-pass mode: restore the original nearest-point progression.
          // The index only moves forward; after reaching the final point, the
          // horizon is padded with that point so NMPC holds the endpoint.
          traj_start_id = std::min(traj_start_id, path_size - 1);
          size_t closest_id = traj_start_id;
          double min_dist_sq = std::numeric_limits<double>::max();
          for (size_t i = traj_start_id; i < path_size; ++i)
          {
            const double dx = x_state_now(0) - current_ref_traj.poses[i].pose.position.x;
            const double dy = x_state_now(1) - current_ref_traj.poses[i].pose.position.y;
            const double dist_sq = dx * dx + dy * dy;
            if (dist_sq < min_dist_sq)
            {
              min_dist_sq = dist_sq;
              closest_id = i;
            }
          }
          if (min_dist_sq < 1.0)
            traj_start_id = closest_id;

          for (int i = 0; i < nmpc_pred_hori_num; ++i)
          {
            const size_t index = std::min(traj_start_id + static_cast<size_t>(i), path_size - 1);
            traj_hori.poses.push_back(current_ref_traj.poses[index]);
          }
        }
      }
      traj_horizon_pub_.publish(traj_hori);
      assert(traj_hori.poses.size() == nmpc_pred_hori_num);

      for (size_t i = 0; i < traj_hori.poses.size(); i++)
      {
        ct::core::StateVector<usv::USVDynamicModel::STATE_DIM> x_ref;
        x_ref.setZero();
        x_ref(0) = traj_hori.poses[i].pose.position.x;
        x_ref(1) = traj_hori.poses[i].pose.position.y;
        double ref_yaw = tf::getYaw(traj_hori.poses[i].pose.orientation);
        x_ref(2) = cos(ref_yaw);  // yaw
        x_ref(3) = sin(ref_yaw);
        x_ref(4) = ref_yaw;

        ct::core::ControlVector<usv::USVDynamicModel::CONTROL_DIM> u_ref;
        u_ref.setZero();

        double t = time_interval * i;
        x_ref_traj.push_back(x_ref, t, true);
        u_ref_traj.push_back(u_ref, t, true);
      }
      auto t1 = ros::Time::now().toNSec();
      //更新GP数据
      VectorXd gpPoints(8);
      if (t_opt_pub > 0.0)
      {
        double dt = current_odom.header.stamp.toSec() - t_opt_pub;
        std::cout<<"dt between opt control and odom time:"<<dt<<endl;
        opt_tracker_->getGPTrainingPoints(x_state_last, x_state_now, control_opt, dt, gpPoints);
        std_msgs::Float64MultiArray acc_error_msg;
        acc_error_msg.data.push_back(gpPoints(0));
        acc_error_msg.data.push_back(gpPoints(1));
        acc_error_msg.data.push_back(gpPoints(2));
        acc_error_msg.data.push_back(gpPoints(3));
        acc_error_msg.data.push_back(gpPoints(4));
        acc_error_msg.data.push_back(gpPoints(5));
        acc_error_msg.data.push_back(gpPoints(6));
        acc_error_msg.data.push_back(gpPoints(7));
        acc_error_pub_.publish(acc_error_msg);
        std_msgs::Float64MultiArray track_error_msg;
        track_error_msg.data.push_back(x_ref_last(0) - x_state_now(0));
        track_error_msg.data.push_back(x_ref_last(1) - x_state_now(1));
        track_error_pub_.publish(track_error_msg);
      }
      auto t2 = ros::Time::now().toNSec();
      //更新nmpc求解器
      opt_tracker_->update_mpc_tracker(x_state_now, x_ref_traj, u_ref_traj);

      //获取当前最优控制
      control_opt = opt_tracker_->getMPCControl(x_state_now, x_pred_traj);
      auto t3 = ros::Time::now().toNSec();
      std_msgs::Float64MultiArray solution_time_msg;
      solution_time_msg.data.push_back((t2 - t1) / 1e6);
      solution_time_msg.data.push_back((t3 - t2) / 1e6);
      solution_time_pub_.publish(solution_time_msg);

      visPredTraj(x_pred_traj);

      if (!control_opt.allFinite())
      {
        ROS_ERROR_THROTTLE(1.0, "NMPC returned a non-finite thrust-force rate; stopping thrusters");
        stopThrusters();
        rate.sleep();
        continue;
      }

      // The NMPC inputs are the rates [N/s] of the two augmented thrust-force
      // states. Integrate the complete feedback law over one MPC interval,
      // then invert the VRX GLF before publishing normalized commands.
      for (Eigen::Index i = 0; i < control_state.rows(); ++i)
      {
        control_opt(i) = std::max(-max_thrust_force_rate,
                                  std::min(max_thrust_force_rate, control_opt(i)));
        control_state(i) += control_opt(i) * time_interval;
        control_state(i) = std::max(min_thrust_force,
                                    std::min(max_thrust_force, control_state(i)));
      }
      const double left_command = opt_tracker_->thrustForceToCommand(control_state(0));
      const double right_command = opt_tracker_->thrustForceToCommand(control_state(1));
      publishThrustCommands(left_command, right_command);
      //记录发布最优控制的时间及当前状态
      x_state_last = x_state_now;
      x_ref_last = x_ref_traj[0];
      t_opt_pub = ros::Time::now().toSec();
      // auto t2 = ros::Time::now().toNSec();
      // cout<<"control loop spend"<<t2-t1<<" ns "<<endl;
      rate.sleep();
    }
  }

  void publishThrustCommands(const double& left, const double& right)
  {
    std_msgs::Float32 left_msg;
    std_msgs::Float32 right_msg;
    left_msg.data = static_cast<float>(left);
    right_msg.data = static_cast<float>(right);
    left_thrust_pub_.publish(left_msg);
    right_thrust_pub_.publish(right_msg);
  }

  void stopThrusters()
  {
    control_state.setZero();
    publishThrustCommands(0.0, 0.0);
  }

  void visPredTraj(const ct::core::StateVectorArray<usv::USVDynamicModel::STATE_DIM>& traj_predict_vec_)
  {
    if (traj_predict_pub_.getNumSubscribers() == 0)
      return;
    nav_msgs::Path traj;
    traj.header.frame_id = "odom";
    traj.header.stamp = ros::Time::now();
    traj.poses.reserve(traj_predict_vec_.size());

    for (size_t i = 0; i < traj_predict_vec_.size(); i++)
    {
      auto pos_temp = traj_predict_vec_[i];
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "odom";
      p.pose.position.x = pos_temp(0);
      p.pose.position.y = pos_temp(1);
      p.pose.position.z = 0.05;
      traj.poses.push_back(p);
    }
    traj_predict_pub_.publish(traj);
  }

  nav_msgs::Path processPathViaMinimumSnap(nav_msgs::Path pathIn, double vel_max,double acc_max)
  {

    MatrixXd waypoints(3, pathIn.poses.size());
    for (size_t i = 0; i < pathIn.poses.size(); i++)
    {
      waypoints(0, i) = pathIn.poses[i].pose.position.x;
      waypoints(1, i) = pathIn.poses[i].pose.position.y;
      if(i < pathIn.poses.size() - 1)
      {
        double dx = pathIn.poses[i + 1].pose.position.x - pathIn.poses[i].pose.position.x;
        double dy = pathIn.poses[i + 1].pose.position.y - pathIn.poses[i].pose.position.y;
        double theta = atan2(dy, dx);
        waypoints(2, i) = theta;
      }else{
        waypoints(2, i) = waypoints(2, i - 1);
      }
    }

    auto ts = allocateTime(waypoints,
                      vel_max,
                      acc_max);
    
    Matrix3d initialState, finalState;
    Eigen::Matrix<double, 3, 4> initialSnapState, finalSnapState;
    initialState.setZero();
    finalState.setZero();
    initialState.col(0) << waypoints.leftCols<1>();
    finalState.col(0) << waypoints.rightCols<1>();

    initialSnapState << initialState, (Eigen::Matrix<double, 3, 1>()<<vel_max*cos(initialState(2)), vel_max*sin(initialState(2)), 0).finished();
    finalSnapState << finalState, (Eigen::Matrix<double, 3, 1>()<<vel_max*cos(finalState(2)), vel_max*sin(finalState(2)), 0).finished();
    

    traj_optimizer_->reset(initialSnapState, finalSnapState, waypoints.cols() - 1);
    traj_optimizer_->generate(waypoints, ts);
    traj_optimizer_->getTraj(minSnapTraj);

    // Convert the trajectory back to a Path message
    nav_msgs::Path pathOut;
    pathOut.header = pathIn.header;
    cout<<"Trajectory time cost in total: "<<ts.sum()<<" seconds"<<endl;
    int step = static_cast<int>(ts.sum()/time_interval);
    for(int i = 0; i < step; i++){
      double dt = i * time_interval;
      Vector3d pos = minSnapTraj.getPos(dt);
      geometry_msgs::PoseStamped p;
      p.header = pathIn.header;
      p.pose.position.x = pos(0);
      p.pose.position.y = pos(1);
      p.pose.position.z = 0.05;
      pathOut.poses.push_back(p);
    }
    cout<<"start velocity: "<<minSnapTraj.getVel(0.0)<<" m/s"<<endl;
    cout<<"End velocity: "<<minSnapTraj.getVel(step * time_interval)<<" m/s"<<endl;

    pathOut = calculatePathYaw(pathOut);
    return pathOut;
  }

/* TODO_1:增加minimum snap 轨迹处理 */
  nav_msgs::Path processPath(nav_msgs::Path pathIn, double vel_max)
  {
    pathIn = calculatePathYaw(pathIn);
    pathIn = fixPathDensity(pathIn, vel_max);
    pathIn = smoothPath(pathIn);
    return pathIn;
  }


  VectorXd allocateTime(const MatrixXd &wayPs,
                      double vel,
                      double acc)
  {
    int N = (int)(wayPs.cols()) - 1;
    VectorXd durations(N);
    if (N > 0)
    {
        Vector3d p0, p1;
        double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;
        for (int k = 0; k < N; k++)
        {
            p0 = wayPs.col(k);
            p1 = wayPs.col(k + 1);
            D = (p1 - p0).norm();

            acct = vel / acc;
            accd = (acc * acct * acct / 2);
            dcct = vel / acc;
            dccd = acc * dcct * dcct / 2;

            if (D < accd + dccd)
            {
                t1 = sqrt(acc * D) / acc;
                t2 = (acc * t1) / acc;
                dtxyz = t1 + t2;
            }
            else
            {
                t1 = acct;
                t2 = (D - accd - dccd) / vel;
                t3 = dcct;
                dtxyz = t1 + t2 + t3;
            }

            durations(k) = dtxyz;
        }
    }

    return durations;
  }

  nav_msgs::Path calculatePathYaw(nav_msgs::Path pathIn)
  {
    int length = pathIn.poses.size();
    if (length <= 1)
    {
      if (length == 1)
        pathIn.poses[0].pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
      return pathIn;
    }

    for (int i = 0; i < length - 1; ++i)
    {
      double dx = pathIn.poses[i + 1].pose.position.x - pathIn.poses[i].pose.position.x;
      double dy = pathIn.poses[i + 1].pose.position.y - pathIn.poses[i].pose.position.y;
      double theta = atan2(dy, dx);
      pathIn.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    }

    pathIn.poses.back().pose.orientation = pathIn.poses[length - 2].pose.orientation;

    return pathIn;
  }

  nav_msgs::Path smoothPath(nav_msgs::Path path)
  {
    if (path.poses.size() <= 2)
      return path;

    double weight_data = 0.45;
    double weight_smooth = 0.4;
    double tolerance = 0.05;

    nav_msgs::Path smoothPath_out = path;

    double change = tolerance;
    double xtemp, ytemp;
    int nIterations = 0;

    int size = path.poses.size();

    while (change >= tolerance)
    {
      change = 0.0;
      for (int i = 1; i < size - 1; i++)
      {
        xtemp = smoothPath_out.poses[i].pose.position.x;
        ytemp = smoothPath_out.poses[i].pose.position.y;

        smoothPath_out.poses[i].pose.position.x +=
            weight_data * (path.poses[i].pose.position.x - smoothPath_out.poses[i].pose.position.x);
        smoothPath_out.poses[i].pose.position.y +=
            weight_data * (path.poses[i].pose.position.y - smoothPath_out.poses[i].pose.position.y);

        smoothPath_out.poses[i].pose.position.x +=
            weight_smooth * (smoothPath_out.poses[i - 1].pose.position.x + smoothPath_out.poses[i + 1].pose.position.x -
                             (2.0 * smoothPath_out.poses[i].pose.position.x));
        smoothPath_out.poses[i].pose.position.y +=
            weight_smooth * (smoothPath_out.poses[i - 1].pose.position.y + smoothPath_out.poses[i + 1].pose.position.y -
                             (2.0 * smoothPath_out.poses[i].pose.position.y));

        change += fabs(xtemp - smoothPath_out.poses[i].pose.position.x);
        change += fabs(ytemp - smoothPath_out.poses[i].pose.position.y);
      }
      nIterations++;
    }

    return smoothPath_out;
  }

  nav_msgs::Path fixPathDensity(nav_msgs::Path path, double vel_max)
  {
    double _pathResolution = vel_max * time_interval;  //这里应该给实际跟踪的最大速度
    if (path.poses.size() == 0)
      return path;

    double dis = 0, ang = 0;
    double margin = _pathResolution * 0.01;
    double remaining = 0;
    int nPoints = 0;

    nav_msgs::Path fixedPath = path;
    fixedPath.poses.clear();
    fixedPath.poses.push_back(path.poses[0]);

    size_t start = 0, next = 1;
    while (next < path.poses.size())
    {
      dis += hypot(path.poses[next].pose.position.x - path.poses[next - 1].pose.position.x,
                   path.poses[next].pose.position.y - path.poses[next - 1].pose.position.y) +
             remaining;
      ang = atan2(path.poses[next].pose.position.y - path.poses[start].pose.position.y,
                  path.poses[next].pose.position.x - path.poses[start].pose.position.x);

      if (dis < _pathResolution - margin)
      {
        next++;
        remaining = 0;
      }
      else if (dis > (_pathResolution + margin))
      {
        geometry_msgs::PoseStamped point_start = path.poses[start];
        nPoints = dis / _pathResolution;
        for (int j = 0; j < nPoints; j++)
        {
          point_start.pose.position.x = point_start.pose.position.x + _pathResolution * cos(ang);
          point_start.pose.position.y = point_start.pose.position.y + _pathResolution * sin(ang);
          point_start.pose.orientation = tf::createQuaternionMsgFromYaw(ang);
          fixedPath.poses.push_back(point_start);
        }
        remaining = dis - nPoints * _pathResolution;
        start++;
        path.poses[start].pose.position = point_start.pose.position;
        dis = 0;
        next++;
      }
      else
      {
        dis = 0;
        remaining = 0;
        fixedPath.poses.push_back(path.poses[next]);
        next++;
        start = next - 1;
      }
    }

    return fixedPath;
  }

private:
  /* ==State Estimation== */
  mutex odom_mtx, traj_mtx, control_mtx;
  Eigen::Matrix<double, usv::USVDynamicModel::CONTROL_DIM, 1> control_state;

  Dsp::Filter* filter;
  std::deque<Eigen::Vector3d> vel_history;
  int numSamples = 10;
  double lamda_learning;

  void initFilters(int sample_rate, double cutoff_freq)
  {
    // Design a 4th order Butterworth low-pass filter
    filter = new Dsp::SmoothedFilterDesign
      <Dsp::Butterworth::Design::LowPass <4>, 3, Dsp::DirectFormII> (1024);
    // Initialize the filter state
    Dsp::Params params;
    params[0] = sample_rate; // sample rate
    params[1] = 4; // order
    params[2] = cutoff_freq; // cut-off frequency
    filter->setParams(params);

  }

  void getFilterVec(const Eigen::Vector3d& vel_raw, Eigen::Vector3d& vel_filt)
  {
    vel_history.push_back(vel_raw);
    if (vel_history.size() < numSamples)
    {
      vel_filt = vel_raw;
      return;
    }

    auto it = vel_history.begin();
    float* samples[3];
    samples[0] = new float[numSamples];//surge velocity
    samples[1] = new float[numSamples];//sway velocity
    samples[2] = new float[numSamples];//yaw rate
    for (int i = 0; i < numSamples; i++)
    {
      samples[0][i] = static_cast<float>((*it)(0));
      samples[1][i] = static_cast<float>((*it)(1));
      samples[2][i] = static_cast<float>((*it)(2));
      it++;
    }

    filter->process(numSamples, samples);
    vel_filt(0) = samples[0][numSamples - 1];
    vel_filt(1) = samples[1][numSamples - 1];
    vel_filt(2) = samples[2][numSamples - 1];

    vel_history.pop_front();
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "usv_opt_tracker");
  ros::NodeHandle nh;
  ROS_INFO("\033[1;32m----> USV Opt Track Process Started.\033[0m");
  ControlInterface ci(nh);

  std::thread controlThread(&ControlInterface::controller_loop, &ci);

  ros::spin();

  controlThread.join();

  return 0;
}
