/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <uav_system_ros.h>

#include <rosgraph_msgs/Clock.h>

#include <geometry_msgs/PoseArray.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_multirotor_simulator/multirotor_simulatorConfig.h>

#include <KDTreeVectorOfVectorsAdaptor.h>
#include <Eigen/Dense>

#include "ueds-connector/drone-controller.h"
#include "ueds-connector/game-mode-controller.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

//}

namespace mrs_multirotor_simulator
{

typedef std::vector<Eigen::VectorXd> my_vector_of_vectors_t;

/* class MultirotorSimulator //{ */

class MultirotorSimulator : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_;

  // | ------------------------- params ------------------------- |

  double _simulation_rate_;

  ros::Time  sim_time_;
  std::mutex mutex_sim_time_;

  double _clock_min_dt_;

  std::string _world_frame_name_;

  // | ------------------------- timers ------------------------- |

  ros::WallTimer timer_main_;
  void           timerMain(const ros::WallTimerEvent& event);

  ros::WallTimer timer_status_;
  void           timerStatus(const ros::WallTimerEvent& event);

  // | ------------------------ rtf check ----------------------- |

  double    actual_rtf_ = 1.0;
  ros::Time last_sim_time_status_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<rosgraph_msgs::Clock>     ph_clock_;
  mrs_lib::PublisherHandler<geometry_msgs::PoseArray> ph_poses_;

  // | ------------------------- system ------------------------- |

  std::vector<std::unique_ptr<UavSystemRos>> uavs_;



  // | -------------------------- time -------------------------- |

  ros::Time last_published_time_;

  // | ------------------------- methods ------------------------ |

  void handleCollisions(void);

  void publishPoses(void);

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                       mutex_drs_;
  typedef mrs_multirotor_simulator::multirotor_simulatorConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>             Drs_t;
  boost::shared_ptr<Drs_t>                                     drs_;
  void                                                         callbackDrs(mrs_multirotor_simulator::multirotor_simulatorConfig& config, uint32_t level);
  DrsConfig_t                                                  drs_params_;
  std::mutex                                                   mutex_drs_params_;

  // | ------------------------- unreal sim stuff ------------------------- |

  std::mutex uavs_mutex;
  std::mutex ueds_controllers_mutex;
  bool is_unreal_used_;
  std::unique_ptr<ueds_connector::GameModeController> ueds_game_controller_;
  std::vector<std::unique_ptr<ueds_connector::DroneController>> ueds_uavs_controllers_;
  ueds_connector::Coordinates ueds_world_frame_;

  ros::Timer timer_ueds_pose_;
  void uedsSetLocationAndRotationTimer(void);
  ros::Timer timer_ueds_lidar_;
  ros::Publisher pub_lidar_;
  void uedsPublishLidarTimer();
  ros::Timer timer_ueds_camera_;
  ros::Publisher pub_camera_;
  void uedsPublishCameraTimer();

  std::vector<std::string> uav_names;
};

//}

/* onInit() //{ */

void MultirotorSimulator::onInit() {

  is_initialized_ = false;

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  if (!(nh_.hasParam("/use_sim_time"))) {
    nh_.setParam("/use_sim_time", true);
  }

  srand(time(NULL));

  sim_time_            = ros::Time(0);
  last_published_time_ = ros::Time(0);

  mrs_lib::ParamLoader param_loader(nh_, "MultirotorSimulator");

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");
  param_loader.addYamlFileFromParam("config_uavs");

  param_loader.loadParam("simulation_rate", _simulation_rate_);
  param_loader.loadParam("realtime_factor", drs_params_.realtime_factor);
  param_loader.loadParam("collisions/enabled", drs_params_.collisions_enabled);
  param_loader.loadParam("collisions/crash", drs_params_.collisions_crash);
  param_loader.loadParam("collisions/rebounce", drs_params_.collisions_rebounce);
  param_loader.loadParam("frames/world/name", _world_frame_name_);

  double clock_rate;
  param_loader.loadParam("clock_rate", clock_rate);

  drs_params_.paused = false;

  //std::vector<std::string> uav_names;

  param_loader.loadParam("uav_names", uav_names);

  for (size_t i = 0; i < uav_names.size(); i++) {

    std::string uav_name = uav_names[i];

    ROS_INFO("[MultirotorSimulator]: initializing '%s'", uav_name.c_str());

    uavs_.push_back(std::make_unique<UavSystemRos>(nh_, uav_name));
  }

  // | ------------------------- unreal sim stuff ------------------------- |

  ROS_ERROR("KUNDA");

  //TODO load param
  is_unreal_used_ = true;

  if(is_unreal_used_){
    ueds_game_controller_ = std::make_unique<ueds_connector::GameModeController>(LOCALHOST, 8000);
    bool connect_result = ueds_game_controller_->Connect();
    if (connect_result != 1) {
      ROS_ERROR("Unreal: Error connecting to game mode controller. connect_result was %d", connect_result);
      is_unreal_used_ = false;
    }
  }

  if(is_unreal_used_){
    for (size_t i = 0; i < uav_names.size(); i++) {
      const auto [resSpawn, port] = ueds_game_controller_->SpawnDrone();
      if(!resSpawn){
        ROS_ERROR("Unreal: SpawnDrone FAIL");
        is_unreal_used_ = false;
        break;
      }

      ROS_INFO("Unreal: %s spawn.", uav_names[i].c_str());

      ueds_uavs_controllers_.push_back(std::make_unique<ueds_connector::DroneController>(LOCALHOST, port));
      auto connect_result = ueds_uavs_controllers_[i]->Connect();
      if (connect_result != 1) {
        ROS_ERROR("Unreal: %s - Error connecting to drone controller. connect_result was %d", uav_names[i].c_str(), connect_result);
        is_unreal_used_ = false;
        break;
      }
      if(i == 0){
        const auto [res, location] = ueds_uavs_controllers_[i]->GetLocation();
        if (!res) {
            ROS_ERROR("Unreal: %s - DroneError: getting location", uav_names[i].c_str());
            return;
        } else {
            ueds_world_frame_ = location;
            ROS_INFO("ueds_world_frame_: x[%lf] y[%lf] z[%lf]",ueds_world_frame_.x, ueds_world_frame_.y, ueds_world_frame_.z);
        }
      }

    }

    timer_ueds_pose_ = nh_.createTimer(ros::Duration(1.0 / 50.0), std::bind(&MultirotorSimulator::uedsSetLocationAndRotationTimer, this));

    timer_ueds_lidar_ = nh_.createTimer(ros::Duration(1.0 / 10.0), std::bind(&MultirotorSimulator::uedsPublishLidarTimer, this));
    pub_lidar_ = nh_.advertise<sensor_msgs::PointCloud2>("ueds/"+uav_names[0]+"/lidar",1);

    // timer_ueds_camera_ = nh_.createTimer(ros::Duration(1.0/1.0), std::bind(&MultirotorSimulator::uedsPublishCameraTimer, this));
    // pub_camera_ = nh_.advertise<sensor_msgs::CompressedImage>("ueds/"+uav_names[0]+"/camera/image/compressed",1);
  }



  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&MultirotorSimulator::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MultirotorSimulator]: could not load all parameters!");
    ros::shutdown();
  }

  _clock_min_dt_ = 1.0 / clock_rate;

  // | ----------------------- publishers ----------------------- |

  ph_clock_ = mrs_lib::PublisherHandler<rosgraph_msgs::Clock>(nh_, "clock_out", 10, false);

  ph_poses_ = mrs_lib::PublisherHandler<geometry_msgs::PoseArray>(nh_, "uav_poses_out", 10, false);

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createWallTimer(ros::WallDuration(1.0 / (_simulation_rate_ * drs_params_.realtime_factor)), &MultirotorSimulator::timerMain, this);

  timer_status_ = nh_.createWallTimer(ros::WallDuration(1.0), &MultirotorSimulator::timerStatus, this);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[MultirotorSimulator]: initialized");
}

//}


// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void MultirotorSimulator::timerMain([[maybe_unused]] const ros::WallTimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MultirotorSimulator]: main timer spinning");

  double simulation_step_size = 1.0 / _simulation_rate_;

  // step the time
  sim_time_ = sim_time_ + ros::Duration(simulation_step_size);

  std::unique_lock<std::mutex> lock(uavs_mutex);

  for (size_t i = 0; i < uavs_.size(); i++) {
    uavs_[i]->makeStep(simulation_step_size);
  }

  publishPoses();

  handleCollisions();
  
  lock.unlock();



  // | ---------------------- publish time ---------------------- |

  if ((sim_time_ - last_published_time_).toSec() >= _clock_min_dt_) {

    rosgraph_msgs::Clock ros_time;

    ros_time.clock.fromSec(sim_time_.toSec());

    ph_clock_.publish(ros_time);

    last_published_time_ = sim_time_;
  }
}

//}

/* timeStatus() //{ */

void MultirotorSimulator::timerStatus([[maybe_unused]] const ros::WallTimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  auto sim_time   = mrs_lib::get_mutexed(mutex_sim_time_, sim_time_);
  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  ros::Duration last_sec_sim_dt = sim_time - last_sim_time_status_;

  last_sim_time_status_ = sim_time;

  double last_sec_rtf = last_sec_sim_dt.toSec() / 1.0;

  actual_rtf_ = 0.9 * actual_rtf_ + 0.1 * last_sec_rtf;

  ROS_INFO("[MultirotorSimulator]: desired RTF = %.2f, actual RTF = %.2f", drs_params.realtime_factor, actual_rtf_);
}

//}

/* callbackDrs() //{ */

void MultirotorSimulator::callbackDrs(mrs_multirotor_simulator::multirotor_simulatorConfig& config, [[maybe_unused]] uint32_t level) {

  {
    // | ----------------- pausing the simulation ----------------- |

    auto old_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

    if (!old_params.paused && config.paused) {
      timer_main_.stop();
    } else if (old_params.paused && !config.paused) {
      timer_main_.start();
    }
  }

  // | --------------------- save the params -------------------- |

  {
    std::scoped_lock lock(mutex_drs_params_);

    drs_params_ = config;
  }

  // | ----------------- set the realtime factor ---------------- |

  timer_main_.setPeriod(ros::WallDuration(1.0 / (_simulation_rate_ * config.realtime_factor)), true);

  ROS_INFO("[MultirotorSimulator]: DRS updated params");
}

//}

/* handleCollisions() //{ */

void MultirotorSimulator::handleCollisions(void) {

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  std::vector<Eigen::VectorXd> poses;

  for (size_t i = 0; i < uavs_.size(); i++) {
    poses.push_back(uavs_[i]->getPose());
  }

  typedef KDTreeVectorOfVectorsAdaptor<my_vector_of_vectors_t, double> my_kd_tree_t;

  my_kd_tree_t mat_index(3, poses, 10);

  std::vector<nanoflann::ResultItem<int, double>> indices_dists;

  std::vector<Eigen::Vector3d> forces;

  for (size_t i = 0; i < uavs_.size(); i++) {
    forces.push_back(Eigen::Vector3d::Zero());
  }

  for (size_t i = 0; i < uavs_.size(); i++) {

    MultirotorModel::State       state_1  = uavs_[i]->getState();
    MultirotorModel::ModelParams params_1 = uavs_[i]->getParams();

    nanoflann::RadiusResultSet<double, int> resultSet(3.0, indices_dists);

    mat_index.index->findNeighbors(resultSet, &state_1.x[0]);

    for (size_t j = 0; j < resultSet.m_indices_dists.size(); j++) {

      const size_t idx  = resultSet.m_indices_dists[j].first;
      const double dist = resultSet.m_indices_dists[j].second;

      if (idx == i) {
        continue;
      }

      MultirotorModel::State       state_2  = uavs_[idx]->getState();
      MultirotorModel::ModelParams params_2 = uavs_[idx]->getParams();

      const double crit_dist = params_1.arm_length + params_1.prop_radius + params_2.arm_length + params_2.prop_radius;

      const Eigen::Vector3d rel_pos = state_1.x - state_2.x;

      if (dist < crit_dist) {
        if (drs_params.collisions_crash) {
          uavs_[idx]->crash();
        } else {
          forces[i] += drs_params.collisions_rebounce * rel_pos.normalized() * params_1.mass * (params_2.mass / (params_1.mass + params_2.mass));
        }
      }
    }
  }

  for (size_t i = 0; i < uavs_.size(); i++) {
    uavs_[i]->applyForce(forces[i]);
  }
}

//}

/* publishPoses() //{ */

void MultirotorSimulator::publishPoses(void) {

  auto sim_time = mrs_lib::get_mutexed(mutex_sim_time_, sim_time_);

  geometry_msgs::PoseArray pose_array;

  pose_array.header.stamp    = sim_time;
  pose_array.header.frame_id = _world_frame_name_;

  for (size_t i = 0; i < uavs_.size(); i++) {

    auto state = uavs_[i]->getState();

    geometry_msgs::Pose pose;

    pose.position.x  = state.x[0];
    pose.position.y  = state.x[1];
    pose.position.z  = state.x[2];
    pose.orientation = mrs_lib::AttitudeConverter(state.R);

    pose_array.poses.push_back(pose);
  }

  ph_poses_.publish(pose_array);
}

void MultirotorSimulator::uedsSetLocationAndRotationTimer(void){
  if(is_unreal_used_){
    for (size_t i = 0; i < uavs_.size(); i++) {
      std::unique_lock<std::mutex> lc(uavs_mutex);
      MultirotorModel::State state = uavs_[i]->getState();
      Eigen::Vector3d rpy = state.R.eulerAngles(0, 1, 2)*180/M_PI;
      lc.unlock();

      //ROS_WARN("uavPose: %lf %lf %lf", state.x.x(), state.x.y(), state.x.z());

      std::unique_lock<std::mutex> lock(ueds_controllers_mutex);
      const auto [res, teleportedTo, rotatedTo, isHit, impactPoint] = ueds_uavs_controllers_[i]->SetLocationAndRotation(ueds_connector::Coordinates(
                                    ueds_world_frame_.x + state.x.x()*100,
                                    ueds_world_frame_.y - state.x.y()*100,
                                    ueds_world_frame_.z + state.x.z()*100),
                                    ueds_connector::Rotation(-rpy.y(),-rpy.z(), -rpy.x()));
      lock.unlock();

      if(isHit){
        lc.lock();
        if(!uavs_[i]->hasCrashed()){
          uavs_[i]->crash();
        }
        lc.unlock();
      }

    }
  }
}

void MultirotorSimulator::uedsPublishLidarTimer()
{ 
  int i = 0;
 
  bool res;
  std::vector<ueds_connector::LidarData> lidarData;
  ueds_connector::Coordinates start;

  std::unique_lock<std::mutex> lock(ueds_controllers_mutex);
  std::tie(res, lidarData, start) = ueds_uavs_controllers_[i]->GetLidarData();
  lock.unlock();

  
  if(!res){
      ROS_ERROR("Unreal: [uav %d] - ERROR getLidarData", i);
  }
  else {
  //ROS_INFO("Lidar Start: x[%lf] y[%lf] z[%lf]",x, start.y, start.z);
  sensor_msgs::PointCloud2 pcl_msg;
  
  //Modifier to describe what the fields are.
  sensor_msgs::PointCloud2Modifier modifier(pcl_msg);        
  modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::PointField::FLOAT32);
  //Msg header
  pcl_msg.header = std_msgs::Header();
  pcl_msg.header.stamp = ros::Time::now();
  pcl_msg.header.frame_id = uav_names[i] +"/world_origin";
  pcl_msg.height = 1; //unordered 1D data array points cloud
  pcl_msg.width = lidarData.size(); //360; //num_of_points
  pcl_msg.is_dense = true;
  //Total number of bytes per point
  pcl_msg.point_step = 16;
  pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
  pcl_msg.data.resize(pcl_msg.row_step);
  //Iterators for PointCloud msg
  sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iterIntensity(pcl_msg, "intensity");
  
  for(const ueds_connector::LidarData &i : lidarData){
      tf::Vector3 dir = tf::Vector3(i.directionX, i.directionY, i.directionZ);
      dir = dir.normalize() * (i.distance/100.0);
      tf::Vector3 lidarTransform = tf::Vector3(
          (start.x - ueds_world_frame_.x)/100,
          (start.y - ueds_world_frame_.y)/100,
          (start.z - ueds_world_frame_.z)/100
      );
      *iterX = lidarTransform.x() + dir.x();
      *iterY = -lidarTransform.y() - dir.y(); //convert left-hand to right-hand coordinates
      *iterZ = lidarTransform.z() + dir.z();
      *iterIntensity = i.distance;
      // ROS_WARN("UEDworldStart: %lf %lf %lf", ueds_world_frame_.x, ueds_world_frame_.y, ueds_world_frame_.z);
      // ROS_WARN("UEDstartStart: %lf %lf %lf", start.x, start.y, start.z);
      // ROS_WARN("UEDSlidarStart: %lf %lf %lf", lidarTransform.x(), lidarTransform.y(), lidarTransform.z());
      //ROS_WARN("lidarStart: %f %f %f", *iterX, *iterY, *iterZ);
      ++iterX;
      ++iterY;
      ++iterZ;
      ++iterIntensity;
  }
  // *iterX = 10;
  // *iterY = 0; //convert left-hand to right-hand coordinates
  // *iterZ = 0;
  // *iterIntensity = 10;
  pub_lidar_.publish(pcl_msg);

  }
}

void MultirotorSimulator::uedsPublishCameraTimer(){
  int i = 0;

  bool res;
  std::vector<unsigned char> cameraData;
  uint32_t size;

  std::unique_lock<std::mutex> lock(ueds_controllers_mutex);
  std::tie(res, cameraData, size) = ueds_uavs_controllers_[i]->GetCameraData();
  lock.unlock();

  // sensor_msgs::Image img_msg;
  // img_msg.data.resize(size);
  // std::copy(cameraData.begin(), cameraData.end(), std::back_inserter(img_msg.data));
  if(res){
    sensor_msgs::CompressedImage img_msg;
    img_msg.header.stamp = ros::Time::now();
    img_msg.format = "jpeg";

    // std::vector<u_char> data;
    //     // Example: Read JPEG binary data from a file
    // std::ifstream file("/home/jan/Documents/BachelorThesis/ueds-connector/cmake-build-debug/examples/cli/testImage.jpeg", std::ios::binary);
    
    // u_char c;
    // while (file >> c)
    // {
    //   data.push_back(c);
    // }
    // file.close();

    img_msg.data = cameraData;
    
    pub_camera_.publish(img_msg);
  }else{
    ROS_ERROR("Unreal: can not send camera msg");
  }

}

//}

}  // namespace mrs_multirotor_simulator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_multirotor_simulator::MultirotorSimulator, nodelet::Nodelet)
