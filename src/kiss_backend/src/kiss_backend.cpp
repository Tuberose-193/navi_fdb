//Based on the example showing KISS-Matcher registration with animated transformation visualization.
//202503 beastsempai213 330709390@qq.com
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <kiss_matcher/KISSMatcher.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_eigen/tf2_eigen.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
using PointType = pcl::PointXYZ;

//gicp config
struct GICPConfig {
  int num_threads_               = 4;
  int correspondence_randomness_ = 20;
  int max_num_iter_              = 20;
  double voxel_res_                = 0.1;
  double max_corr_dist_              = 1.0;
  double scale_factor_for_corr_dist_ = 5.0;
  double overlap_threshold_          = 80.0;
}gicp_config_;
//regist score
size_t num_inliers_threshold_    = 15;
size_t high_num_inliers_threshold_ = 5;
size_t low_num_inliers_threshold_    = 5;
size_t rot_num_inliers_threshold_ = 10;
// Registration Output
struct RegOutput {
  bool is_valid_        = false;
  bool is_converged_    = false;
  double score_         = std::numeric_limits<double>::max();
  double overlapness_   = 0.0;
  Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
};

class KissBackend : public rclcpp::Node {

 public:
  KissBackend() : Node("kiss_beckend_node") {
    callback_group = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
    declareAndGetParameters();

    // 初始化局部地图
    local_map_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    voxel_filter_.setLeafSize(
        get_parameter("input.voxel_size").as_double(),
        get_parameter("input.voxel_size").as_double(),
        get_parameter("input.voxel_size").as_double()
    );
    pass.setFilterFieldName("z");         // 设置过滤时所考虑的坐标字段（这里为Z轴）
    pass.setFilterLimits(-std::numeric_limits<float>::max(), get_parameter("input.passthrough_height").as_double());  // 设置坐标过滤的范围（这里为小于等于2米）
    
    source_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/src_cloud", 10);
    target_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tgt_cloud", 10);
    map_frame_pointcloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/map_frame", 10);//debug

    src_cloud_.reset(new pcl::PointCloud<PointType>());
    tgt_cloud_.reset(new pcl::PointCloud<PointType>());
    coarse_aligned_.reset(new pcl::PointCloud<PointType>());
    aligned_.reset(new pcl::PointCloud<PointType>());
    loadPointClouds();

    global_reg_handler_ = std::make_shared<kiss_matcher::KISSMatcher>(kiss_config);
    local_reg_handler_  = std::make_shared<small_gicp::RegistrationPCL<PointType, PointType>>();
    local_reg_handler_->setNumThreads(gicp_config_.num_threads_);
    local_reg_handler_->setCorrespondenceRandomness(gicp_config_.correspondence_randomness_);
    local_reg_handler_->setMaxCorrespondenceDistance(gicp_config_.max_corr_dist_);
    local_reg_handler_->setVoxelResolution(gicp_config_.voxel_res_);
    local_reg_handler_->setRegistrationType("GICP");  // "VGICP" or "GICP"

    local_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    get_parameter("input.input_pointcloud_topic").as_string(), 10,
    std::bind(&KissBackend::cloudCallback, this, std::placeholders::_1),rclcpp::SubscriptionOptions());
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&KissBackend::odomCallback, this, std::placeholders::_1),rclcpp::SubscriptionOptions());
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    global_regist_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                        std::bind(&KissBackend::pubMapOdomTF, this),callback_group);
    pointclouid_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&KissBackend::publish_pointcloud, this),callback_group);

    
    
    // computeRegistration();
    // animateTransformation();
  }

 private:
 //input
 rclcpp::CallbackGroup::SharedPtr callback_group;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::optional<Eigen::Matrix4f> latest_odom_;
  std::mutex odom_mutex_;
  std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> scans_queue_;
  std::deque<Eigen::Matrix4f> poses_queue_;
 
  rclcpp::TimerBase::SharedPtr global_regist_timer_;
  rclcpp::TimerBase::SharedPtr pointclouid_publish_timer_;
  pcl::PointCloud<PointType>::Ptr src_cloud_;
  pcl::PointCloud<PointType>::Ptr tgt_cloud_;
  pcl::PointCloud<PointType>::Ptr coarse_aligned_;
  pcl::PointCloud<PointType>::Ptr aligned_;
  pcl::PointCloud<PointType>::Ptr debug_cloud_;

  std::shared_ptr<kiss_matcher::KISSMatcher> global_reg_handler_                        = nullptr;
  std::shared_ptr<small_gicp::RegistrationPCL<PointType, PointType>> local_reg_handler_ = nullptr;
  std::chrono::steady_clock::time_point last_success_icp_time_;
  bool has_success_icp_time_ = false;
  Eigen::Matrix4d pose_  = Eigen::Matrix4d::Identity();

  std::string base_dir_;
  std::string src_pcd_path_;
  std::string tgt_pcd_path_;
  std::string src_pcd_topic_;
  double resolution_;
  kiss_matcher::KISSMatcherConfig kiss_config;
  double moving_rate_;
  double frame_rate_;
  double scale_factor_;
  Eigen::Matrix4f estimated_transform_ = Eigen::Matrix4f::Identity();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr source_publisher_, target_publisher_,map_frame_pointcloud_pub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_sub_;
  bool local_map_init_ok;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_{new pcl::PointCloud<pcl::PointXYZ>};
    // void localMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    //     pcl::fromROSMsg(*msg, *local_map_);
    // }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void declareAndGetParameters() {
    local_map_init_ok = false; //for init
    declare_parameter("input.need_voxel", false);
    declare_parameter("input.voxel_size", 0.2);
    declare_parameter("input.window_size", 15);
    declare_parameter("input.passthrough_height",2.5);
    declare_parameter("input.odom_frame", "odom");
    declare_parameter("input.lidar_frame", "lidar_link");
    declare_parameter("input.input_pointcloud_topic", "/livox/lidar/pointcloud");

    declare_parameter("base_dir", "");//useless now path is absolute
    declare_parameter("src_pcd_path", "");
    declare_parameter("src_pcd_topic", "low_local_map");
    declare_parameter("tgt_pcd_path", "/home/auto/25automapws/0329seglow005.pcd");
    declare_parameter("resolution", 0.1);
    declare_parameter("moving_rate", 10.0);
    declare_parameter("frame_rate", 10.0);
    declare_parameter("scale_factor", 1.0);

    // struct GICPConfig {
    //   int num_threads_               = 4;
    //   int correspondence_randomness_ = 20;
    //   int max_num_iter_              = 20;
    //   double voxel_res_                = 0.1;
    //   double max_corr_dist_              = 1.0;
    //   double scale_factor_for_corr_dist_ = 5.0;
    //   double overlap_threshold_          = 80.0;
    // }gicp_config_;
    declare_parameter("gicp_config.num_threads_", 4);
    declare_parameter("gicp_config.correspondence_randomness_", 20);
    declare_parameter("gicp_config.max_num_iter_", 20);
    declare_parameter("gicp_config.voxel_res_", 0.1);
    declare_parameter("gicp_config.max_corr_dist_", 1.0);
    declare_parameter("gicp_config.scale_factor_for_corr_dist_", 5.0);
    declare_parameter("gicp_config.overlap_threshold_", 80.0);
    gicp_config_.num_threads_ = get_parameter("gicp_config.num_threads_").as_int();
    gicp_config_.correspondence_randomness_ = get_parameter("gicp_config.correspondence_randomness_").as_int();
    gicp_config_.max_num_iter_ = get_parameter("gicp_config.max_num_iter_").as_int();
    gicp_config_.voxel_res_ = get_parameter("gicp_config.voxel_res_").as_double();
    gicp_config_.max_corr_dist_ = get_parameter("gicp_config.max_corr_dist_").as_double();
    gicp_config_.scale_factor_for_corr_dist_ = get_parameter("gicp_config.scale_factor_for_corr_dist_").as_double();
    gicp_config_.overlap_threshold_ = get_parameter("gicp_config.overlap_threshold_").as_double();
    RCLCPP_INFO(this->get_logger(),"gicp_config_: num_threads_:%d ,correspondence_randomness_:%d ,max_num_iter_:%d ,voxel_res_:%f ,max_corr_dist_:%f ,scale_factor_for_corr_dist_:%f ,overlap_threshold_:%f",gicp_config_.num_threads_,gicp_config_.correspondence_randomness_,gicp_config_.max_num_iter_,gicp_config_.voxel_res_,gicp_config_.max_corr_dist_,gicp_config_.scale_factor_for_corr_dist_,gicp_config_.overlap_threshold_);

      // KISSMatcherConfig(const float voxel_size         = 0.3,
      //               const float use_voxel_sampling = true,
      //               const float use_quatro         = false,
      //               const float thr_linearity      = 1.0,
      //               const int num_max_corr         = 5000,
      //               // Below params just works in general cases
      //               const float normal_r_gain = 3.0,
      //               const float fpfh_r_gain   = 5.0,
      //               // The smaller, more conservative
      //               const float robin_noise_bound_gain     = 1.0,
      //               const float solver_noise_bound_gain    = 0.75,
      //               const bool enable_noise_bound_clamping = true)
      declare_parameter("kiss_config.voxel_size",0.3);
      declare_parameter("kiss_config.use_voxel_sampling",true);
      declare_parameter("kiss_config.use_quatro",false);
      declare_parameter("kiss_config.thr_linearity",1.0);
      declare_parameter("kiss_config.num_max_corr",5000);
      declare_parameter("kiss_config.normal_r_gain",3.0);
      declare_parameter("kiss_config.fpfh_r_gain",5.0);
      declare_parameter("kiss_config.robin_noise_bound_gain",1.0);
      declare_parameter("kiss_config.solver_noise_bound_gain",0.75);
      declare_parameter("kiss_config.enable_noise_bound_clamping",true);
      declare_parameter("kiss_config.overlap_threshold_high",15.0);
      declare_parameter("kiss_config.overlap_threshold_low",10.0);
      kiss_matcher::KISSMatcherConfig config_(
      get_parameter("kiss_config.voxel_size").as_double(),
      get_parameter("kiss_config.use_voxel_sampling").as_bool() ,
      get_parameter("kiss_config.use_quatro").as_bool() ,
      get_parameter("kiss_config.thr_linearity").as_double() ,
      get_parameter("kiss_config.num_max_corr").as_int() ,
      get_parameter("kiss_config.normal_r_gain").as_double() ,
      get_parameter("kiss_config.fpfh_r_gain").as_double() ,
      get_parameter("kiss_config.robin_noise_bound_gain").as_double() ,
      get_parameter("kiss_config.solver_noise_bound_gain").as_double() ,
      get_parameter("kiss_config.enable_noise_bound_clamping").as_bool() );
      kiss_config = config_;
      RCLCPP_INFO(this->get_logger(),"kiss_config: voxel_size:%f, use_voxel_sampling_:%d, use_quatro_:%d, thr_linearity_:%f, num_max_corr_:%d, normal_radius_:%f, fpfh_radius_:%f, robin_noise_bound_gain_:%f, solver_noise_bound_gain_:%f, enable_noise_bound_clamping:%d",kiss_config.voxel_size_,kiss_config.use_voxel_sampling_,kiss_config.use_quatro_,kiss_config.thr_linearity_,kiss_config.num_max_corr_,
      kiss_config.normal_radius_,kiss_config.fpfh_radius_,kiss_config.robin_noise_bound_gain_,kiss_config.solver_noise_bound_gain_,
      get_parameter("kiss_config.enable_noise_bound_clamping").as_bool());

    get_parameter("base_dir", base_dir_);
    get_parameter("src_pcd_path", src_pcd_path_);
    get_parameter("src_pcd_topic", src_pcd_topic_);
    get_parameter("tgt_pcd_path", tgt_pcd_path_);
    // get_parameter("resolution", resolution_);
    get_parameter("moving_rate", moving_rate_);
    get_parameter("frame_rate", frame_rate_);
    get_parameter("scale_factor", scale_factor_);
  }
    bool getTransform(const std::string& target_frame, 
                     const std::string& source_frame,
                     Eigen::Matrix4f& output) {
        try {
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_->lookupTransform(
                    target_frame, 
                    source_frame,
                    tf2::TimePointZero);
            output = tf2::transformToEigen(transform.transform).matrix().cast<float>();
            return true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "TF lookup failed: %s", ex.what());
            return false;
        }
    }
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 转换ROS消息为PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);
        
        // 与最新里程计位姿一起处理
        std::lock_guard<std::mutex> lock(odom_mutex_);

        Eigen::Matrix4f lidar_to_odom;
        // if (!getTransform(get_parameter("input.odom_frame").as_string(), 
        //                  get_parameter("input.lidar_frame").as_string(), 
        //                  lidar_to_odom)) {
        //     return;
        // }
        geometry_msgs::msg::TransformStamped tf_livox_to_map;
        try{
            tf_livox_to_map = tf_buffer_->lookupTransform("odom", "lidar_link", tf2::TimePointZero);
        }catch(tf2::TransformException &ex){
            RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", ex.what());
            return; 
        }
        pcl_ros::transformPointCloud(*cloud, *cloud, tf_livox_to_map);

        //debug
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.frame_id = "odom";
        // output.header.stamp = livox_lidar_stamp_; 仿真
        output.header.stamp = this->now();
        map_frame_pointcloud_pub->publish(output);
        //debug

        if (latest_odom_.has_value()) {
            addScanToMap(cloud, latest_odom_.value());
        }
    }
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 存储最新里程计位姿
        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_ = convertOdomToMatrix(*msg);
    }
    Eigen::Matrix4f convertOdomToMatrix(const nav_msgs::msg::Odometry& odom) {
        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
        
        // 位置
        mat(0, 3) = odom.pose.pose.position.x;
        mat(1, 3) = odom.pose.pose.position.y;
        mat(2, 3) = odom.pose.pose.position.z;
        
        // 姿态（四元数转旋转矩阵）
        Eigen::Quaternionf q(
            odom.pose.pose.orientation.w,
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z
        );
        mat.block<3, 3>(0, 0) = q.toRotationMatrix();
        
        return mat;
    }

    void addScanToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan,
                     const Eigen::Matrix4f& odom_pose) {
        // // 坐标变换到世界系
        // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_scan(new pcl::PointCloud<pcl::PointXYZ>());
        // pcl::transformPointCloud(*scan, *transformed_scan, odom_pose);

        // 添加到滑动窗口
        scans_queue_.push_back(scan);
        poses_queue_.push_back(odom_pose);

        // 维护窗口大小
        const size_t max_size = get_parameter("input.window_size").as_int();
        if (scans_queue_.size() > max_size) {
            scans_queue_.pop_front();
            poses_queue_.pop_front();
        }
      local_map_init_ok = true;
        
    }

    //匹配之前调用
    //注意，这里的localmap是全局变量！
    void updateLocalMap() {
        local_map_->clear();
        
        // 合并滑动窗口内点云
        for (const auto& scan : scans_queue_) {
            *local_map_ += *scan;
        }

        // 降采样
        if(get_parameter("input.need_voxel").as_bool()==true)
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
          voxel_filter_.setInputCloud(local_map_);
          voxel_filter_.filter(*filtered);
          local_map_.swap(filtered);
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr low_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pass.setInputCloud(local_map_);  
        pass.filter(*low_filtered);
        local_map_.swap(low_filtered);
        
    }

  void loadPointClouds() {
    auto logger = get_logger();

    // if (pcl::io::loadPCDFile<pcl::PointXYZ>(base_dir_ + src_pcd_path_, *source_cloud_) == -1 ||
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(tgt_pcd_path_, *tgt_cloud_) == -1) {
      RCLCPP_ERROR(logger, "Failed to load PCD files.");
      return;
    }
    std::cout << "tgt_pcd_path" << tgt_pcd_path_<<std::endl;

    std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(*source_cloud_, *source_cloud_, indices);
    pcl::removeNaNFromPointCloud(*tgt_cloud_, *tgt_cloud_, indices);

    Eigen::Matrix4f scale_transform = Eigen::Matrix4f::Identity();
    scale_transform.block<3, 3>(0, 0) *= scale_factor_;
    // pcl::transformPointCloud(*source_cloud_, *source_cloud_, scale_transform);
    pcl::transformPointCloud(*tgt_cloud_, *tgt_cloud_, scale_transform);

    RCLCPP_INFO(logger, "PCD files loaded and downsampled successfully.");
    
  }

  std::vector<Eigen::Vector3f> convertCloudToEigenVector(
      const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    std::vector<Eigen::Vector3f> points;
    points.reserve(cloud.size());
    for (const auto& point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
      points.emplace_back(point.x, point.y, point.z);
    }
    return points;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr convertEigenVectorToCloud(
      const std::vector<Eigen::Vector3f>& points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->reserve(points.size());
    for (const auto& point : points) {
      cloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    }
    return cloud;
  }

//type UTILITY

inline void matrixEigenToTF2(const Eigen::Matrix3d &in, tf2::Matrix3x3 &out) {
  out.setValue(
      in(0, 0), in(0, 1), in(0, 2), in(1, 0), in(1, 1), in(1, 2), in(2, 0), in(2, 1), in(2, 2));
}

inline void matrixTF2ToEigen(const tf2::Matrix3x3 &in, Eigen::Matrix3d &out) {
  out << in[0][0], in[0][1], in[0][2], in[1][0], in[1][1], in[1][2], in[2][0], in[2][1], in[2][2];
}

inline pcl::PointCloud<PointType>::Ptr voxelize(const pcl::PointCloud<PointType> &cloud,
                                                float voxel_res) {
  pcl::VoxelGrid<PointType> voxelgrid;
  voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);

  pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>);

  cloud_ptr->reserve(cloud.size());
  cloud_out->reserve(cloud.size());
  *cloud_ptr = cloud;

  voxelgrid.setInputCloud(cloud_ptr);
  voxelgrid.filter(*cloud_out);
  return cloud_out;
}
template <typename T>
inline sensor_msgs::msg::PointCloud2 toROSMsg(const pcl::PointCloud<T> &cloud,
                                              const std::string &frame_id = "map") {
  sensor_msgs::msg::PointCloud2 cloud_ros;
  pcl::toROSMsg(cloud, cloud_ros);
  cloud_ros.header.frame_id = frame_id;
  return cloud_ros;
}
template <typename T>
inline std::vector<Eigen::Vector3f> convertCloudToVec(const pcl::PointCloud<T> &cloud) {
  std::vector<Eigen::Vector3f> vec;
  vec.reserve(cloud.size());
  for (const auto &pt : cloud.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    vec.emplace_back(pt.x, pt.y, pt.z);
  }
  return vec;
}
template <typename T>
inline pcl::PointCloud<T> transformPcd(const pcl::PointCloud<T> &cloud_in,
                                       const Eigen::Matrix4d &pose) {
  if (cloud_in.empty()) {
    return cloud_in;
  }
  pcl::PointCloud<T> cloud_out;
  pcl::transformPointCloud(cloud_in, cloud_out, pose);
  return cloud_out;
}
inline geometry_msgs::msg::PoseStamped eigenToPoseStamped(const Eigen::Matrix4d &pose,
                                                          const std::string &frame_id = "map") {
  tf2::Matrix3x3 mat_tf;
  matrixEigenToTF2(pose.block<3, 3>(0, 0), mat_tf);

  double roll, pitch, yaw;
  mat_tf.getRPY(roll, pitch, yaw);

  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);

  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id    = frame_id;
  msg.pose.position.x    = pose(0, 3);
  msg.pose.position.y    = pose(1, 3);
  msg.pose.position.z    = pose(2, 3);
  msg.pose.orientation.w = quat.w();
  msg.pose.orientation.x = quat.x();
  msg.pose.orientation.y = quat.y();
  msg.pose.orientation.z = quat.z();

  return msg;
}

inline geometry_msgs::msg::Pose egienToGeoPose(const Eigen::Matrix4d &pose) {
  tf2::Matrix3x3 mat_tf;
  matrixEigenToTF2(pose.block<3, 3>(0, 0), mat_tf);

  double roll, pitch, yaw;
  mat_tf.getRPY(roll, pitch, yaw);

  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);

  geometry_msgs::msg::Pose msg;
  msg.position.x    = pose(0, 3);
  msg.position.y    = pose(1, 3);
  msg.position.z    = pose(2, 3);
  msg.orientation.w = quat.w();
  msg.orientation.x = quat.x();
  msg.orientation.y = quat.y();
  msg.orientation.z = quat.z();

  return msg;
}


//type UTILITY

  void computeRegistration() {
    // auto logger = get_logger();

    // auto source_points = convertCloudToEigenVector(*source_cloud_);
    // auto target_points = convertCloudToEigenVector(*target_cloud_);

    // if (source_points.empty() || target_points.empty()) {
    //   RCLCPP_ERROR(logger, "Error: One of the point clouds is empty after filtering.");
    //   return;
    // }

    // RCLCPP_INFO(logger, "Running KISSMatcher...");

    // kiss_matcher::KISSMatcherConfig config(resolution_);
    // kiss_matcher::KISSMatcher matcher(config);
    // auto solution = matcher.estimate(source_points, target_points);

    // estimated_transform_.block<3, 3>(0, 0)    = solution.rotation.cast<float>();
    // estimated_transform_.topRightCorner(3, 1) = solution.translation.cast<float>();

    // matcher.print();

    // constexpr size_t min_inliers = 5;
    // if (matcher.getNumFinalInliers() < min_inliers) {
    //   RCLCPP_WARN(logger, "=> Registration might have failed.");
    // } else {
    //   RCLCPP_INFO(logger, "=> Registration likely succeeded.");
    // }

    // source_cloud_ = convertEigenVectorToCloud(source_points);
    // target_cloud_ = convertEigenVectorToCloud(target_points);
  }

  void animateTransformation() {
    // rclcpp::Rate loop_rate(frame_rate_);
    // sensor_msgs::msg::PointCloud2 target_msg, transformed_source_msg;

    // Eigen::Vector4d source_centroid;
    // pcl::compute3DCentroid(*source_cloud_, source_centroid);

    // Eigen::Matrix4f translation_to_origin   = Eigen::Matrix4f::Identity();
    // Eigen::Matrix4f translation_back        = Eigen::Matrix4f::Identity();
    // translation_to_origin.block<3, 1>(0, 3) = -source_centroid.head<3>().cast<float>();
    // translation_back.block<3, 1>(0, 3)      = source_centroid.head<3>().cast<float>();

    // Eigen::Matrix4f accumulated_transform = Eigen::Matrix4f::Identity();

    // int num_total_steps = 2 * moving_rate_;

    // for (double step = 0; step <= num_total_steps; ++step) {
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr animated_cloud =
    //       std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    //   Eigen::Matrix4f frame_transform = Eigen::Matrix4f::Identity();

    //   if (step <= num_total_steps / 2) {
    //     double progress = static_cast<double>(step) / (num_total_steps / 2);
    //     Eigen::Quaternionf start_rotation(Eigen::Matrix3f::Identity());
    //     Eigen::Quaternionf goal_rotation(estimated_transform_.block<3, 3>(0, 0));
    //     Eigen::Quaternionf interpolated_rotation = start_rotation.slerp(progress, goal_rotation);

    //     Eigen::Matrix4f rotation_transform   = Eigen::Matrix4f::Identity();
    //     rotation_transform.block<3, 3>(0, 0) = interpolated_rotation.toRotationMatrix();

    //     frame_transform       = translation_back * rotation_transform * translation_to_origin;
    //     accumulated_transform = frame_transform;
    //   } else {
    //     double translation_ratio =
    //         static_cast<double>(step - num_total_steps / 2) / (num_total_steps / 2);
    //     Eigen::Vector3f start_translation = accumulated_transform.block<3, 1>(0, 3);
    //     Eigen::Vector3f goal_translation  = estimated_transform_.block<3, 1>(0, 3);

    //     Eigen::Vector3f interpolated_translation =
    //         start_translation + translation_ratio * (goal_translation - start_translation);

    //     frame_transform                   = accumulated_transform;
    //     frame_transform.block<3, 1>(0, 3) = interpolated_translation;
    //   }

    //   pcl::transformPointCloud(*source_cloud_, *animated_cloud, frame_transform);
    //   pcl::toROSMsg(*animated_cloud, transformed_source_msg);
    //   pcl::toROSMsg(*target_cloud_, target_msg);
    //   transformed_source_msg.header.frame_id = "map";
    //   target_msg.header.frame_id             = "map";
    //   target_publisher_->publish(target_msg);
    //   source_publisher_->publish(transformed_source_msg);

    //   loop_rate.sleep();
    // }
  }


//step2 fine regist with GICP
RegOutput icpAlignment(const pcl::PointCloud<PointType> &src,
                                    const pcl::PointCloud<PointType> &tgt) {
  RegOutput reg_output;
  aligned_->clear();
  // merge subkeyframes before ICP
  pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr tgt_cloud(new pcl::PointCloud<PointType>());
  *src_cloud = src;
  *tgt_cloud = tgt;
  local_reg_handler_->setInputTarget(tgt_cloud);
  local_reg_handler_->setInputSource(src_cloud);

  local_reg_handler_->align(*aligned_);

  const auto &local_reg_result = local_reg_handler_->getRegistrationResult();

  double overlapness =
      static_cast<double>(local_reg_result.num_inliers) / src_cloud->size() * 100.0;
  reg_output.overlapness_ = overlapness;

  // NOTE(hlim): fine_T_coarse
  reg_output.pose_ = local_reg_handler_->getFinalTransformation().cast<double>();
  // if matchness overlapness is over than threshold,
  // that means the registration result is likely to be sufficiently overlapped
  if (overlapness > gicp_config_.overlap_threshold_) {
    reg_output.is_valid_     = true;
    reg_output.is_converged_ = true;

    last_success_icp_time_ = std::chrono::steady_clock::now();
    has_success_icp_time_  = true;
    
  }
  //output log
  // if (config_.verbose_) {
    if (overlapness > gicp_config_.overlap_threshold_) {
      RCLCPP_INFO(this->get_logger(),
                  "Overlapness: \033[1;32m%.2f%% > %.2f%%\033[0m",
                  overlapness,
                  gicp_config_.overlap_threshold_);
      is_regist_success=true;
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Overlapness: %.2f%% < %.2f%%\033[0m",
                  overlapness,
                  gicp_config_.overlap_threshold_);
    }
  // }
  return reg_output;
}

Eigen::Matrix4d last_alignment;
bool is_regist_success = false;
//step1 global rough regist 
//if success, fine regist will go on 
RegOutput coarseToFineAlignment(const pcl::PointCloud<PointType> &src,
                                             const pcl::PointCloud<PointType> &tgt) {
// RegOutput coarseToFineAlignment(const pcl::PointCloud<PointType> &src) {
  RegOutput reg_output;
  coarse_aligned_->clear();

  const auto &src_vec = convertCloudToVec(src);
  const auto &tgt_vec = convertCloudToVec(tgt);//always globalmap TODO: global define
  // global_reg_handler_->getInitialCorrespondences(src_vec, tgt_vec);
  const auto &solution = global_reg_handler_->estimate(src_vec, tgt_vec);

  Eigen::Matrix4d coarse_alignment      = Eigen::Matrix4d::Identity();
  coarse_alignment.block<3, 3>(0, 0)    = solution.rotation.cast<double>();
  coarse_alignment.topRightCorner(3, 1) = solution.translation.cast<double>();
  // 提取当前的yaw角度
    double yaw = atan2(solution.rotation(1, 0), solution.rotation(0, 0));

    // 构造只有yaw旋转的新旋转矩阵
    Eigen::Matrix3d new_rotation;
    new_rotation << cos(yaw), -sin(yaw), 0,
                    sin(yaw),  cos(yaw), 0,
                    0,          0,       1;
    Eigen::Vector3d new_translation;
    new_translation << solution.translation.x(), solution.translation.y(), 0.1;

    // 将新的旋转矩阵赋值给coarse_alignment
    // coarse_alignment.block<3, 3>(0, 0) = new_rotation;
    // coarse_alignment.topRightCorner(3, 1) = new_translation;

  *coarse_aligned_ = transformPcd(src, coarse_alignment);
  // if(is_regist_success==false)
  // {
  //   *coarse_aligned_ = transformPcd(src, coarse_alignment);
  // }
  // else
  // {
  //   *coarse_aligned_ = transformPcd(src, last_alignment);
  // }
  

  const size_t num_inliers = global_reg_handler_->getNumFinalInliers();
  const size_t rot_num_inliers = global_reg_handler_->getNumRotationInliers();

  //score log
  // if (config_.verbose_) {
    if (num_inliers > num_inliers_threshold_) {
      RCLCPP_INFO(this->get_logger(),
                  "\033[1;32m# final inliers: %lu > %lu\033[0m , \033[1;32m# final rot inliers: %lu \033[0m",
                  num_inliers,
                  num_inliers_threshold_,
                  rot_num_inliers);
    } else {
      RCLCPP_WARN(
          this->get_logger(), "# final inliers: %lu < %lu , final rot inliers: %lu ",
            num_inliers, num_inliers_threshold_,rot_num_inliers);
    }
  // }

  // NOTE(hlim): A small number of inliers suggests that the initial alignment may have failed,
  // so fine alignment is meaningless.
  if (!solution.valid || num_inliers < num_inliers_threshold_) {
    return reg_output;
  } else {
    if(is_regist_success==false)
    {
      last_alignment = coarse_alignment;
    }
    const auto &fine_output = icpAlignment(*coarse_aligned_, tgt);
    reg_output              = fine_output;
    reg_output.pose_        = fine_output.pose_ * coarse_alignment;

    // Use this cloud to debug whether the transformation is correct.
    // *debug_cloud_        = transformPcd(src, reg_output.pose_);
  }
  return reg_output;
}

bool message_published_;
std::mutex pointcloud_publish_mutex_;
void publish_pointcloud()
{
    if(!local_map_||!tgt_cloud_||local_map_->size()<1)
    {
      return;
    }
  std::lock_guard<std::mutex> lock(pointcloud_publish_mutex_);
  // 检查是否有订阅者
  if (source_publisher_->get_subscription_count() > 0)
  {
    source_publisher_->publish(toROSMsg(*local_map_,base_frame_));
  }
  if (target_publisher_->get_subscription_count() > 0 && !message_published_)
  {
    // 如果有订阅者并且消息尚未发布，则发布消息
    message_published_ = true;//flag to control publish
    target_publisher_->publish(toROSMsg(*tgt_cloud_,map_frame_));
    RCLCPP_INFO(this->get_logger(), "[publish_pointcloud] : Publishing GlobalMap Pointcloud");
    
    return;
  }
  else
  {
    message_published_ = false;//flag to control publish
  }
  

  
}

//PublishTF
std::mutex realtime_pose_mutex_;
std::string map_frame_="map";
std::string base_frame_="odom";

private:
std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
void pubMapOdomTF() 
  // NOTE(hlim): Instead of visualizing only when adding keyframes (node-wise), which can feel
  // choppy, we visualize the current frame every cycle to ensure smoother, real-time visualization.
  {
    if(!local_map_init_ok)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
    updateLocalMap();
    if(!local_map_||!tgt_cloud_||local_map_->size()<1)
    {
      RCLCPP_INFO(this->get_logger(), "Unvalid local map.");
      return;
    }
    
    // odom_delta_                    = odom_delta_ * lastest_odom.inverse() * current_frame_.pose_;
    // current_frame_.pose_corrected_ = last_corrected_pose_ * odom_delta_;
    // target_publisher_->publish(toROSMsg(*tgt_cloud_,map_frame_));//move to function:publish_pointcloud
    // source_publisher_->publish(toROSMsg(*local_map_,base_frame_));//move to function:publish_pointcloud
    auto reg_output_ = coarseToFineAlignment(*local_map_,*tgt_cloud_);
    if(!reg_output_.is_valid_)
    {
      global_regist_timer_ = this->create_wall_timer(std::chrono::milliseconds(300),
                                        std::bind(&KissBackend::pubMapOdomTF, this));
      num_inliers_threshold_=get_parameter("kiss_config.overlap_threshold_high").as_double();
      return;
    }
    global_regist_timer_ = this->create_wall_timer(std::chrono::milliseconds(100000),
                                        std::bind(&KissBackend::pubMapOdomTF, this));
    num_inliers_threshold_ = get_parameter("kiss_config.overlap_threshold_low").as_double();
    

    geometry_msgs::msg::PoseStamped ps =
        eigenToPoseStamped(reg_output_.pose_, map_frame_);
    // realtime_pose_pub_->publish(ps);

    geometry_msgs::msg::TransformStamped transform_stamped;
    // transform_stamped.header.stamp    = timestamp;
    transform_stamped.header.stamp    = this->now();
    transform_stamped.header.frame_id = map_frame_;
    transform_stamped.child_frame_id  = base_frame_;
    Eigen::Quaterniond q(reg_output_.pose_.block<3, 3>(0, 0));
    transform_stamped.transform.translation.x = reg_output_.pose_(0, 3);
    transform_stamped.transform.translation.y = reg_output_.pose_(1, 3);
    transform_stamped.transform.translation.z = reg_output_.pose_(2, 3);
    transform_stamped.transform.rotation.x    = q.x();
    transform_stamped.transform.rotation.y    = q.y();
    transform_stamped.transform.rotation.z    = q.z();
    transform_stamped.transform.rotation.w    = q.w();
    tf_broadcaster_->sendTransform(transform_stamped);
  }



};


// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<KissBackend>());
//   rclcpp::shutdown();
//   return 0;
// }
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  
  // 创建节点
  auto node = std::make_shared<KissBackend>();
  
  // 创建多线程执行器
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(),  // 默认选项
    3  // 线程数
  );
  
  // 添加节点到执行器
  executor.add_node(node);
  
  // 运行执行器
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
