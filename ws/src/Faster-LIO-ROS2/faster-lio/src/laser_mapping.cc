
// #include <tf/transform_broadcaster.h>
//ROS2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>
#include <yaml-cpp/yaml.h>
#include <execution>
#include <fstream>

#include "laser_mapping.h"
#include "utils.h"


const std::string PARAM_PATH_SAVE_EN = "path_save_en";
const std::string PARAM_PUBLISH_PATH_PUBLISH_EN ="publish.path_publish_en";
const std::string PARAM_PUBLISH_SCAN_PUBLISH_EN ="publish.scan_publish_en";
const std::string PARAM_PUBLISH_DENSE_PUBLISH_EN ="publish.dense_publish_en";
const std::string PARAM_PUBLISH_SCAN_BODYFRAME_PUB_EN ="publish.scan_bodyframe_pub_en";
const std::string PARAM_PUBLISH_SCAN_EFFECT_PUB_EN ="publish.scan_effect_pub_en";
const std::string PARAM_COMMON_TIME_SYNC_EN ="common.time_sync_en";
const std::string PARAM_FITER_SIZE_SURF ="filter_size_surf";
const std::string PARAM_FILTER_SIZE_MAP ="filter_size_map";
const std::string PARAM_CUBE_SIDE_LENGTH ="cube_side_length";
const std::string PARAM_MAPPING_DET_RNAGE ="mapping.det_range";
const std::string PARAM_MAPPING_GYR_COV ="mapping.gyr_cov";
const std::string PARAM_MAPPING_ACC_COV ="mapping.acc_cov";
const std::string PARAM_MAPPING_B_GYR_COV ="mapping.b_gyr_cov";
const std::string PARAM_MAPPING_B_ACC_COV ="mapping.b_acc_cov";
const std::string PARAM_MAX_ITERATION ="max_iteration";
const std::string PARAM_ESTI_PLANE_THRESHOLD ="esti_plane_threshold";
const std::string PARAM_PREPROCESS_BLIND ="preprocess.blind";
const std::string PARAM_PREPROCESS_TIME_SCLAE ="preprocess.time_scale"; 
const std::string PARAM_PREPROCESS_LIDAR_TYPE ="preprocess.lidar_type";
const std::string PARAM_PREPROCESS_VERTICAL_RESOLUTION ="preprocess.vertical_resolution";
const std::string PARAM_PREPROCESS_HORIZONTAL_RESOLUTION ="preprocess.horizontal_resolution";
// const std::string PARAM_PREPROCESS_SCAN_LINE ="preprocess.scan_line";
const std::string PARAM_POINT_FILTER_NUM ="point_filter_num";
const std::string PARAM_FEATRUE_EXTRACT_ENABLE ="feature_extract_enable";  
const std::string PARAM_RUNTIME_POS_LOG_ENABLE ="runtime_pos_log_enable";
const std::string PARAM_MAPPING_EXTRINSIC_EST_EN ="mapping.extrinsic_est_en";
const std::string PARAM_PCD_SAVE_PCD_SAVE_EN ="pcd_save.pcd_save_en";
const std::string PARAM_PCA_SAVE_INTERVAL ="pcd_save.interval";
const std::string PARAM_MAPPING_EXTRINSIC_T ="mapping.extrinsic_T";
const std::string PARAM_MAPPING_EXTRINSIC_R ="mapping.extrinsic_R";
const std::string PARAM_IVOX_GRID_RESOLUTION ="ivox_grid_resolution";
const std::string PARAM_IVOX_NEARBY_TYPE ="ivox_nearby_type";

namespace faster_lio {

// bool LaserMapping::InitROS(ros::NodeHandle &nh) 
void LaserMapping::InitROS() {
    
    
    // LoadParams(nh);
    // SubAndPubToROS(nh);

    LoadParams();
    SubAndPubToROS();


    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    std::vector<double> epsi(23, 0.001);
    kf_.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS, epsi.data());

    // return true;
}

bool LaserMapping::InitWithoutROS(const std::string &config_yaml) {
    LOG(INFO) << "init laser mapping from " << config_yaml;
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    std::vector<double> epsi(23, 0.001);
    kf_.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS, epsi.data());

    if (std::is_same<IVoxType, IVox<3, IVoxNodeType::PHC, pcl::PointXYZI>>::value == true) {
        LOG(INFO) << "using phc ivox";
    } else if (std::is_same<IVoxType, IVox<3, IVoxNodeType::DEFAULT, pcl::PointXYZI>>::value == true) {
        LOG(INFO) << "using default ivox";
    }

    return true;
}

// bool LaserMapping::LoadParams(ros::NodeHandle &nh) {
void LaserMapping::LoadParams() {
    // get params from param server
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;

    // nh.param<bool>("path_save_en", path_save_en_, true);
    // nh.param<bool>("publish/path_publish_en", path_pub_en_, true);
    // nh.param<bool>("publish/scan_publish_en", scan_pub_en_, true);
    // nh.param<bool>("publish/dense_publish_en", dense_pub_en_, false);
    // nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en_, true);
    // nh.param<bool>("publish/scan_effect_pub_en", scan_effect_pub_en_, false);
    // nh.param<std::string>("publish/tf_imu_frame", tf_imu_frame_, "body");
    // nh.param<std::string>("publish/tf_world_frame", tf_world_frame_, "camera_init");

    // nh.param<int>("max_iteration", options::NUM_MAX_ITERATIONS, 4);
    // nh.param<float>("esti_plane_threshold", options::ESTI_PLANE_THRESHOLD, 0.1);
    // nh.param<std::string>("map_file_path", map_file_path_, "");
    // nh.param<bool>("common/time_sync_en", time_sync_en_, false);
    // nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
    // nh.param<double>("filter_size_map", filter_size_map_min_, 0.0);
    // nh.param<double>("cube_side_length", cube_len_, 200);
    // nh.param<float>("mapping/det_range", det_range_, 300.f);
    // nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    // nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    // nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    // nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    // nh.param<double>("preprocess/blind", preprocess_->Blind(), 0.01);
    // nh.param<float>("preprocess/time_scale", preprocess_->TimeScale(), 1e-3);
    // nh.param<int>("preprocess/lidar_type", lidar_type, 1);
    // nh.param<int>("preprocess/scan_line", preprocess_->NumScans(), 16);
    // nh.param<int>("point_filter_num", preprocess_->PointFilterNum(), 2);
    // nh.param<bool>("feature_extract_enable", preprocess_->FeatureEnabled(), false);
    // nh.param<bool>("runtime_pos_log_enable", runtime_pos_log_, true);
    // nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en_, true);
    // nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en_, false);
    // nh.param<int>("pcd_save/interval", pcd_save_interval_, -1);
    // nh.param<std::vector<double>>("mapping/extrinsic_T", extrinT_, std::vector<double>());
    // nh.param<std::vector<double>>("mapping/extrinsic_R", extrinR_, std::vector<double>());

    // nh.param<float>("ivox_grid_resolution", ivox_options_.resolution_, 0.2);
    // nh.param<int>("ivox_nearby_type", ivox_nearby_type, 18);


    // ROS2 

    this->declare_parameter(PARAM_PATH_SAVE_EN,  rclcpp::PARAMETER_BOOL);
    this->declare_parameter(PARAM_PUBLISH_PATH_PUBLISH_EN,  rclcpp::PARAMETER_BOOL);
    this->declare_parameter(PARAM_PUBLISH_SCAN_PUBLISH_EN,  rclcpp::PARAMETER_BOOL);
    this->declare_parameter(PARAM_PUBLISH_DENSE_PUBLISH_EN,  rclcpp::PARAMETER_BOOL);
    this->declare_parameter(PARAM_PUBLISH_SCAN_BODYFRAME_PUB_EN,  rclcpp::PARAMETER_BOOL);
    this->declare_parameter(PARAM_PUBLISH_SCAN_EFFECT_PUB_EN,  rclcpp::PARAMETER_BOOL);

    this->declare_parameter("map_file_path", "");
    this->declare_parameter(PARAM_COMMON_TIME_SYNC_EN,  rclcpp::PARAMETER_BOOL);
    this->declare_parameter(PARAM_FITER_SIZE_SURF,rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(PARAM_FILTER_SIZE_MAP, rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(PARAM_CUBE_SIDE_LENGTH,rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(PARAM_MAPPING_DET_RNAGE, rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(PARAM_MAPPING_GYR_COV, rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(PARAM_MAPPING_ACC_COV,rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(PARAM_MAPPING_B_GYR_COV,rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(PARAM_MAPPING_B_ACC_COV ,rclcpp::PARAMETER_DOUBLE);

    this->declare_parameter(PARAM_MAX_ITERATION, rclcpp::PARAMETER_INTEGER);
    this->declare_parameter(PARAM_ESTI_PLANE_THRESHOLD, rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(PARAM_PREPROCESS_BLIND, rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(PARAM_PREPROCESS_TIME_SCLAE, rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(PARAM_PREPROCESS_LIDAR_TYPE, rclcpp::PARAMETER_INTEGER);

    this->declare_parameter(PARAM_PREPROCESS_VERTICAL_RESOLUTION,  rclcpp::PARAMETER_INTEGER);
    this->declare_parameter(PARAM_PREPROCESS_HORIZONTAL_RESOLUTION,  rclcpp::PARAMETER_INTEGER);
    this->declare_parameter(PARAM_POINT_FILTER_NUM, rclcpp::PARAMETER_INTEGER);
    this->declare_parameter(PARAM_FEATRUE_EXTRACT_ENABLE,  rclcpp::PARAMETER_BOOL);

    this->declare_parameter(PARAM_RUNTIME_POS_LOG_ENABLE, true);
    this->declare_parameter(PARAM_MAPPING_EXTRINSIC_EST_EN,  rclcpp::PARAMETER_BOOL);
    this->declare_parameter(PARAM_PCD_SAVE_PCD_SAVE_EN,  rclcpp::PARAMETER_BOOL);
    this->declare_parameter(PARAM_PCA_SAVE_INTERVAL,  rclcpp::PARAMETER_INTEGER);
    this->declare_parameter(PARAM_MAPPING_EXTRINSIC_T, rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter(PARAM_MAPPING_EXTRINSIC_R, rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter(PARAM_IVOX_GRID_RESOLUTION, rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(PARAM_IVOX_NEARBY_TYPE, rclcpp::PARAMETER_INTEGER);

    if (!this->get_parameter(PARAM_PATH_SAVE_EN, path_save_en_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter path_save_en not found");
    }

    if (!this->get_parameter(PARAM_PUBLISH_PATH_PUBLISH_EN, path_pub_en_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter path_pub_en_ not found");
    }
    
    if (!this->get_parameter(PARAM_PUBLISH_SCAN_PUBLISH_EN, scan_pub_en_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter scan_pub_en_ not found");
    }
    
    if (!this->get_parameter(PARAM_PUBLISH_DENSE_PUBLISH_EN, dense_pub_en_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter dense_pub_en_ not found");
    }
    
    if (!this->get_parameter(PARAM_PUBLISH_SCAN_BODYFRAME_PUB_EN, scan_body_pub_en_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter scan_body_pub_en_ not found");
    }
    
    if (!this->get_parameter(PARAM_PUBLISH_SCAN_EFFECT_PUB_EN, scan_effect_pub_en_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter scan_effect_pub_en_ not found");
    }
    
    if (!this->get_parameter("map_file_path", map_file_path_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter map_file_path_ not found");
    }
    
    if (!this->get_parameter(PARAM_COMMON_TIME_SYNC_EN, time_sync_en_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter time_sync_en_ not found");
    }

    if (!this->get_parameter(PARAM_FITER_SIZE_SURF, filter_size_surf_min)) {
        RCLCPP_WARN(this->get_logger(), "Parameter filter_size_surf_min not found");
    }
    if (!this->get_parameter(PARAM_FILTER_SIZE_MAP, filter_size_map_min_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter filter_size_map_min_ not found");
    }
    if (!this->get_parameter(PARAM_CUBE_SIDE_LENGTH, cube_len_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter cube_len_ not found");
    }
    if (!this->get_parameter(PARAM_MAPPING_DET_RNAGE, det_range_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter det_range_ not found");
    }
    if (!this->get_parameter(PARAM_MAPPING_GYR_COV, gyr_cov)) {
        RCLCPP_WARN(this->get_logger(), "Parameter gyr_cov not found");
    }
    if (!this->get_parameter(PARAM_MAPPING_ACC_COV, acc_cov)) {
        RCLCPP_WARN(this->get_logger(), "Parameter acc_cov not found");
    }
    if (!this->get_parameter(PARAM_MAPPING_B_GYR_COV, b_gyr_cov)) {
        RCLCPP_WARN(this->get_logger(), "Parameter b_gyr_cov not found");
    }
    if (!this->get_parameter(PARAM_MAPPING_B_ACC_COV, b_acc_cov)) {
        RCLCPP_WARN(this->get_logger(), "Parameter b_acc_cov not found");
    }

    if (!this->get_parameter(PARAM_MAX_ITERATION, options::NUM_MAX_ITERATIONS)) {
        RCLCPP_WARN(this->get_logger(), "Parameter options::NUM_MAX_ITERATIONS not found");
    }
    
    if (!this->get_parameter(PARAM_ESTI_PLANE_THRESHOLD, options::ESTI_PLANE_THRESHOLD)) {
        RCLCPP_WARN(this->get_logger(), "Parameter options::ESTI_PLANE_THRESHOLD not found");
    }
    
    if (!this->get_parameter(PARAM_PREPROCESS_BLIND, preprocess_->Blind())) {
        RCLCPP_WARN(this->get_logger(), "Parameter preprocess_->Blind() not found");
    }
    if (!this->get_parameter(PARAM_PREPROCESS_TIME_SCLAE,  preprocess_->TimeScale())) {
        RCLCPP_WARN(this->get_logger(), "Parameter  preprocess_->TimeScale() not found");
    }
    if (!this->get_parameter(PARAM_PREPROCESS_LIDAR_TYPE, lidar_type)) {
        RCLCPP_WARN(this->get_logger(), "Parameter lidar_type not found");
    }

    if (!this->get_parameter(PARAM_PREPROCESS_VERTICAL_RESOLUTION, preprocess_->NumScans())) {
        RCLCPP_WARN(this->get_logger(), "Parameter preprocess_->NumScans() not found");
    }

    if (!this->get_parameter(PARAM_PREPROCESS_HORIZONTAL_RESOLUTION, preprocess_->NumHorizontalScans())) {
        RCLCPP_WARN(this->get_logger(), "Parameter preprocess_->NumHorizontalScans() not found");
    }

    if (!this->get_parameter(PARAM_POINT_FILTER_NUM, preprocess_->PointFilterNum())) {
        RCLCPP_WARN(this->get_logger(), "Parameter preprocess_->PointFilterNum() not found");
    }

    if (!this->get_parameter(PARAM_FEATRUE_EXTRACT_ENABLE, preprocess_->FeatureEnabled())) {
        RCLCPP_WARN(this->get_logger(), "Parameter preprocess_->FeatureEnabled() not found");
    }

    if (!this->get_parameter(PARAM_RUNTIME_POS_LOG_ENABLE,  runtime_pos_log_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter  runtime_pos_log_ not found");
    }

    if (!this->get_parameter(PARAM_MAPPING_EXTRINSIC_EST_EN, extrinsic_est_en_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter extrinsic_est_en_ not found");
    }

    if (!this->get_parameter(PARAM_PCD_SAVE_PCD_SAVE_EN, pcd_save_en_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter pcd_save_en_ not found");
    }
    if (!this->get_parameter(PARAM_PCA_SAVE_INTERVAL, pcd_save_interval_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter pcd_save_interval_ not found");
    }
    if (!this->get_parameter(PARAM_MAPPING_EXTRINSIC_T, extrinT_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter extrinT_ not found");
    }
    if (!this->get_parameter(PARAM_MAPPING_EXTRINSIC_R, extrinR_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter extrinR_ not found");
    }    
    if (!this->get_parameter(PARAM_IVOX_GRID_RESOLUTION, ivox_options_.resolution_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter ivox_options_.resolution_ not found");
    }
    if (!this->get_parameter(PARAM_IVOX_NEARBY_TYPE, ivox_nearby_type)) {
        RCLCPP_WARN(this->get_logger(), "Parameter ivox_nearby_type not found");
    }


    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        // return false;

        return; 

    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    path_.header.stamp = rclcpp::Clock().now();
    path_.header.frame_id = "camera_init";

    voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    lidar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    // return true;
}

bool LaserMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    // get params from yaml
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        path_pub_en_ = yaml["publish"]["path_publish_en"].as<bool>();
        scan_pub_en_ = yaml["publish"]["scan_publish_en"].as<bool>();
        dense_pub_en_ = yaml["publish"]["dense_publish_en"].as<bool>();
        scan_body_pub_en_ = yaml["publish"]["scan_bodyframe_pub_en"].as<bool>();
        scan_effect_pub_en_ = yaml["publish"]["scan_effect_pub_en"].as<bool>();
        tf_imu_frame_ = yaml["publish"]["tf_imu_frame"].as<std::string>("body");
        tf_world_frame_ = yaml["publish"]["tf_world_frame"].as<std::string>("camera_init");
        path_save_en_ = yaml["path_save_en"].as<bool>();

        options::NUM_MAX_ITERATIONS = yaml["max_iteration"].as<int>();
        options::ESTI_PLANE_THRESHOLD = yaml["esti_plane_threshold"].as<float>();
        time_sync_en_ = yaml["common"]["time_sync_en"].as<bool>();

        filter_size_surf_min = yaml["filter_size_surf"].as<float>();
        filter_size_map_min_ = yaml["filter_size_map"].as<float>();
        cube_len_ = yaml["cube_side_length"].as<int>();
        det_range_ = yaml["mapping"]["det_range"].as<float>();
        gyr_cov = yaml["mapping"]["gyr_cov"].as<float>();
        acc_cov = yaml["mapping"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["mapping"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["mapping"]["b_acc_cov"].as<float>();
        preprocess_->Blind() = yaml["preprocess"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["preprocess"]["time_scale"].as<double>();
        lidar_type = yaml["preprocess"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["preprocess"]["vertical_resolution"].as<int>();
        preprocess_->NumHorizontalScans() = yaml["preprocess"]["horizontal_resolution"].as<int>();
        preprocess_->PointFilterNum() = yaml["point_filter_num"].as<int>();
        preprocess_->FeatureEnabled() = yaml["feature_extract_enable"].as<bool>();
        extrinsic_est_en_ = yaml["mapping"]["extrinsic_est_en"].as<bool>();
        pcd_save_en_ = yaml["pcd_save"]["pcd_save_en"].as<bool>();
        pcd_save_interval_ = yaml["pcd_save"]["interval"].as<int>();
        extrinT_ = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();

        ivox_options_.resolution_ = yaml["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = yaml["ivox_nearby_type"].as<int>();
    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    lidar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    run_in_offline_ = true;
    return true;
}

// void LaserMapping::SubAndPubToROS(ros::NodeHandle &nh) {
void LaserMapping::SubAndPubToROS(){

    // ROS subscribe initialization
    std::string lidar_topic, imu_topic;
    // nh.param<std::string>("common/lid_topic", lidar_topic, "/livox/lidar");
    // nh.param<std::string>("common/imu_topic", imu_topic, "/livox/imu");

    // ROS2
    this->declare_parameter("common.lid_topic", "livox/lidar");
    this->declare_parameter("common.imu_topic", "livox/imu");
    if (!this->get_parameter("common.lid_topic", lidar_topic)) {
        RCLCPP_WARN(this->get_logger(), "Parameter common.lid_topic not found");
    }
    if (!this->get_parameter("common.imu_topic", imu_topic)) {
        RCLCPP_WARN(this->get_logger(), "Parameter common.imu_topic not found");
    }


    // if (preprocess_->GetLidarType() == LidarType::AVIA) {
    //     sub_pcl_ = nh.subscribe<livox_ros_driver::CustomMsg>(
    //         lidar_topic, 200000, [this](const livox_ros_driver::CustomMsg::ConstPtr &msg) { LivoxPCLCallBack(msg); });
    // } else {
    //     sub_pcl_ = nh.subscribe<sensor_msgs::PointCloud2>(
    //         lidar_topic, 200000, [this](const sensor_msgs::PointCloud2::ConstPtr &msg) { StandardPCLCallBack(msg); });
    // }

    // sub_imu_ = nh.subscribe<sensor_msgs::Imu>(imu_topic, 200000,
    //                                           [this](const sensor_msgs::Imu::ConstPtr &msg) { IMUCallBack(msg); });

    // ROS2

    if (preprocess_->GetLidarType() == LidarType::AVIA) {
        sub_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lidar_topic, rclcpp::QoS(200000).best_effort(), 
                                [this](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { LivoxPCLCallBack(msg); });
    } else {
        sub_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic, rclcpp::QoS(200000).best_effort(), 
                                [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { StandardPCLCallBack(msg); });
    }

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::QoS(200000).best_effort(), 
                                [this](const sensor_msgs::msg::Imu::SharedPtr msg) { IMUCallBack(msg); });

    // ROS publisher init
    // path_.header.stamp = ros::Time::now();
    // path_.header.frame_id = "camera_init";
    // pub_laser_cloud_world_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    // pub_laser_cloud_body_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    // pub_laser_cloud_effect_world_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_effect_world", 100000);
    // pub_odom_aft_mapped_ = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    // pub_path_ = nh.advertise<nav_msgs::Path>("/path", 100000);

    // ROS2
    path_.header.stamp = rclcpp::Clock().now();
    path_.header.frame_id = "camera_init";
    pub_laser_cloud_world_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered", 100000);
    pub_laser_cloud_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_body", 100000);
    pub_laser_cloud_effect_world_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_effect_world", 100000);
    pub_odom_aft_mapped_ = this->create_publisher<nav_msgs::msg::Odometry>("Odometry", 100000);
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("path", 100000);
    br  = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

// LaserMapping::LaserMapping() {
LaserMapping::LaserMapping(const std::string &name) : Node(name) {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
    InitROS();
    
}


void LaserMapping::Run() {
    if (!SyncPackages()) {
        return;
    }

    /// IMU process, kf prediction, undistortion
    p_imu_->Process(measures_, kf_, scan_undistort_);
    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return;
    }

    /// the first scan
    if (flg_first_scan_) {
        ivox_->AddPoints(scan_undistort_->points);
        first_lidar_time_ = measures_.lidar_bag_time_;
        flg_first_scan_ = false;
        return;
    }
    flg_EKF_inited_ = (measures_.lidar_bag_time_ - first_lidar_time_) >= options::INIT_TIME;

    /// downsample
    Timer::Evaluate(
        [&, this]() {
            voxel_scan_.setInputCloud(scan_undistort_);
            voxel_scan_.filter(*scan_down_body_);
        },
        "Downsample PointCloud");

    int cur_pts = scan_down_body_->size();
    if (cur_pts < 5) {
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        return;
    }
    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);
    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, true);
    plane_coef_.resize(cur_pts, common::V4F::Zero());

    // ICP and iterated Kalman filter update
    Timer::Evaluate(
        [&, this]() {
            // iterated state estimation
            double solve_H_time = 0;
            // update the observation model, will call nn and point-to-plane residual computation
            kf_.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);
            // save the state
            state_point_ = kf_.get_x();
            euler_cur_ = SO3ToEuler(state_point_.rot);
            pos_lidar_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;
        },
        "IEKF Solve and Update");

    // update local map
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");

    LOG(INFO) << "[ mapping ]: In num: " << scan_undistort_->points.size() << " downsamp " << cur_pts
              << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_num_;

    // publish or save map pcd
    if (run_in_offline_) {
        if (pcd_save_en_) {
            PublishFrameWorld();
        }
        if (path_save_en_) {
            PublishPath(pub_path_);
        }
    } else {
        if (pub_odom_aft_mapped_) {
            PublishOdometry(pub_odom_aft_mapped_);
        }
        if (path_pub_en_ || path_save_en_) {
            PublishPath(pub_path_);
        }
        if (scan_pub_en_ || pcd_save_en_) {
            PublishFrameWorld();
        }
        if (scan_pub_en_ && scan_body_pub_en_) {
            PublishFrameBody(pub_laser_cloud_body_);
        }
        if (scan_pub_en_ && scan_effect_pub_en_) {
            PublishFrameEffectWorld(pub_laser_cloud_effect_world_);
        }
    }

    // Debug variables
    frame_num_++;
}

// void LaserMapping::StandardPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
void LaserMapping::StandardPCLCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    mtx_buffer_.lock();
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            // if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
            //     LOG(ERROR) << "lidar loop back, clear buffer";
            //     lidar_buffer_.clear();
            // }
            //ROS2
            if ((double)msg->header.stamp.sec + (double)1e-9* msg->header.stamp.nanosec < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }
            

            PointCloudType::Ptr ptr(new PointCloudType());
            // Note: point cloud preprocess, this reduces the point cloud size
            pcl::PointCloud<ouster_ros::Point> points;
            pcl::fromROSMsg(*msg, points);
            preprocess_->PCProcess(msg, ptr);
            lidar_buffer_.push_back(ptr);
            // time_buffer_.push_back(msg->header.stamp.toSec());
            // last_timestamp_lidar_ = msg->header.stamp.toSec();
            //ROS2
            time_buffer_.push_back((double)msg->header.stamp.sec + (double)1e-9* msg->header.stamp.nanosec);
            last_timestamp_lidar_ =(double) msg->header.stamp.sec + (double)1e-9* msg->header.stamp.nanosec;
        },
        "Preprocess (Standard)");
    mtx_buffer_.unlock();
}

// void LaserMapping::LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
void LaserMapping::LivoxPCLCallBack(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    mtx_buffer_.lock();
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            // if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
            //     LOG(WARNING) << "lidar loop back, clear buffer";
            //     lidar_buffer_.clear();
            // }
            if ((double)msg->header.stamp.sec + (double)1e-9* msg->header.stamp.nanosec < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            // last_timestamp_lidar_ = msg->header.stamp.toSec();
            //ROS2
            last_timestamp_lidar_ =(double)msg->header.stamp.sec + (double)1e-9* msg->header.stamp.nanosec;
            if (!time_sync_en_ && abs(last_timestamp_imu_ - last_timestamp_lidar_) > 10.0 && !imu_buffer_.empty() &&
                !lidar_buffer_.empty()) {
                LOG(INFO) << "IMU and LiDAR not Synced, IMU time: " << last_timestamp_imu_
                          << ", lidar header time: " << last_timestamp_lidar_;
            }

            if (time_sync_en_ && !timediff_set_flg_ && abs(last_timestamp_lidar_ - last_timestamp_imu_) > 1 &&
                !imu_buffer_.empty()) {
                timediff_set_flg_ = true;
                timediff_lidar_wrt_imu_ = last_timestamp_lidar_ + 0.1 - last_timestamp_imu_;
                LOG(INFO) << "Self sync IMU and LiDAR, time diff is " << timediff_lidar_wrt_imu_;
            }

            PointCloudType::Ptr ptr(new PointCloudType());
            preprocess_->PCProcess(msg, ptr);
            lidar_buffer_.emplace_back(ptr);
            time_buffer_.emplace_back(last_timestamp_lidar_);
        },
        "Preprocess (Livox)");

    mtx_buffer_.unlock();
}

// void LaserMapping::IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in) {
void LaserMapping::IMUCallBack(const sensor_msgs::msg::Imu::SharedPtr msg_in) {
    publish_count_++;
    // sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    //ROS2
    sensor_msgs::msg::Imu::SharedPtr msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);
    if (abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en_) {
        
        // msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu_ + msg_in->header.stamp.toSec());

        rclcpp::Time t(static_cast<uint64_t>((timediff_lidar_wrt_imu_ + ((double)msg_in->header.stamp.sec + (double)1e-9* msg_in->header.stamp.nanosec))*1e9 ));
        msg->header.stamp = t;
    
    }

    // double timestamp = msg->header.stamp.toSec();


    double timestamp =  (double)msg->header.stamp.sec + (double)1e-9* msg->header.stamp.nanosec;

    mtx_buffer_.lock();
    if (timestamp < last_timestamp_imu_) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }

    last_timestamp_imu_ = timestamp;
    imu_buffer_.emplace_back(msg);
    mtx_buffer_.unlock();
}

bool LaserMapping::SyncPackages() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.lidar_ = lidar_buffer_.front();
        measures_.lidar_bag_time_ = time_buffer_.front();

        if (measures_.lidar_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else if (measures_.lidar_->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_bag_time_ + measures_.lidar_->points.back().curvature / double(1000);
            lidar_mean_scantime_ +=
                (measures_.lidar_->points.back().curvature / double(1000) - lidar_mean_scantime_) / scan_num_;
        }

        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    // double imu_time = imu_buffer_.front()->header.stamp.toSec();
    //ROS2
    double imu_time = (double) imu_buffer_.front()->header.stamp.sec + (double)1e-9*imu_buffer_.front()->header.stamp.nanosec ;
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        // imu_time = imu_buffer_.front()->header.stamp.toSec();
        imu_time = (double)imu_buffer_.front()->header.stamp.sec +(double) imu_buffer_.front()->header.stamp.nanosec *1e-9;
        if (imu_time > lidar_end_time_) break;
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
}

void LaserMapping::PrintState(const state_ikfom &s) {
    LOG(INFO) << "state r: " << s.rot.coeffs().transpose() << ", t: " << s.pos.transpose()
              << ", off r: " << s.offset_R_L_I.coeffs().transpose() << ", t: " << s.offset_T_L_I.transpose();
}

void LaserMapping::MapIncremental() {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    int cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    std::for_each(std::execution::unseq, index.begin(), index.end(), [&](const size_t &i) {
        /* transform to world frame */
        PointBodyToWorld(&(scan_down_body_->points[i]), &(scan_down_world_->points[i]));

        /* decide if need add to map */
        PointType &point_world = scan_down_world_->points[i];
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector &points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            bool need_add = true;
            float dist = common::calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= options::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < options::NUM_MATCH_POINTS; readd_i++) {
                    if (common::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    Timer::Evaluate(
        [&, this]() {
            ivox_->AddPoints(points_to_add);
            ivox_->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");
}

/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void LaserMapping::ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
    int cnt_pts = scan_down_body_->size();

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    Timer::Evaluate(
        [&, this]() {
            auto R_wl = (s.rot * s.offset_R_L_I).cast<float>();
            auto t_wl = (s.rot * s.offset_T_L_I + s.pos).cast<float>();

            /** closest surface search and residual computation **/
            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                PointType &point_body = scan_down_body_->points[i];
                PointType &point_world = scan_down_world_->points[i];

                /* transform to world frame */
                common::V3F p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                auto &points_near = nearest_points_[i];
                if (ekfom_data.converge) {
                    /** Find the closest surfaces in the map **/
                    ivox_->GetClosestPoint(point_world, points_near, options::NUM_MATCH_POINTS);
                    point_selected_surf_[i] = points_near.size() >= options::MIN_NUM_MATCH_POINTS;
                    if (point_selected_surf_[i]) {
                        point_selected_surf_[i] =
                            common::esti_plane(plane_coef_[i], points_near, options::ESTI_PLANE_THRESHOLD);
                    }
                }

                if (point_selected_surf_[i]) {
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef_[i].dot(temp);

                    bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                    if (valid_corr) {
                        point_selected_surf_[i] = true;
                        residuals_[i] = pd2;
                    }
                }
            });
        },
        "    ObsModel (Lidar Match)");

    effect_feat_num_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_num_] = plane_coef_[i];
            corr_pts_[effect_feat_num_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_num_][3] = residuals_[i];

            effect_feat_num_++;
        }
    }
    corr_pts_.resize(effect_feat_num_);
    corr_norm_.resize(effect_feat_num_);

    if (effect_feat_num_ < 1) {
        ekfom_data.valid = false;
        LOG(WARNING) << "No Effective Points!";
        return;
    }

    Timer::Evaluate(
        [&, this]() {
            /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
            ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num_, 12);  // 23
            ekfom_data.h.resize(effect_feat_num_);

            index.resize(effect_feat_num_);
            const common::M3F off_R = s.offset_R_L_I.toRotationMatrix().cast<float>();
            const common::V3F off_t = s.offset_T_L_I.cast<float>();
            const common::M3F Rt = s.rot.toRotationMatrix().transpose().cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                common::V3F point_this_be = corr_pts_[i].head<3>();
                common::M3F point_be_crossmat = SKEW_SYM_MATRIX(point_this_be);
                common::V3F point_this = off_R * point_this_be + off_t;
                common::M3F point_crossmat = SKEW_SYM_MATRIX(point_this);

                /*** get the normal vector of closest surface/corner ***/
                common::V3F norm_vec = corr_norm_[i].head<3>();

                /*** calculate the Measurement Jacobian matrix H ***/
                common::V3F C(Rt * norm_vec);
                common::V3F A(point_crossmat * C);

                if (extrinsic_est_en_) {
                    common::V3F B(point_be_crossmat * off_R.transpose() * C);
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], B[0],
                        B[1], B[2], C[0], C[1], C[2];
                } else {
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0;
                }

                /*** Measurement: distance to the closest surface/corner ***/
                ekfom_data.h(i) = -corr_pts_[i][3];
            });
        },
        "    ObsModel (IEKF Build Jacobian)");
}

/////////////////////////////////////  debug save / show /////////////////////////////////////////////////////

// void LaserMapping::PublishPath(const ros::Publisher pub_path) {
void LaserMapping::PublishPath(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path) {
    SetPosestamp(msg_body_pose_);
    // msg_body_pose_.header.stamp = ros::Time().fromSec(lidar_end_time_);
    rclcpp::Time time_stamp(static_cast<uint64_t>(lidar_end_time_ * 1e9));
    msg_body_pose_.header.stamp = time_stamp;
    msg_body_pose_.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    path_.poses.push_back(msg_body_pose_);
    if (run_in_offline_ == false) {
        pub_path->publish(path_);
    }
}

void LaserMapping::PublishOdometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_aft_mapped) {
    odom_aft_mapped_.header.frame_id = "camera_init";
    odom_aft_mapped_.child_frame_id = "body";
    rclcpp::Time time_stamp(static_cast<uint64_t>(lidar_end_time_ * 1e9));
    odom_aft_mapped_.header.stamp = time_stamp;
    // odom_aft_mapped_.header.stamp = ros::Time().fromSec(lidar_end_time_);  // ros::Time().fromSec(lidar_end_time_);
    SetPosestamp(odom_aft_mapped_.pose);
    pub_odom_aft_mapped->publish(odom_aft_mapped_);
    auto P = kf_.get_P();
    for (int i = 0; i < 6; i++) {
        int k = i < 3 ? i + 3 : i - 3;
        odom_aft_mapped_.pose.covariance[i * 6 + 0] = P(k, 3);
        odom_aft_mapped_.pose.covariance[i * 6 + 1] = P(k, 4);
        odom_aft_mapped_.pose.covariance[i * 6 + 2] = P(k, 5);
        odom_aft_mapped_.pose.covariance[i * 6 + 3] = P(k, 0);
        odom_aft_mapped_.pose.covariance[i * 6 + 4] = P(k, 1);
        odom_aft_mapped_.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // tf::Quaternion q;
    // transform.setOrigin(tf::Vector3(odom_aft_mapped_.pose.pose.position.x, odom_aft_mapped_.pose.pose.position.y,
    //                                 odom_aft_mapped_.pose.pose.position.z));
    // q.setW(odom_aft_mapped_.pose.pose.orientation.w);
    // q.setX(odom_aft_mapped_.pose.pose.orientation.x);
    // q.setY(odom_aft_mapped_.pose.pose.orientation.y);
    // q.setZ(odom_aft_mapped_.pose.pose.orientation.z);
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, odom_aft_mapped_.header.stamp, tf_world_frame_, tf_imu_frame_));

    // ROS2
    geometry_msgs::msg::TransformStamped stampedTransform;
    stampedTransform.header.stamp = odom_aft_mapped_.header.stamp;
    stampedTransform.header.frame_id =  "camera_init";
    stampedTransform.child_frame_id =  "body";
    stampedTransform.transform.translation.x = odom_aft_mapped_.pose.pose.position.x;
    stampedTransform.transform.translation.y = odom_aft_mapped_.pose.pose.position.y;
    stampedTransform.transform.translation.z = odom_aft_mapped_.pose.pose.position.z;
    stampedTransform.transform.rotation.w = odom_aft_mapped_.pose.pose.orientation.w;
    stampedTransform.transform.rotation.x = odom_aft_mapped_.pose.pose.orientation.x;
    stampedTransform.transform.rotation.y = odom_aft_mapped_.pose.pose.orientation.y;
    stampedTransform.transform.rotation.z = odom_aft_mapped_.pose.pose.orientation.z;
    br->sendTransform(stampedTransform);

}

void LaserMapping::PublishFrameWorld() {
    if (!(run_in_offline_ == false && scan_pub_en_) && !pcd_save_en_) {
        return;
    }

    PointCloudType::Ptr laserCloudWorld;
    if (dense_pub_en_) {
        PointCloudType::Ptr laserCloudFullRes(scan_undistort_);
        int size = laserCloudFullRes->points.size();
        laserCloudWorld.reset(new PointCloudType(size, 1));
        for (int i = 0; i < size; i++) {
            PointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }
    } else {
        laserCloudWorld = scan_down_world_;
    }

    if (run_in_offline_ == false && scan_pub_en_) {
        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        rclcpp::Time time_stamp(static_cast<uint64_t>(lidar_end_time_ * 1e9));
        laserCloudmsg.header.stamp = time_stamp;
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
        laserCloudmsg.header.frame_id = "camera_init";
        pub_laser_cloud_world_->publish(laserCloudmsg);
        publish_count_ -= options::PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en_) {
        *pcl_wait_save_ += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save_->size() > 0 && pcd_save_interval_ > 0 && scan_wait_num >= pcd_save_interval_) {
            pcd_index_++;
            std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/scans_") + std::to_string(pcd_index_) +
                                       std::string(".pcd"));
            pcl::PCDWriter pcd_writer;
            LOG(INFO) << "current scan saved to /PCD/" << all_points_dir;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_);
            pcl_wait_save_->clear();
            scan_wait_num = 0;
        }
    }
}

// void LaserMapping::PublishFrameBody(const ros::Publisher &pub_laser_cloud_body) {
void LaserMapping::PublishFrameBody(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_body) {
    int size_scan = scan_undistort_->points.size();
    int size_full = preprocess_->NumHorizontalScans() * preprocess_->NumScans();

    PointCloudType::Ptr laser_cloud_imu_body(new PointCloudType(size_full, 1));

    for (int i = 0; i < size_scan; i++) {
        PointBodyLidarToIMU(&scan_undistort_->points[i], &laser_cloud_imu_body->points[i]);
    }

    // Note: set the point cloud to size that matches the original input, so that the ros_numpy.numpify function in the
    // infer node can parse it properly 
    if(size_scan < size_full){
        for(int i = size_scan; i < size_full; i++){
            laser_cloud_imu_body->points[i].x = 0;
            laser_cloud_imu_body->points[i].y = 0;
            laser_cloud_imu_body->points[i].z = 0;
        }
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laser_cloud_imu_body, laserCloudmsg);
    rclcpp::Time time_stamp(static_cast<uint64_t>(lidar_end_time_ * 1e9));
    laserCloudmsg.header.stamp = time_stamp;
    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
    laserCloudmsg.header.frame_id = "body";
    pub_laser_cloud_body->publish(laserCloudmsg);
    publish_count_ -= options::PUBFRAME_PERIOD;
}

// void LaserMapping::PublishFrameEffectWorld(const ros::Publisher &pub_laser_cloud_effect_world) {
void LaserMapping::PublishFrameEffectWorld(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_effect_world) {
    int size = corr_pts_.size();
    PointCloudType::Ptr laser_cloud(new PointCloudType(size, 1));

    for (int i = 0; i < size; i++) {
        PointBodyToWorld(corr_pts_[i].head<3>(), &laser_cloud->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laser_cloud, laserCloudmsg);
    rclcpp::Time time_stamp(static_cast<uint64_t>(lidar_end_time_ * 1e9));
    laserCloudmsg.header.stamp = time_stamp;
    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
    laserCloudmsg.header.frame_id = "camera_init";
    pub_laser_cloud_effect_world->publish(laserCloudmsg);
    publish_count_ -= options::PUBFRAME_PERIOD;
}

void LaserMapping::Savetrajectory(const std::string &traj_file) {
    std::ofstream ofs;
    ofs.open(traj_file, std::ios::out);
    if (!ofs.is_open()) {
        LOG(ERROR) << "Failed to open traj_file: " << traj_file;
        return;
    }

    ofs << "#timestamp x y z q_x q_y q_z q_w" << std::endl;
    for (const auto &p : path_.poses) {

        ofs << std::fixed << std::setprecision(6) << (double)p.header.stamp.sec+ (double)1e-9* p.header.stamp.nanosec<< " " << std::setprecision(15)
            << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << " " << p.pose.orientation.x
            << " " << p.pose.orientation.y << " " << p.pose.orientation.z << " " << p.pose.orientation.w << std::endl;
    }

    ofs.close();
}

///////////////////////////  private method /////////////////////////////////////////////////////////////////////
template <typename T>
void LaserMapping::SetPosestamp(T &out) {
    out.pose.position.x = state_point_.pos(0);
    out.pose.position.y = state_point_.pos(1);
    out.pose.position.z = state_point_.pos(2);
    out.pose.orientation.x = state_point_.rot.coeffs()[0];
    out.pose.orientation.y = state_point_.rot.coeffs()[1];
    out.pose.orientation.z = state_point_.rot.coeffs()[2];
    out.pose.orientation.w = state_point_.rot.coeffs()[3];
}

void LaserMapping::PointBodyToWorld(const PointType *pi, PointType *const po) {
    common::V3D p_body(pi->x, pi->y, pi->z);
    common::V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
                         state_point_.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LaserMapping::PointBodyToWorld(const common::V3F &pi, PointType *const po) {
    common::V3D p_body(pi.x(), pi.y(), pi.z());
    common::V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
                         state_point_.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = std::abs(po->z);
}

void LaserMapping::PointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
    common::V3D p_body_lidar(pi->x, pi->y, pi->z);
    common::V3D p_body_imu(state_point_.offset_R_L_I * p_body_lidar + state_point_.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void LaserMapping::Finish() {
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save_->size() > 0 && pcd_save_en_) {
        std::string file_name = std::string("scans.pcd");
        std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        LOG(INFO) << "current scan saved to /PCD/" << file_name;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_);
    }

    LOG(INFO) << "finish done";
}
}  // namespace faster_lio