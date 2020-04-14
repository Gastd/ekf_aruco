#include "ekf_aruco/ekf.h"


//-------------- Utility functions -------------------------

EKF::EKF()
{
    initFilter();
    initRos();
}

EKF::~EKF() {}

void EKF::initRos()
{
    /*Initalize Ros*/
    control_topic_ = "pose";
    measur_topic_ = "/fiducial_transforms";
    debug_topic_ = "/debug_filter";
    ros::NodeHandle node_handle_;
    control_sub_  = node_handle_.subscribe(control_topic_, 1, &EKF::controlCallback, this);
    measur_sub_ = node_handle_.subscribe(measur_topic_, 1, &EKF::measurementCallback, this);
    odom_filter_pub_ = node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("indoor_ekf/pose", 0);
    vis_pub_ = node_handle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    debug_pub_ = node_handle_.advertise<std_msgs::Float32>( debug_topic_, 0);
}

void EKF::controlCallback(const nav_msgs::OdometryConstPtr& odom_msg)
{
    static ros::Time old_time;
    ros::Duration dt = odom_msg->header.stamp - old_time;
    old_time = odom_msg->header.stamp;

    if(dt.toSec() > 1)
        return;

    double v = odom_msg->twist.twist.linear.x;
    double w = odom_msg->twist.twist.angular.z;

    // controls_.push_back();
    dt_ = dt.toSec();
    // ROS_INFO_STREAM("v = " << v);
    // ROS_INFO_STREAM("w = " << w);
    // ROS_INFO_STREAM("dt = " << dt_);
    // predict(Eigen::Vector2d(v,w));
}

// void EKF::measurementCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
// {
//     // measurements_.push_back(pose_msg->pose);
//     correct(pose_msg->pose);
// }

void EKF::measurementCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& fid_trans)
{
    Eigen::VectorXd measurement = Eigen::VectorXd::Zero(7);

    if(fid_trans->transforms.size() > 0)
    {

        for(auto marker_it = fid_trans->transforms.begin(); marker_it != fid_trans->transforms.end(); ++marker_it)
        {
            measurement(0) = marker_it->transform.rotation.x;
            measurement(1) = marker_it->transform.rotation.y;
            measurement(2) = marker_it->transform.rotation.z;
            measurement(3) = marker_it->transform.rotation.w;

            measurement(4) = marker_it->transform.translation.x;
            measurement(5) = marker_it->transform.translation.y;
            measurement(6) = marker_it->transform.translation.z;

            ROS_INFO_STREAM("id: " << marker_it->fiducial_id << " image_error: " << marker_it->object_error);

            correct(marker_it->fiducial_id, measurement, marker_it->object_error);
        }
    }
}

void EKF::initFilter()
{
    /*Initialize state*/
    //       q0,q1,q2,q3,px,py,pz
    n_states_ = 7;
    state_ = Eigen::VectorXd::Zero(n_states_);
    state_(3) = 1.0; // unit quaternion
    cov_state_ = Eigen::MatrixXd::Identity(n_states_, n_states_);
    cov_state_ *= 1e-1;

    /*Initialize variances*/
    motion_noise_ = Eigen::MatrixXd::Identity(2,2);
    motion_noise_ *= 1e-3;

    // Kalman filter Rt, 
    double std_x = 1e-1;
    double std_y = 1e-1;
    double std_yaw = 1e-1;
    
    process_noise_covar_ = Eigen::MatrixXd::Identity(7, 7);
    // process_noise_covar_(0,0) = 1e-2;
    // process_noise_covar_(1,1) = 1e-2;
    // process_noise_covar_(2,2) = 1e-2;
    // process_noise_covar_(3,3) = 1e-2;

    // process_noise_covar_(4,4) = 1e-3;
    // process_noise_covar_(5,5) = 1e-3;
    // process_noise_covar_(6,6) = 1e-3;

    /*Initialize Jacobians*/
    motion_jacob_ = Eigen::MatrixXd::Identity(7, 7);
    motion_noise_jacob_ = Eigen::MatrixXd::Identity(7, 7);
    measurement_jacob_ = Eigen::MatrixXd::Identity(7, 7);
}

void EKF::calculateJacobians(Eigen::Vector2d control)
{
    // motion_jacob_ = Eigen::MatrixXd::Identity(6,6);
}

void EKF::predict(Eigen::Vector2d control)
{
    calculateJacobians(control);

    Eigen::Vector3d mean, delta_state;

    // p = g(ut,xt-1)
    // state_ += delta_state;

    // Covpt = Gt*Covt-1*Gt.t + Qt
    motion_jacob_ = Eigen::MatrixXd::Identity(n_states_, n_states_);
    Eigen::MatrixXd prediction_noise = Eigen::MatrixXd::Zero(n_states_, n_states_);
    for (int i = 0; i < n_states_; i += 7)
    {
        prediction_noise(i + 0, i + 0) = 1e-1;
        prediction_noise(i + 1, i + 1) = 1e-1;
        prediction_noise(i + 2, i + 2) = 1e-1;
        prediction_noise(i + 3, i + 3) = 1e-1;

        prediction_noise(i + 4, i + 4) = 1e-2;
        prediction_noise(i + 5, i + 5) = 1e-2;
        prediction_noise(i + 6, i + 6) = 1e-2;
        // ROS_INFO_STREAM("\n" << prediction_noise /*<< "\n" << cov_state_*/);
    }
    cov_state_ = motion_jacob_ * cov_state_ * motion_jacob_.transpose() + prediction_noise;
    // ROS_INFO_STREAM("\n" << prediction_noise /*<< "\n" << cov_state_*/);
}

void EKF::correct(uint32_t measurement_id, Eigen::VectorXd measurement, double object_error)
{
    uint8_t is_new;
    int offset = id_measurement(static_cast<uint8_t>(measurement_id), is_new);
    // ROS_INFO_STREAM("offset: " << offset);
    if(is_new == 1)
    {
        state_.segment<7>(offset) = measurement;
        cov_state_.block<7, 7>(offset, offset) = Eigen::MatrixXd::Identity(7, 7) * object_error;
        return;
    }

    // ROS_INFO_STREAM("state size: " << state_.size() << " cov size: " << cov_state_.size());
    // Eigen::VectorXd st = state_.segment<7>(offset);
    // Eigen::MatrixXd cov = cov_state_.block<7, 7>(offset, offset);

    // Kt = Covpt*Ht.t*(Ht*Covpt*Ht.t + Qt)^-1
    // xt = p + Kt*(zt - h(p))
    // Covt = (I - Kt*Ht)*Covpt
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_states_, n_states_);
    Eigen::MatrixXd measurement_jacob_ = Eigen::MatrixXd::Zero(7, n_states_);
    // ROS_INFO_STREAM("measurement_jacob_ cols: " << measurement_jacob_.cols() << " rows: " << measurement_jacob_.rows());
    measurement_jacob_.block<7, 7>(0, offset) = Eigen::MatrixXd::Identity(7, 7);
    // Eigen::MatrixXd measurement_jacob_ = Eigen::MatrixXd::Identity(7, 7);

    process_noise_covar_(0,0) = object_error / 10.0;
    process_noise_covar_(1,1) = object_error / 10.0;
    process_noise_covar_(2,2) = object_error / 10.0;
    process_noise_covar_(3,3) = object_error / 10.0;

    process_noise_covar_(4,4) = object_error;
    process_noise_covar_(5,5) = object_error;
    process_noise_covar_(6,6) = object_error;

    Eigen::VectorXd state_measurement = measurement_jacob_ * state_;
    Eigen::MatrixXd innovation_cov = measurement_jacob_ * cov_state_ * measurement_jacob_.transpose() + process_noise_covar_;
    Eigen::MatrixXd kalman_gain = cov_state_ * measurement_jacob_.transpose() * innovation_cov.inverse();

    /*Mahalanobis*/
    Eigen::MatrixXd innovation = measurement - state_measurement;

    Eigen::MatrixXd cov_mahala = innovation_cov;
    Eigen::MatrixXd mahala = (innovation.transpose() * (cov_mahala).inverse() * innovation);

    ROS_INFO_STREAM("mahala " << mahala);
    if(mahala(0) > 20) //  14.067
    {
        ROS_INFO_STREAM("innovation: \n" << innovation);
        ROS_INFO_STREAM("cov_mahala: \n" << cov_mahala);
        ROS_INFO_STREAM("innovation.transpose(): \n" << innovation.transpose());
        return;
    }

    // ROS_INFO_STREAM("kalman_gain cols: " << kalman_gain.cols() << " rows: " << kalman_gain.rows());
    // ROS_INFO_STREAM("kalman_gain: \n" << kalman_gain);
    // ROS_INFO_STREAM("innovation cols: " << innovation.cols() << " rows: " << innovation.rows());
    // ROS_INFO_STREAM("state_ cols: " << state_.cols() << " rows: " << state_.rows());
    state_ += kalman_gain * (innovation);
    cov_state_ = (I - kalman_gain * measurement_jacob_) * cov_state_;

    // state_.segment<7>(offset) = st;
    // cov_state_.block<7, 7>(offset, offset) = cov;

    // normalize quaternion and its covariance
    magPseudoMeasurementNormalization(offset);
    // ROS_INFO_STREAM("state \n" << state_);
    // ROS_INFO_STREAM("covariance \n" << cov_state_);
    std_msgs::Float32 mahala_filter_msg;
    mahala_filter_msg.data = mahala(0);
    debug_pub_.publish(mahala_filter_msg);

    return;
}

int EKF::id_measurement(uint8_t id, uint8_t& is_new)
{
    int offset;

    is_new = 0;
    if (measurements_ids_.size() == 0)
    {
        offset = 0;
        measurements_ids_[id] = offset;
        return offset;
    }

    std::map<uint8_t, uint8_t>::iterator it = measurements_ids_.find(id);
    if (it == measurements_ids_.end())
    {
        offset = n_states_;
        is_new = 1;

        Eigen::VectorXd new_state = Eigen::VectorXd::Zero(n_states_ + 7);
        Eigen::MatrixXd new_cov = Eigen::MatrixXd::Zero(n_states_ + 7, n_states_ + 7);
        for (int i = 0; i < n_states_; ++i)
        {
            new_state(i) = state_(i);
            for (int j = 0; j < n_states_; ++j)
            {
                new_cov(i, j) = cov_state_(i, j);
            }
        }
        state_ = new_state;
        cov_state_ = new_cov;
        n_states_ += 7;
        measurements_ids_[id] = offset;
    }
    else
        offset = it->second;

    return offset;
}

// int search_id(uint32_t id)
// {
//     measurements_ids_
// }

// Quaternion normalization through magnitude pseudo-measurement normalization 
void EKF::magPseudoMeasurementNormalization(int offset)
{
    Eigen::VectorXd st = state_.segment<7>(offset);
    Eigen::MatrixXd cov = cov_state_.block<7, 7>(offset, offset);

    Eigen::MatrixXd quat_jacob_ = Eigen::RowVectorXd::Zero(7);
    // quat_jacob_ = [ 2q_0 2q_1 2q_2 2q_3 0 0 0 ]
    quat_jacob_(0) = 2 * st(0);
    quat_jacob_(1) = 2 * st(1);
    quat_jacob_(2) = 2 * st(2);
    quat_jacob_(3) = 2 * st(3);

    // Kalman update
    Eigen::MatrixXd quat_noise_covar = Eigen::MatrixXd::Identity(1,1) * 1e-11;

    Eigen::MatrixXd p_quat = quat_jacob_ * cov * quat_jacob_.transpose() + quat_noise_covar;
    Eigen::MatrixXd p_xquat = cov * quat_jacob_.transpose();
    Eigen::MatrixXd kalman_gain_quat = p_xquat * p_quat.inverse();

    // quat_sum_sqr_minus_one = SUM from 0 to 3 (1 - q_i^2)
    double quat_sum_sqr_minus_one = 1;
    for(int i = 0; i < 4; i++)
        quat_sum_sqr_minus_one -= st(i) * st(i);

    Eigen::MatrixXd I_Kqu_jacob = Eigen::MatrixXd::Identity(7, 7) - kalman_gain_quat * quat_jacob_;
    st = st + kalman_gain_quat * quat_sum_sqr_minus_one;
    cov = I_Kqu_jacob * cov * I_Kqu_jacob.transpose() + kalman_gain_quat * quat_noise_covar * kalman_gain_quat.transpose();

    state_.segment<7>(offset) = st;
    cov_state_.block<7, 7>(offset, offset) = cov;
    
    return;
}

void EKF::run()
{
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        predict(Eigen::Vector2d::Zero(2));
        ros::spinOnce();
        loop_rate.sleep();
        publishData();
    }
}

void EKF::publishData()
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "camera";
    marker.header.stamp = ros::Time();
    for (int i = 0; i < n_states_; i += 7)
    {
        marker.id = i/7;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = state_(i + 4);
        marker.pose.position.y = state_(i + 5);
        marker.pose.position.z = state_(i + 6);
        marker.pose.orientation.x = state_(i + 0);
        marker.pose.orientation.y = state_(i + 1);
        marker.pose.orientation.z = state_(i + 2);
        marker.pose.orientation.w = state_(i + 3);
        marker.scale.x = 0.042;
        marker.scale.y = 0.042;
        marker.scale.z = 0.001;

        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        vis_pub_.publish(marker);
    }

    // geometry_msgs::PoseWithCovarianceStamped odom;
    // odom.header.stamp = ros::Time::now();
    // odom.header.frame_id = "map";
    // // odom.child_frame_id = "base_link";
    // geometry_msgs::Quaternion rotation;
    
    // odom.pose.pose.position.x = state_(3);
    // odom.pose.pose.position.y = state_(4);
    // odom.pose.pose.position.z = state_(5);
    // odom.pose.pose.orientation = geometry_msgs::Quaternion(rotation);

    // odom.pose.covariance[0]  = cov_state_(0); // x x
    // odom.pose.covariance[1]  = cov_state_(1); // x y
    // odom.pose.covariance[5]  = cov_state_(2); // x yaw

    // odom.pose.covariance[6]  = cov_state_(3); // x y
    // odom.pose.covariance[7]  = cov_state_(4); // y y
    // odom.pose.covariance[11] = cov_state_(5); // y yaw

    // odom.pose.covariance[30] = cov_state_(6); // x yaw
    // odom.pose.covariance[31] = cov_state_(7); // y yaw
    // odom.pose.covariance[35] = cov_state_(8); // yaw yaw


    // The localization node should send map_frame to odom_frame transform,
    // the odom_frame to base_frame is send by p2os_driver, this node estimate
    // the transform between map_frame and base_frame, which has been find above

    tf::Transform base_to_map;
    // tf::StampedTransform odom_to_base;
    tf::Quaternion rotation;

    // try
    // {
    //     listener_.lookupTransform("odom", "base_link", ros::Time(0), odom_to_base);
    // }
    // catch (tf::TransformException ex)
    // {
    //     ROS_ERROR("%s",ex.what());
    //     return;
    // }
    tf::quaternionMsgToTF(marker.pose.orientation, rotation);
    base_to_map.setOrigin( tf::Vector3(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z) );
    base_to_map.setRotation(rotation);

    // tf::Transform odom_to_map = base_to_map * odom_to_base.inverse();
    ros::Time ts = ros::Time::now();
    ros::Duration transform_tolerance(0.1);
    br_.sendTransform(tf::StampedTransform(base_to_map, ts+transform_tolerance, "camera", "id0"));

    // odom_filter_pub_.publish(odom);
}


Eigen::VectorXd EKF::getState()
{
    return state_;
}

Eigen::MatrixXd EKF::getCovariance()
{
    return cov_state_;
}
