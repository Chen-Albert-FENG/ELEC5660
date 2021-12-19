#include <ekf_filter.h>

namespace ekf_imu_vision {
EKFImuVision::EKFImuVision(/* args */) {}

EKFImuVision::~EKFImuVision() {}

void EKFImuVision::init(ros::NodeHandle& nh) {
  node_ = nh;

  /* ---------- parameter ---------- */
  Qt_.setZero();
  Rt1_.setZero();
  Rt2_.setZero();

  // addition and removel of augmented state
  
  // TODO
  // set M_a_ and M_r_
  M_a_.setZero();
  M_a_.block(0,0,15,15).setIdentity();
  M_a_.block(15,0,6,6).setIdentity();

  M_r_.setZero();
  M_r_.topLeftCorner(15,15).setIdentity();

  for (int i = 0; i < 3; i++) {
    /* process noise */
    node_.param("aug_ekf/ng", Qt_(i, i), -1.0);
    node_.param("aug_ekf/na", Qt_(i + 3, i + 3), -1.0);
    node_.param("aug_ekf/nbg", Qt_(i + 6, i + 6), -1.0);
    node_.param("aug_ekf/nba", Qt_(i + 9, i + 9), -1.0);
    node_.param("aug_ekf/pnp_p", Rt1_(i, i), -1.0);
    node_.param("aug_ekf/pnp_q", Rt1_(i + 3, i + 3), -1.0);
    node_.param("aug_ekf/vo_pos", Rt2_(i, i), -1.0);
    node_.param("aug_ekf/vo_rot", Rt2_(i + 3, i + 3), -1.0);
  }

  init_        = false;

  for(int i = 0; i < 4; i++)
    latest_idx[i] = 0;

  /* ---------- subscribe and publish ---------- */
  imu_sub_ =
      node_.subscribe<sensor_msgs::Imu>("/dji_sdk_1/dji_sdk/imu", 100, &EKFImuVision::imuCallback, this);
  pnp_sub_     = node_.subscribe<nav_msgs::Odometry>("tag_odom", 10, &EKFImuVision::PnPCallback, this);
  // opti_tf_sub_ = node_.subscribe<geometry_msgs::PointStamped>("opti_tf_odom", 10,
                                                              // &EKFImuVision::opticalCallback, this);
  stereo_sub_  = node_.subscribe<stereo_vo::relative_pose>("/vo/Relative_pose", 10,
                                                          &EKFImuVision::stereoVOCallback, this);
  fuse_odom_pub_ = node_.advertise<nav_msgs::Odometry>("ekf_fused_odom", 10);
  path_pub_         = node_.advertise<nav_msgs::Path>("/aug_ekf/Path", 100);

  ros::Duration(0.5).sleep();

  ROS_INFO("Start ekf.");
}

void EKFImuVision::PnPCallback(const nav_msgs::OdometryConstPtr& msg) {

  // TODO
  // construct a new state using the absolute measurement from marker PnP and process the new state
  // ROS_INFO_STREAM("pnp");
  bool pnp_lost = fabs(msg->pose.pose.position.x) < 1e-4 && fabs(msg->pose.pose.position.y) < 1e-4 &&
      fabs(msg->pose.pose.position.z) < 1e-4;
  if (pnp_lost) return;

  Mat3x3 R_w_b = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                     .toRotationMatrix();
  Vec3 t_w_b;
  t_w_b[0] = msg->pose.pose.position.x;
  t_w_b[1] = msg->pose.pose.position.y;
  t_w_b[2] = msg->pose.pose.position.z;

  AugState         new_state;

  new_state.time_stamp = msg->header.stamp;
  new_state.type = pnp;
  new_state.ut.head(3)  =     t_w_b;
  new_state.ut.segment(3, 3) = rotation2Euler(R_w_b);
  new_state.mean.setZero();
  new_state.covariance.setZero();

  // ROS_INFO_STREAM("pnp ut"<<new_state.ut);

  if (!processNewState(new_state, false)) {
    return;
  }

}

void EKFImuVision::stereoVOCallback(const stereo_vo::relative_poseConstPtr& msg) {

  // TODO
  // label the previous keyframe
  // construct a new state using the relative measurement from VO and process the new state
  // ROS_INFO_STREAM("vo");
  Mat3x3 R_k_c = Eigen::Quaterniond(msg->relative_pose.orientation.w, msg->relative_pose.orientation.x,
                                    msg->relative_pose.orientation.y, msg->relative_pose.orientation.z)
                     .toRotationMatrix();
  Vec3 t_k_c;
  t_k_c[0] = msg->relative_pose.position.x;
  t_k_c[1] = msg->relative_pose.position.y;
  t_k_c[2] = msg->relative_pose.position.z;

  AugState         new_state;

  new_state.time_stamp = msg->header.stamp;
  new_state.key_frame_time_stamp = msg->key_stamp;
  new_state.type = vo;
  new_state.ut.head(3)  =     t_k_c;
  new_state.ut.segment(3, 3) = rotation2Euler(R_k_c);
  new_state.mean.setZero();
  new_state.covariance.setZero();


  if (!processNewState(new_state, false)) {
    return;
  }

}

void EKFImuVision::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

  // TODO
  // construct a new state using the IMU input and process the new state
  // ROS_INFO_STREAM("imu");
  Vec3 angular_velocity;
  Vec3 linear_acceleration;

  angular_velocity[0] = imu_msg->angular_velocity.x;
  angular_velocity[1] = imu_msg->angular_velocity.y;
  angular_velocity[2] = imu_msg->angular_velocity.z;

  linear_acceleration[0] = imu_msg->linear_acceleration.x;
  linear_acceleration[1] = imu_msg->linear_acceleration.y;
  linear_acceleration[2] = imu_msg->linear_acceleration.z;

  AugState         new_state;

  new_state.time_stamp = imu_msg->header.stamp;
  new_state.type = imu;
  new_state.ut.head(3)  =     angular_velocity;
  new_state.ut.segment(3, 3) = linear_acceleration;
  new_state.mean.setZero();
  new_state.covariance.setZero();

  // ROS_INFO_STREAM("imu ut: "<<new_state.ut);

  if (!processNewState(new_state, false)) {
    return;
  }

}

void EKFImuVision::predictIMU(AugState& cur_state, AugState& prev_state, Vec6 ut) {

  // TODO
  // predict by IMU inputs
  double delta_t = (cur_state.time_stamp - prev_state.time_stamp).toSec();
  // ROS_INFO_STREAM("delta_t"<<delta_t); //0.00031~0.00032
  // ROS_INFO_STREAM("predict IMU");

  Vec12 noise_n;
  Vec15 mu_t_1;

  Mat15x15 Ft;
  Mat15x12 Vt;

  Vec15 mu_t_hat;
  Mat15x15 SIGMAt_hat;

  noise_n.setZero();
  mu_t_1 = prev_state.mean.head(15);

  // ROS_INFO_STREAM("mu_t_1: " << mu_t_1);
  
  Ft = Mat15x15::Identity() + delta_t*jacobiFx(mu_t_1, cur_state.ut, noise_n);
  //Ft = delta_t*jacobiFx(mu_t_1, cur_state.ut, noise_n);
  Vt = delta_t*jacobiFn(mu_t_1, cur_state.ut, noise_n);


  mu_t_hat = mu_t_1 + delta_t*modelF(mu_t_1, cur_state.ut, noise_n);
  SIGMAt_hat = Ft*prev_state.covariance.topLeftCorner(15,15)*Ft.transpose() + Vt*Qt_*Vt.transpose();

  cur_state.mean = prev_state.mean;
  cur_state.mean.head(15) = mu_t_hat;
  
  cur_state.covariance = prev_state.covariance;
  cur_state.covariance.topLeftCorner(15, 15) = SIGMAt_hat;
  cur_state.covariance.block(0,15,15,6) = Ft*prev_state.covariance.block(0,15,15,6);
  cur_state.covariance.block(15,0,6,15) = prev_state.covariance.block(15,0,6,15)*Ft.transpose();
  cur_state.mean.segment<3>(3) = angle_limit(cur_state.mean.segment<3>(3));

  // ROS_INFO_STREAM("mu_t_hat: " << mu_t_hat);

}

void EKFImuVision::updatePnP(AugState& cur_state, AugState& prev_state) {

  // TODO
  // update by marker PnP measurements
  Mat6x21 Ct;
  Ct.setZero(6,21);
  Ct.topLeftCorner(6,15) = jacobiG1x(prev_state.mean.head(15), Vec6::Zero(6));
  Mat6x6 Wt = jacobiG1v(prev_state.mean.head(15), Vec6::Zero(6));
  Mat21x6 Kt = prev_state.covariance * Ct.transpose()*(Ct*prev_state.covariance*Ct.transpose()+Wt*Rt1_*Wt.transpose()).inverse();
  
  Vec6 mdiff = (cur_state.ut - modelG1(prev_state.mean.head(15), Vec6::Zero(6)));
  mdiff.segment<3>(3) = angle_limit(mdiff.segment<3>(3));
  cur_state.mean = prev_state.mean;
  cur_state.mean = prev_state.mean + Kt * mdiff;
  cur_state.covariance = prev_state.covariance;
  cur_state.covariance = prev_state.covariance - Kt*Ct*prev_state.covariance;
  cur_state.mean.segment<3>(3) = angle_limit(cur_state.mean.segment<3>(3));
  
}

void EKFImuVision::updateVO(AugState& cur_state, AugState& prev_state) {

  // TODO
  // update by relative pose measurements
  Mat6x21 Ct = jacobiG2x(prev_state.mean, Vec6::Zero(6));
  Mat6x6 Wt = jacobiG2v(prev_state.mean, Vec6::Zero(6));
  Mat21x6 Kt = prev_state.covariance * Ct.transpose() * (Ct * prev_state.covariance * Ct.transpose() + Wt*Rt2_*Wt.transpose()).inverse();

  cur_state.mean = prev_state.mean + Kt * (cur_state.ut - modelG2(prev_state.mean, Vec6::Zero(6)));
  cur_state.covariance = prev_state.covariance - Kt * Ct * prev_state.covariance;

}

void EKFImuVision::changeAugmentedState(AugState& state) {
  ROS_ERROR("----------------change keyframe------------------------");

  // TODO
  // change augmented state
  // Vec15 r_mean = M_r_*state.mean;
  // // state.covariance.topLeftCorner(15,15) = M_r_*state.covariance*M_r_.transpose();
  // Mat15x15 Sigma = M_r_*state.covariance*M_r_.transpose();
  // state.mean = M_a_*r_mean;
  // state.covariance = M_a_*Sigma*M_a_.transpose();

  state.covariance = M_a_ * M_r_ * state.covariance * M_r_.transpose() * M_a_.transpose();
  state.mean = M_a_ * M_r_ * state.mean;
  state.key_frame_time_stamp = state.time_stamp;

}

bool EKFImuVision::processNewState(AugState& new_state, bool change_keyframe) {

  // TODO
  // process the new state
  // step 1: insert the new state into the queue and get the iterator to start to propagate (be careful about the change of key frame).
  // step 2: try to initialize the filter if it is not initialized.
  // step 3: repropagate from the iterator you extracted.
  // step 4: remove the old states.
  // step 5: publish the latest fused odom
  deque<AugState>::iterator start_it=insertNewState(new_state);

  if (!init_){
    init_ = initFilter();
    return false;
  }else{
    repropagate(start_it);
    removeOldState();
    publishFusedOdom();
  }


  return true;

}

deque<AugState>::iterator EKFImuVision::insertNewState(AugState& new_state){
  
  ros::Time time = new_state.time_stamp;
  deque<AugState>::iterator state_it;

  // TODO
  // insert the new state to the queue
  // update the latest_idx of the type of the new state
  // return the iterator point to the new state in the queue 
  int inserted_flag=0;
  unsigned int idx=0;

  for (state_it=aug_state_hist_.begin(); state_it!=aug_state_hist_.end();state_it++)
  {
    if (state_it->time_stamp>time)
    {
      aug_state_hist_.insert(state_it, new_state);
      state_it=aug_state_hist_.begin()+idx;
      inserted_flag=1;
      break;
    }
    idx++;
  }

  if (inserted_flag==0)
  {
    aug_state_hist_.push_back(new_state);
    state_it = aug_state_hist_.end()-1;
  }

  if (state_it->type==vo && latest_idx[vo]!=0)
  {
    if (state_it->key_frame_time_stamp==aug_state_hist_[latest_idx[vo]].time_stamp)
    {
      latest_idx[keyframe] = latest_idx[vo];
      state_it = aug_state_hist_.begin()+latest_idx[keyframe];
      state_it->type=keyframe;
    }
  }

  //update latest idx
  idx=0;
  deque<AugState>::iterator it;
  // ROS_INFO_STREAM("-NEW DEQUE-");
  for (it=aug_state_hist_.begin(); it!=aug_state_hist_.end();it++)
  {
    if (it->type==imu) latest_idx[imu]=idx;
    if (it->type==pnp) latest_idx[pnp]=idx;
    if (it->type==vo) latest_idx[vo]=idx;
    idx++;
  }

  return state_it;

}

void EKFImuVision::repropagate(deque<AugState>::iterator& new_input_it) {


  // TODO
  // repropagate along the queue from the new input according to the type of the inputs / measurements
  // remember to consider the initialization case  
  for (new_input_it; new_input_it!=aug_state_hist_.end();new_input_it++)
  {
    if (new_input_it->type==imu) 
    {
      predictIMU(*new_input_it, *(new_input_it-1), current_imu_ut);
    }
    if (new_input_it->type==pnp) 
    {
      updatePnP(*new_input_it, *(new_input_it-1));
    }
    if (new_input_it->type==vo) 
    {
      updateVO(*new_input_it, *(new_input_it-1));
    }
    if (new_input_it->type==keyframe)
    {
      updateVO(*new_input_it, *(new_input_it-1));
      changeAugmentedState(*new_input_it);
    }
  }

}

void EKFImuVision::removeOldState() {

  // TODO
  // remove the unnecessary old states to prevent the queue from becoming too long

  unsigned int remove_idx = min(min(latest_idx[imu], latest_idx[pnp]), latest_idx[keyframe]);

  aug_state_hist_.erase(aug_state_hist_.begin(), aug_state_hist_.begin() + remove_idx);

  for(int i = 0; i < 4; i++){
    latest_idx[i] -= remove_idx;
  }

  
}

void EKFImuVision::publishFusedOdom() {
  AugState last_state = aug_state_hist_.back();

  double phi, theta, psi;
  phi   = last_state.mean(3);
  theta = last_state.mean(4);
  psi   = last_state.mean(5);


  if (last_state.mean.head(3).norm() > 20) {
    ROS_ERROR_STREAM("error state: " << last_state.mean.head(3).transpose());
    return;
  }

  // using the zxy euler angle
  Eigen::Quaterniond q = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY());
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp    = last_state.time_stamp;

  odom.pose.pose.position.x = last_state.mean(0);
  odom.pose.pose.position.y = last_state.mean(1);
  odom.pose.pose.position.z = last_state.mean(2);

  odom.pose.pose.orientation.w = q.w();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();

  odom.twist.twist.linear.x = last_state.mean(6);
  odom.twist.twist.linear.y = last_state.mean(7);
  odom.twist.twist.linear.z = last_state.mean(8);


  fuse_odom_pub_.publish(odom);

  geometry_msgs::PoseStamped path_pose;
  path_pose.header.frame_id = path_.header.frame_id = "world";
  path_pose.pose.position.x                         = last_state.mean(0);
  path_pose.pose.position.y                         = last_state.mean(1);
  path_pose.pose.position.z                         = last_state.mean(2);
  path_.poses.push_back(path_pose);
  path_pub_.publish(path_);
}


bool EKFImuVision::initFilter() {

  // TODO
  // Initial the filter when a keyframe after marker PnP measurements is available
  ROS_INFO_STREAM("latest idx:"<<latest_idx[imu]<<","<<latest_idx[pnp]<<','<<latest_idx[vo]<<','<<latest_idx[keyframe]);
  int idx=0;
  int vo_idx=0;
  int key_idx=0;
  bool vo_flag=false;
  deque<AugState>::iterator it;
  for (it=aug_state_hist_.begin(); it!=aug_state_hist_.end();it++)
  {
    if (it->type==vo && !vo_flag)
    {
      vo_idx=idx;
      vo_flag=true;
      it->mean.setZero();
      it->covariance = Mat21x21::Identity();
      return true;

    }
    if (vo_flag)
    {
      if (it->type==keyframe)
      {
        return initUsingPnP(aug_state_hist_.begin()+vo_idx);
      }
    }
  }

  return false;
}


bool EKFImuVision::initUsingPnP(deque<AugState>::iterator start_it) {

  // TODO
  // Initialize the absolute pose of the state in the queue using marker PnP measurement.
  // This is only step 1 of the initialization.
  start_it->mean.setZero();
  start_it->mean.head(6) = start_it->ut;
  start_it->covariance.setIdentity();

  deque<AugState>::iterator init_it = start_it+1;
  // repropagate(init_it);
  for (init_it; init_it->type!=keyframe; init_it++)
  {
    if (init_it->type==imu) predictIMU(*init_it, *(init_it-1), current_imu_ut);
    if (init_it->type==pnp) updatePnP(*init_it, *(init_it-1));
    if (init_it->type==vo)
    {
      // init_it->mean = (init_it-1)->mean;
      // init_it->covariance = (init_it-1)->covariance;
      init_it->mean.setZero();
      init_it->covariance = 0.2*Mat21x21::Identity();
    }
  }
  changeAugmentedState(*(init_it-1));
  repropagate(init_it);
  ROS_INFO_STREAM("----initiated-----");

  return true;
}


Vec3 EKFImuVision::rotation2Euler(const Mat3x3& R) {
  double phi   = asin(R(2, 1));
  double theta = atan2(-R(2, 0), R(2, 2));
  double psi   = atan2(-R(0, 1), R(1, 1));
  return Vec3(phi, theta, psi);
}


}  // namespace ekf_imu_vision
