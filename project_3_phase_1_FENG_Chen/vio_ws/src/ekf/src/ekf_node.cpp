#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include<cmath>

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
ros::Publisher odom_p_pub;
MatrixXd Q = MatrixXd::Identity(12, 12);// gyro + acc + gyro bias + acc bias
MatrixXd Rt = MatrixXd::Identity(6,6);// 3D pose and 3D orientation

Eigen::Vector3d gravity_acc(0, 0, 9.8);
Eigen::VectorXd state = VectorXd::Zero(15);// position3, orientation3, linear velocity3, gyro bias3, acc bias3
Eigen::MatrixXd state_cov = 0.5*MatrixXd::Identity(15,15);//covariance matrix

double imu_last_t = 0.0;//imu last timestamp
bool vo_in = false;//whether vo is used


double norm_angle(double angle)
{
    double norm = atan2(sin(angle),cos(angle)); 
    return norm;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    double dt = msg->header.stamp.toSec() - imu_last_t;
    if(!vo_in || dt>1){
        imu_last_t = msg->header.stamp.toSec();
        return;
    }
    //accerlation
    Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    //angular velocity
    Vector3d angular_vec(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
    Matrix3d G, R;
    double phi =state(3);
    double theta=state(4);
    double psi=state(5);
    G << cos(theta), 0, -cos(phi)*sin(theta),
         0, 1, sin(phi),
         sin(theta), 0, cos(theta)*cos(phi);
    
    R << cos(psi)*cos(theta)- sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi),
        cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi), sin(psi)*sin(theta)-cos(psi)*sin(phi)*cos(theta),
            -cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta);
    
    //mean propagation
    VectorXd ft = VectorXd::Zero(15);//f(state_t-1, input_t, 0)
    ft.block<3,1>(0,0) = state.block<3,1>(6,0);//linear velocity
    ft.block<3,1>(3,0) = (G.inverse()) * (angular_vec-Vector3d(state(9),state(10),state(11)));//angle_dot
    ft.block<3,1>(6,0) = gravity_acc + R * (acc-Vector3d(state(12),state(13),state(14)));//acceleration

    state += dt*ft;// mean_t = mean_t-1 + dt*f(state_t-1, input_t, 0)
    state(3) = norm_angle(state(3));
    state(4) = norm_angle(state(4));
    state(5) = norm_angle(state(5));

    //covariance propagation
    Eigen::MatrixXd At = MatrixXd::Zero(15,15);// df/dx partial derivative -- Jacobian matrix
    Eigen::MatrixXd Ut = MatrixXd::Zero(15,12);// df/dn partial derivative -- Jacobian matrix

    Ut.block<3,3>(3,0) << -(G.inverse());
    Ut.block<3,3>(6,3) << -R;
    Ut.block<6,6>(9,6) << MatrixXd::Identity(6,6); // according to imu covariance derivative

    Matrix3d G_inv = G.inverse();
    
    Matrix3d G_inverse_dot, R_dot;

    double x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15;
    double n1,n2,n3,n4,n5,n6,w1,w2,w3;
    x4 = state(3); x5 = state(4), x6 = state(5), x7 = state(6), x8 = state(7), x9 = state(8), x10 = state(9), x11 = state(10), x12 = state(11), x13 = state(12), x14 = state(13), x15 = state(14);
    n1 = 0, n2 = 0, n3 = 0, n4= 0, n5 = 0, n6 = 0;
    w1 = angular_vec(0), w2 = angular_vec(1), w3 = angular_vec(2);
    G_inverse_dot << 0, (sin(x5)*(n1 - w1 + x10))/(cos(x5)*cos(x5) + sin(x5)*sin(x5)) - (cos(x5)*(n3 - w3 + x12))/(cos(x5)*cos(x5) + sin(x5)*sin(x5)), 0, 
                 (cos(x4)*cos(x5)*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) - (cos(x4)*sin(x5)*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(5)) + (cos(x5)*sin(x4)*(sin(x4)*cos(x5)*cos(x5) + sin(x4)*sin(x5)*sin(x5))*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5))*(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) - (sin(x4)*sin(x5)*(sin(x4)*cos(x5)*cos(x5) + sin(x4)*sin(x5)*sin(x5))*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5))*(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)), 
                 - (sin(x4)*sin(x5)*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) - (cos(x5)*sin(x4)*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)), 0,
                (sin(x5)*(sin(x4)*cos(x5)*cos(x5) + sin(x4)*sin(x5)*sin(x5))*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5))*(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) - (cos(x5)*(sin(x4)*cos(x5)*cos(x5) + sin(x4)*sin(x5)*sin(x5))*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5))*(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)), (cos(x5)*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) + (sin(x5)*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)), 0;
 
    
    R_dot << acc(1)*sin(phi)*sin(psi) + acc(2)*cos(phi)*cos(theta)*sin(psi) - acc(0)*cos(phi)*sin(theta)*sin(psi), acc(2)*(cos(theta)*cos(psi) - sin(phi)*sin(theta)*sin(psi)) - acc(0)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)), - acc(0)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - acc(2)*(sin(theta)*sin(psi) - cos(theta)*cos(psi)*sin(phi)) - acc(1)*cos(phi)*cos(psi),
           acc(0)*cos(phi)*cos(psi)*sin(theta) - acc(2)*cos(phi)*cos(theta)*cos(psi) - acc(1)*cos(psi)*sin(phi), acc(2)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - acc(0)*(sin(theta)*sin(psi) - cos(theta)*cos(psi)*sin(phi)),   acc(0)*(cos(theta)*cos(psi) - sin(phi)*sin(theta)*sin(psi)) + acc(2)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - acc(1)*cos(phi)*sin(psi),
           acc(1)*cos(phi) - acc(2)*cos(theta)*sin(phi) + acc(0)*sin(phi)*sin(theta), - acc(0)*cos(phi)*cos(theta) - acc(2)*cos(phi)*sin(theta), 0;

    At.block<3,3>(0,6) = MatrixXd::Identity(3,3);
    At.block<3,3>(3,3) << G_inverse_dot;
    At.block<3,3>(6,3) << R_dot;
    At.block<3,3>(3,9) << -G_inv;
    At.block<3,3>(6,12) << -R;

    MatrixXd Ft = MatrixXd::Identity(15,15) + dt*At;
    MatrixXd Vt = dt*Ut;

    state_cov = Ft * state_cov * Ft.transpose() + Vt * Q * Vt.transpose();// state covariance propagation

    imu_last_t = msg->header.stamp.toSec();

}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //your code for update
    // camera position in the IMU frame = (0.05, 0.05, 0)
    // camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //							             0, -1, 0,
    //                                       0, 0, -1;
    Eigen::Vector3d t_cw, t_ic, t_wi;
    Eigen::Matrix3d R_cw, R_ic, R_wi;

    Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    t_cw << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z; 
    R_cw << q.toRotationMatrix();

    t_ic << 0.05, 0.05, 0;
    R_ic << 1,  0,  0,
            0, -1,  0,
            0,  0, -1;
    
    R_wi = R_cw.transpose() * R_ic.transpose();
    t_wi = -R_cw.transpose()*R_ic.transpose()*t_ic - R_cw.transpose()*t_cw;

    double phi = asin(R_wi(2, 1));
    double theta = atan2(-R_wi(2, 0) / cos(phi), R_wi(2, 2) / cos(phi));
    double psi = atan2(-R_wi(0, 1) / cos(phi), R_wi(1, 1) / cos(phi));

    Eigen::VectorXd zt = VectorXd::Zero(6);//position and orientation measurement
    Eigen::VectorXd gain = VectorXd::Zero(6);// gain value for Kalman gain

    Eigen::MatrixXd Ct = MatrixXd::Zero(6,15);//Ct for linearization
    Eigen::MatrixXd Kt;//Kalman gain
    zt << t_wi(0), t_wi(1), t_wi(2), phi, theta, psi;

    Ct.block(0,0,6,6).setIdentity(6,6);//partial derivative g/x
    gain = zt - Ct*state;

    gain(3) = norm_angle(gain(3));
    gain(4) = norm_angle(gain(4));
    gain(5) = norm_angle(gain(5));

    Kt = (Ct*state_cov*Ct.transpose()+Rt).lu().solve(Ct*state_cov).transpose(); // LU solver to obtain Kalman gain
    
    state = state + Kt*gain;
    state_cov = state_cov - Kt*Ct*state_cov;

    Eigen::Matrix3d R_ekf;
    R_ekf << cos(state(5))*cos(state(4))- sin(state(3))*sin(state(5))*sin(state(4)), -cos(state(3))*sin(state(5)), cos(state(5))*sin(state(4))+cos(state(4))*sin(state(3))*sin(state(5)),
          cos(state(4))*sin(state(5))+cos(state(5))*(sin(state(3)))*sin(state(4)), cos(state(3))*cos(state(5)), sin(state(5))*sin(state(4))-cos(state(5))*sin(state(3))*cos(state(4)),
          -cos(state(3))*sin(state(4)), sin(state(3)), cos(state(3))*cos(state(4));
    
    Quaterniond Q_p(R_wi);
    nav_msgs::Odometry p_vio;
    p_vio.header.stamp = msg->header.stamp;
    p_vio.header.frame_id = "world";
    p_vio.pose.pose.position.x = t_wi(0);
    p_vio.pose.pose.position.y = t_wi(1);
    p_vio.pose.pose.position.z = t_wi(2);
    p_vio.pose.pose.orientation.w = Q_p.w();
    p_vio.pose.pose.orientation.x = Q_p.x();
    p_vio.pose.pose.orientation.y = Q_p.y();
    p_vio.pose.pose.orientation.z = Q_p.z();
    odom_p_pub.publish(p_vio);

    //publish vio
    Quaterniond Q_ekf(R_ekf);
    nav_msgs::Odometry ekf_vio;
    ekf_vio.header.stamp = msg->header.stamp;
    ekf_vio.header.frame_id = "world";
    ekf_vio.pose.pose.position.x = state(0);
    ekf_vio.pose.pose.position.y = state(1);
    ekf_vio.pose.pose.position.z = state(2);
    ekf_vio.twist.twist.linear.x = state(6);
    ekf_vio.twist.twist.linear.y = state(7);
    ekf_vio.twist.twist.linear.z = state(8);
    ekf_vio.pose.pose.orientation.w = Q_ekf.w();
    ekf_vio.pose.pose.orientation.x = Q_ekf.x();
    ekf_vio.pose.pose.orientation.y = Q_ekf.y();
    ekf_vio.pose.pose.orientation.z = Q_ekf.z();
    odom_pub.publish(ekf_vio);
    vo_in = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    odom_p_pub = n.advertise<nav_msgs::Odometry>("p_odom", 100);
    Rcam = Quaterniond(0, 1, 0, 0).toRotationMatrix();
    cout << "R_cam" << endl << Rcam << endl; 
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    // You should also tune these parameters
    Q.topLeftCorner(6, 6) = 0.1 * Q.topLeftCorner(6, 6);
    Q.bottomRightCorner(6, 6) = 0.04 * Q.bottomRightCorner(6, 6);
    Rt = 0.05*MatrixXd::Identity(6,6);

    ros::spin();
}