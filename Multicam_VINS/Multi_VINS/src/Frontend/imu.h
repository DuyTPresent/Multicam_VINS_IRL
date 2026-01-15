#pragma once

#include <sensor_msgs/Imu.h>
#include <cmath>

class Butter2 {
 public:
  const int n = 2;
  const int fs = 200;
  const int fc = 15;
  
  double tan_ = tan(M_PI * fc / fs);
  double gamma = 1 / (tan_);
  double alphak = 2 * cos(2 * M_PI * (n + 1) / (4 * n));

  Butter2() {
    gain_ = gamma * gamma - alphak * gamma + 1;
    a_[0] = -(gamma * gamma + alphak * gamma + 1) / gain_;
    a_[1] = -(2 - 2 * gamma * gamma) / gain_;
    for (int i = 0; i < 3; i++) {
      xs_[i] = 0.0;
      ys_[i] = 0.0;
    }
    initalised = false;  
  }

  double apply(double sample) {
    if (!initalised) {
      initalised = true;
      return reset(sample);
    }
    xs_[0] = xs_[1];
    xs_[1] = xs_[2];
    xs_[2] = sample / gain_;
    ys_[0] = ys_[1];
    ys_[1] = ys_[2];
    ys_[2] =
        (xs_[0] + xs_[2]) + 2 * xs_[1] + (a_[0] * ys_[0]) + (a_[1] * ys_[1]);
    return ys_[2];
  }

  double reset(double sample) {
    xs_[0] = sample;
    xs_[1] = sample;
    xs_[2] = sample;
    ys_[0] = sample;
    ys_[1] = sample;
    ys_[2] = sample;
    return sample;
  }

 private:
  bool initalised = false;
  double a_[2] = {0.0, 0.0};
  double gain_ = 1.0;
  double xs_[3] = {0.0, 0.0, 0.0};
  double ys_[3] = {0.0, 0.0, 0.0};
};

// ===================== FILTER FUNCTION =====================
inline void filter(const sensor_msgs::ImuConstPtr& data, sensor_msgs::Imu& imu_msg)
{
  static Butter2 butter_ax;
  static Butter2 butter_ay;
  static Butter2 butter_az;
  static Butter2 butter_wx;
  static Butter2 butter_wy;
  static Butter2 butter_wz;

  sensor_msgs::Imu msg_out = *data;

  msg_out.linear_acceleration.x = butter_ax.apply(data->linear_acceleration.x);
  msg_out.linear_acceleration.y = butter_ay.apply(data->linear_acceleration.y);
  msg_out.linear_acceleration.z = butter_az.apply(data->linear_acceleration.z);

  msg_out.angular_velocity.x = butter_wx.apply(data->angular_velocity.x);
  msg_out.angular_velocity.y = butter_wy.apply(data->angular_velocity.y);
  msg_out.angular_velocity.z = butter_wz.apply(data->angular_velocity.z);

  imu_msg.header.stamp = data->header.stamp;
  imu_msg.header.frame_id = "base_link";
  imu_msg.orientation = data->orientation;
  imu_msg.orientation_covariance = data->orientation_covariance;

  imu_msg.angular_velocity.x = msg_out.angular_velocity.x;
  imu_msg.angular_velocity.y = msg_out.angular_velocity.y;
  imu_msg.angular_velocity.z = msg_out.angular_velocity.z;
  imu_msg.angular_velocity_covariance = data->angular_velocity_covariance;

  imu_msg.linear_acceleration.x = msg_out.linear_acceleration.x;
  imu_msg.linear_acceleration.y = msg_out.linear_acceleration.y;
  imu_msg.linear_acceleration.z = msg_out.linear_acceleration.z;
  imu_msg.linear_acceleration_covariance = data->linear_acceleration_covariance;
}