// ROS 2 port of obs_param/src/obs_kf.cpp from jianzhuozhuTHU/MPC-D-CBF.
// Takes clustered obstacle observations, runs one Kalman filter per obstacle
// ID and publishes 25-step predicted future poses. Topics, message layouts
// and all filter parameters are unchanged.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <vector>

#include "kalman.h"

using namespace std;
using namespace Eigen;

static constexpr size_t kObsKfBufferSize = 100;
static constexpr int kPredHorizon = 25;

struct Ellipse
{
  double semimajor;
  double semiminor;
  double cx;
  double cy;
  double theta;
};

struct ObsParam
{
  float x = 0.0, y = 0.0, a = 0.0, b = 0.0, theta = 0.0;
  float mea_cov = 0.0;
};

class ObsKF
{
public:
  int N = kPredHorizon;
  float T = 0.1;

  Kalman ka;
  std::vector<ObsParam> param_list;
  std::vector<ObsParam> pred_list;
  Eigen::VectorXd x;
  Eigen::MatrixXd P0, Q, R, A, B, H;
  Eigen::VectorXd z0, u_v;

  ObsKF() : N(kPredHorizon)
  {
    ka.m_StateSize = 9;
    ka.m_MeaSize = 5;
    ka.m_USize = 9;

    T = 0.1;

    x.resize(9);
    x.setZero();

    float P0_cov = 0.025;
    P0.resize(9, 9);
    P0.setIdentity();
    P0 *= P0_cov;

    float Q_cov = 0.00008;
    Q.resize(9, 9);
    Q.setIdentity();
    Q *= Q_cov;

    float R_cov = 0.0008;
    R.resize(5, 5);
    R.setZero();
    R(0, 0) = R_cov;
    R(1, 1) = R_cov;
    R(2, 2) = 10 * R_cov;
    R(3, 3) = 10 * R_cov;
    R(4, 4) = 0.00000001 * R_cov;

    A.resize(9, 9);
    A.setIdentity();
    A(0, 5) = A(1, 6) = T;

    B.resize(9, 9);
    B.setZero();

    H.resize(5, 9);
    H.setIdentity();

    z0.resize(5);
    z0.setZero();

    u_v.resize(9);
    u_v.setZero();

    ka.Init_Par(x, P0, R, Q, A, B, H, u_v);
  }

  void obs_predict()
  {
    int num = param_list.size();
    if (num == 0)
      return;

    if (num == 1)
    {
      ka.m_x << param_list[0].x, param_list[0].y, param_list[0].a, param_list[0].b, param_list[0].theta, 0, 0, 0, 0;
      ka.m_P = P0;
    }
    else
    {
      float _mea_cov_min = 0.005;
      float _mea_cov_max = 0.05;
      float _k_cov_min = 1;
      float _k_cov_max = 3000;
      float _mea_cov = min(max(param_list.back().mea_cov, _mea_cov_min), _mea_cov_max);
      float _k = log10(_mea_cov / _mea_cov_min) / log10(_mea_cov_max / _mea_cov_min);
      float _k_cov = pow(_k_cov_max, _k) * pow(_k_cov_min, 1 - _k);

      ka.m_R(0, 0) = _k_cov * R(0, 0);
      ka.m_R(1, 1) = _k_cov * R(1, 1);

      VectorXd z(5);
      z << param_list.back().x, param_list.back().y, param_list.back().a, param_list.back().b, param_list.back().theta;

      ka.Predict_State();
      ka.Predict_Cov();
      ka.Mea_Resd(z);
      ka.Cal_Gain();
      ka.Update_State();
      ka.Update_Cov();

      Kalman ka_tmp = ka;

      pred_list.clear();
      for (int i = 0; i < N; i++)
      {
        ka_tmp.Predict_State();
        ka_tmp.Predict_Cov();
        ObsParam pred;
        float x_conf = 0.95 * sqrt(ka_tmp.m_P(0, 0));
        float y_conf = 0.95 * sqrt(ka_tmp.m_P(1, 1));
        float a_conf = 0.95 * sqrt(ka_tmp.m_P(2, 2));
        float b_conf = 0.95 * sqrt(ka_tmp.m_P(3, 3));
        float pos_conf = max(x_conf, y_conf);
        float ab_conf = max(a_conf, b_conf);

        pred.x = ka_tmp.m_x(0);
        pred.y = ka_tmp.m_x(1);
        pred.a = ka_tmp.m_x(2) + pos_conf + ab_conf;
        pred.b = ka_tmp.m_x(3) + pos_conf + ab_conf;
        pred.theta = ka.m_x(4);

        pred_list.push_back(pred);
      }
    }
  }
};

class ObsKfNode : public rclcpp::Node
{
public:
  ObsKfNode() : Node("obs_param_node")
  {
    sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/local_map_pub/for_obs_track", 1,
        std::bind(&ObsKfNode::obscb, this, std::placeholders::_1));
    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/obs_predict_pub", 1);
    vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/obs_predict_vis_pub", 1);
  }

private:
  void visEllipse(const vector<Ellipse> &obs_ellipses)
  {
    visualization_msgs::msg::MarkerArray ellipse_vis;

    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = now();
    mk.ns = "ellipse";
    mk.type = visualization_msgs::msg::Marker::CYLINDER;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (size_t i = 0; i < obs_ellipses.size(); i++)
    {
      mk.color.a = 0.4;
      mk.color.r = 0;
      mk.color.g = 1;
      mk.color.b = 0;

      mk.id = i;

      mk.pose.orientation.x = 0.0;
      mk.pose.orientation.y = 0.0;
      mk.pose.orientation.z = sin(obs_ellipses[i].theta / 2);
      mk.pose.orientation.w = cos(obs_ellipses[i].theta / 2);

      mk.pose.position.x = obs_ellipses[i].cx;
      mk.pose.position.y = obs_ellipses[i].cy;
      mk.pose.position.z = -0.3;

      mk.scale.x = 2 * obs_ellipses[i].semimajor;
      mk.scale.y = 2 * obs_ellipses[i].semiminor;
      mk.scale.z = 0.7;

      ellipse_vis.markers.push_back(mk);
    }

    vis_pub_->publish(ellipse_vis);
  }

  void curve_fitting(ObsKF &obs, std_msgs::msg::Float32MultiArray &obs_pub, vector<Ellipse> &ellipses_array)
  {
    if (obs.param_list.empty())
      return;

    obs.obs_predict();

    for (size_t i = 0; i < obs.pred_list.size(); i++)
    {
      Ellipse ellipse;
      ellipse.cx = obs.pred_list[i].x;
      ellipse.cy = obs.pred_list[i].y;
      ellipse.semimajor = obs.pred_list[i].a;
      ellipse.semiminor = obs.pred_list[i].b;
      ellipse.theta = obs.pred_list[i].theta;
      ellipses_array.push_back(ellipse);

      obs_pub.data.push_back(obs.pred_list[i].x);
      obs_pub.data.push_back(obs.pred_list[i].y);
      obs_pub.data.push_back(obs.pred_list[i].a);
      obs_pub.data.push_back(obs.pred_list[i].b);
      obs_pub.data.push_back(obs.pred_list[i].theta);
    }
  }

  void obscb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "[node] receive the obs track");
    if (msg->data.empty())
      return;

    int num = msg->data.size() / 7;

    std_msgs::msg::Float32MultiArray obs_pub;
    vector<Ellipse> ellipses_array;
    for (int i = 0; i < num; i++)
    {
      int flag = msg->data[7 * i + 5];

      ObsParam _obs_tmp;
      _obs_tmp.x = msg->data[7 * i];
      _obs_tmp.y = msg->data[7 * i + 1];
      _obs_tmp.a = msg->data[7 * i + 2];
      _obs_tmp.b = msg->data[7 * i + 3];
      _obs_tmp.theta = msg->data[7 * i + 4];
      _obs_tmp.mea_cov = msg->data[7 * i + 6];

      _obs_kf[flag].param_list.push_back(_obs_tmp);

      curve_fitting(_obs_kf[flag], obs_pub, ellipses_array);
    }

    pub_->publish(obs_pub);
    visEllipse(ellipses_array);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
  // Kept exactly as in the upstream implementation (fixed array indexed by the
  // obstacle tracking label). local_map hands out monotonically increasing
  // labels, so a long enough run walks past the end of this array -- that is an
  // upstream limitation and is intentionally NOT changed here.
  ObsKF _obs_kf[kObsKfBufferSize];
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObsKfNode>());
  rclcpp::shutdown();
  return 0;
}
