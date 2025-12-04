#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
using std::placeholders::_1;
class TFDyBroadcaster : public rclcpp::Node {
public:
  TFDyBroadcaster() : Node("tf_dy_broadcaster_node") {
    RCLCPP_INFO(this->get_logger(), "动态广播创建！");
    //这里不用创建话题，自动帮我们创建好了
    broadcaster_=std::make_shared<tf2_ros::TransformBroadcaster>(this);
    pose_ =this->create_subscription<turtlesim::msg::Pose>("turtle1/pose",10,std::bind(&TFDyBroadcaster::do_pose,this,std::placeholders::_1));
  }
  private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_;
  void do_pose(const turtlesim::msg::Pose & pose)
  {
    //组织消息
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp=this->now();
    ts.header.frame_id="world";
    ts.child_frame_id="turtle1";
    ts.transform.translation.x=pose.x;
    ts.transform.translation.y=pose.y;
    ts.transform.translation.z=0;
    //欧拉角转换为四元数
    //乌龟的欧拉角只有yaw的取值
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,pose.theta);
    ts.transform.rotation.x=qtn.getX();
    ts.transform.rotation.y=qtn.getY();
    ts.transform.rotation.z=qtn.getZ();
    //发布
    broadcaster_->sendTransform(ts);
  }
};


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFDyBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}