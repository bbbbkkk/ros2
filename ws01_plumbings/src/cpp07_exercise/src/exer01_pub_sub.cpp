/*
   需求：订阅窗口1中的乌龟速度，然后生成控制窗口2乌龟运动的指令并发布。
   步骤：
        订阅话题: /turtle1/pose
        订阅消息: turtlesim/msg/Pose
        x: 0.0
        y: 0.0
        theta: 0.0
        linear_velocity: 0.0
        angular_velocity: 0.0

        发布话题: /t2/turtle1/cmd_vel
        发布消息: geometry_msgs/msg/Twist
        linear:
          x: 0.0 ---- 前后
          y: 0.0 ---- 左右
          z: 0.0 ---- 上下
        angular:
          x: 0.0 ---- 翻滚
          y: 0.0 ---- 俯仰
          z: 0.0 ---- 左右转


       1.包含头文件；
       2.初始化 ROS2 客户端；
       3.定义节点类；
          3-1.创建控制第二个窗体乌龟运动的发布方；
          3-2.创建订阅第一个窗体乌龟pose的订阅方；
          3-3.根据订阅的乌龟的速度生成控制窗口2乌龟运动的速度消息并发布。
       4.调用spin函数，并传入节点对象指针；
       5.释放资源。
*/
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
using std::placeholders::_1;
class Exer01PubSub : public rclcpp::Node {
public:
  Exer01PubSub() : Node("exer01_pub_sub_node") {
    RCLCPP_INFO(this->get_logger(),"案例1对象创建成功！");
    pub_=this->create_publisher<geometry_msgs::msg::Twist>("/t2/turtle1/cmd_vel",10);
    sub_=this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(&Exer01PubSub::pose_cb,this,std::placeholders::_1));

  }
  private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
  void pose_cb(const turtlesim::msg::Pose & pose)
  {
    //订阅方的回调函数，处理速度，生成并发布控制乌龟2的速度指令
    //创建速度指令
    geometry_msgs::msg::Twist twist;
    twist.linear.x=pose.linear_velocity;
    twist.angular.z=-pose.angular_velocity;//镜像运动，角速度取反
    //发布
    pub_->publish(twist);
  }
  
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Exer01PubSub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}