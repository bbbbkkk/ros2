/*  
    需求：发布雷达坐标系中某个坐标点相对于雷达（laser）坐标系的位姿。
    步骤：
        1.包含头文件；
        2.初始化 ROS 客户端；
        3.定义节点类；
            3-1.创建坐标点发布方；
            3-2.创建定时器；
            3-3.组织并发布坐标点消息。
        4.调用 spin 函数，并传入对象指针；
        5.释放资源。


*/
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
using namespace std::chrono_literals;
class PointBroadcaster : public rclcpp::Node {
public:
  PointBroadcaster() : Node("point_broadcaster_node") {
    RCLCPP_INFO(this->get_logger(), "坐标点创建！");
    publisher_=this->create_publisher<geometry_msgs::msg::PointStamped>("point",10);
    timer_=create_wall_timer(0.5s,std::bind(&PointBroadcaster::on_timer,this));
  }
  private:
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double x=0.0;
  void on_timer()
  {
    geometry_msgs::msg::PointStamped ps;
    ps.header.stamp=this->now();
    ps.header.frame_id="laser";
    x+=0.05;
    ps.point.x=x;
    ps.point.y=0.0;
    ps.point.z=-0.1;

    publisher_->publish(ps);
  }

};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}