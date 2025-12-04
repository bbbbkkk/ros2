/* 
  需求：读取 bag 文件数据。
  步骤：
    1.包含头文件；
    2.初始化 ROS 客户端；
    3.定义节点类；
      3-1.创建读取对象指针；
      3-2.设置读取的目标文件；
      3-3.读消息；
      3-4.关闭文件。
    4.调用 spin 函数，并传入对象指针；
    5.释放资源。

 */
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "geometry_msgs/msg/twist.hpp"
class SimpleBagPlay : public rclcpp::Node {
public:
  SimpleBagPlay() : Node("simple_bage_play_node") {
    RCLCPP_INFO(this->get_logger(), "消息回访对象创建！");
    reader_=std::make_unique<rosbag2_cpp::Reader>();
    reader_->open("my_bag");
    while(reader_->has_next())
    {
      auto reader=reader_->read_next<geometry_msgs::msg::Twist>();
      RCLCPP_INFO(this->get_logger(),"线速度：%.2f,角速度：%.2f",reader.linear.x,reader.angular.z);

    }
    reader_->close();

  }
  private:
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleBagPlay>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}