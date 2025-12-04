/* 
  需求：录制 turtle_teleop_key 节点发布的速度指令。
  步骤：
    1.包含头文件；
    2.初始化 ROS 客户端；
    3.定义节点类；
      3-1.创建写出对象指针；
      3-2.设置写出的目标文件；
      3-3.写出消息。
    4.调用 spin 函数，并传入对象指针；
    5.释放资源。

 */
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rosbag2_cpp/writer.hpp"
using std::placeholders::_1;

class SimpleBagRecord : public rclcpp::Node {
public:
  SimpleBagRecord() : Node("simple_bag_record_node"){
    RCLCPP_INFO(this->get_logger(), "录制消息对象创建！");
    writer_=std::make_unique<rosbag2_cpp::Writer>();
    writer_->open("my_bag");//相对路径，是工作空间 的直接子集
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10,std::bind(&SimpleBagRecord::do_write,this,std::placeholders::_1));
    // writer_->write();

  }
  private:
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    void do_write(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      // void write<MessageT>(const MessageT &message, 
      //   const std::string &topic_name, 
      //   const rclcpp::Time &time)
      RCLCPP_INFO(this->get_logger(),"数据写出...");
      writer_->write(*msg,"/turtle1/cmd_vel",this->now());
    }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleBagRecord>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
