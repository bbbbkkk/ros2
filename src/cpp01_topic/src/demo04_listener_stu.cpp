/*  
  需求：以某个固定频率接受文本学生信息，包含学生的姓名、年龄、身高等数据。
*/
/*1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建订阅方；
      3-3.接受并分析信息
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。*/
    // 1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"
using base_interfaces_demo::msg::Student;
using std::placeholders::_1;

// 3.定义节点类；
class listenerStu : public rclcpp::Node {
public:
  listenerStu() : Node("listener_node_cpp") {
    RCLCPP_INFO(this->get_logger(), "订阅方创建 ！");
    // 3-1.创建订阅方；
    subscription_=this->create_subscription<Student>("chatter_stu",10,std::bind(&listenerStu::do_call_back,this,std::placeholders::_1));
   // 3-2.创建定时器；
    // 3-3.接受并分析信息
  }
  private:
  rclcpp::Subscription<Student>::SharedPtr subscription_;
  void do_call_back(const Student &stu)
  {
    //   3-3.接受并分析信息
    RCLCPP_INFO(this->get_logger(),"订阅的学生信息：name=%s,age=%d,height=%.2f",stu.name.c_str(),stu.age,stu.height);

  }
};

int main(int argc, char ** argv) {
    // 2.初始化 ROS2 客户端；
  rclcpp::init(argc, argv);
  auto node = std::make_shared<listenerStu>();
//   4.调用spin函数，并传入节点对象指针；
  rclcpp::spin(node);
//   5.释放资源
  rclcpp::shutdown();
  return 0;
}