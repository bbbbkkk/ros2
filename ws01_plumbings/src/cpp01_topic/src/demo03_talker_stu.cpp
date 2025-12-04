/*  
  需求：以某个固定频率发送文本学生信息，包含学生的姓名、年龄、身高等数据。
*/
/*1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建发布方；
      3-2.创建定时器；
      3-3.组织消息并发布。
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。*/

    // 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"
using base_interfaces_demo::msg::Student;
using namespace std::chrono_literals;
// 3.定义节点类；
class talkerstu : public rclcpp::Node {
public:
  talkerstu() : Node("talker_node_cpp"),age(0){
    RCLCPP_INFO(this->get_logger(), "发送方创建 ！");
    //  3-1.创建发布方；
    publisher_=this->create_publisher<Student>("chatter_stu",10);
    // 3-2.创建定时器；
    timer_=this->create_wall_timer(500ms,std::bind(&talkerstu::on_timer,this));

  }
  private:
  rclcpp::Publisher<Student>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int age;
//   3-3.组织消息并发布。
  void on_timer()
  {
    auto stu=Student();
    //Student.msg创建后，编译以后会自动创建struct Student
    //这里是调用这个Student的默认构造 Student()
    stu.name="zhangsan";
    stu.age=age;
    stu.height=1.70;
    age++;
    publisher_->publish(stu);
    RCLCPP_INFO(this->get_logger(), "发布信息：(%s,%d,%.2f)",stu.name.c_str(),stu.age,stu.height);
  }

};

int main(int argc, char ** argv) {
    // 2.初始化 ROS2 客户端；
  rclcpp::init(argc, argv);
  auto node = std::make_shared<talkerstu>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

