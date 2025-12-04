/*
    需求：编写参数服务端，设置并操作参数。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.声明参数；
            3-2.查询参数；
            3-3.修改参数；
            3-4.删除参数。
        4.创建节点对象指针，调用参数操作函数，并传递给spin函数；
        5.释放资源。

*/
#include "rclcpp/rclcpp.hpp"

class MyParam : public rclcpp::Node {
public:
  MyParam() : Node("my_param_node_cpp") {
    RCLCPP_INFO(this->get_logger(), "参数API创建！");
    //3-1.声明参数；
    rclcpp::Parameter p1("car_name","Porsche");
    rclcpp::Parameter p2("car_height",1.68);
    rclcpp::Parameter p3("car_width",4);
    //解析对象
    RCLCPP_INFO(this->get_logger(),"car_name:%s",p1.as_string().c_str());
    RCLCPP_INFO(this->get_logger(),"car_height:%.2f",p2.as_double());
    RCLCPP_INFO(this->get_logger(),"car_width:%ld",p3.as_int());
    //获取参数键
    RCLCPP_INFO(this->get_logger(),"name:%s",p1.get_name().c_str());
    RCLCPP_INFO(this->get_logger(),"type:%s",p2.get_type_name().c_str());
    RCLCPP_INFO(this->get_logger(),"value_to_string:%s",p3.value_to_string().c_str());

  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyParam>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}