 /*需求：订阅发布方发布的消息，并输出到终端。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建订阅方；
            3-2.处理订阅到的消息。
        4.调用spin函数，并传入节点对象指针；
        5.释放资源。
*/
//  1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

// 3.定义节点类；
 class Listener:public rclcpp::Node{
    public:
    Listener():Node("listen_node_cpp")
    {
    RCLCPP_INFO(this->get_logger(),"订阅方创建！");
    // 3-1.创建订阅方；
    subscription_=this->create_subscription<std_msgs::msg::String>("chatter",10,std::bind(&Listener::do_call_back,this,std::placeholders::_1));
    
    // 3-2.处理订阅到的消息。
    }
    private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    void do_call_back(const std_msgs::msg::String &msg)
    {
        RCLCPP_INFO(this->get_logger(),"订阅方消息是：%s",msg.data.c_str());
    }

};
int main(int argc, char ** argv)
{
  //2.初始化 ROS2 客户端；
  rclcpp::init(argc,argv);
  //4.调用spin函数，并传入节点对象指针；
  rclcpp::spin(std::make_shared<Listener>());
  //5.释放资源。
  rclcpp::shutdown();

  return 0;
}