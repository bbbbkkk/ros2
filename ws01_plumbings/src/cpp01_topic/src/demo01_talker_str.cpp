 /*1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建发布方；
      3-2.创建定时器；
      3-3.组织消息并发布。
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。*/
// ********************************
//1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
//可以在设置定时器的时，传入的时间间隔设置为数字+单位 eg:1s
//3.定义节点类；
class Talker:public rclcpp::Node
{
  public:
  Talker():Node("talker_node_cpp"),count_(0)
  {
    RCLCPP_INFO(this->get_logger(),"发布节点创建！");
    // 3-1.创建发布方；
    /*模板：
    参数：
      1.话题名称
      2.QOS(消息队列长度)
    返回值：发布对象指针
    */
    publisher_=this->create_publisher<std_msgs::msg::String>("chatter",10);
    //create_publisher：在当前节点上创建一个话题发布器
    // 3-2.创建定时器
    timer_=this->create_wall_timer(1s,std::bind(&Talker::on_timer,this));
    //第一个参数是间隔时间
    //第二个参数是回调函数 必须先绑上对象（this），用 bind 或 lambda 包一层，才能当回调。
  }
  private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  /*
    1. rclcpp::Publisher<...>是模板类，里面需要填写模板
    2. std_msgs::msg::String 
      std_msgs 标准消息包
      msg 是子命名空间
      String 是消息类型
      SharedPtr是智能指针
    3. publisher_成员便变量
  */
 rclcpp::TimerBase::SharedPtr timer_;
 size_t count_;
 //TimerBase就是 ROS2 里所有定时器的“抽象基类"
  void on_timer()
  {
    auto message=std_msgs::msg::String();
    message.data="hello world!"+std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(),"发布者发布消息：%s",message.data.c_str());
    publisher_->publish(message);
    
  }
  
};

int main(int argc, char ** argv)
{
  //2.初始化 ROS2 客户端；
  rclcpp::init(argc,argv);
  //4.调用spin函数，并传入节点对象指针；
  rclcpp::spin(std::make_shared<Talker>());
  //5.释放资源。
  rclcpp::shutdown();

  return 0;
}
