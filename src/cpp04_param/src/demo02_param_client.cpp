/*
    需求：编写参数客户端，获取或修改服务端参数。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.查询参数；
            3-2.修改参数；
        4.创建节点对象指针，调用参数操作函数；
        5.释放资源。
*/
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;
// 3.定义节点类；
class ParamClient : public rclcpp::Node {
public:
 ParamClient() : Node("param_client_node_cpp") {
    RCLCPP_INFO(this->get_logger(), "参数客户端创建");
    //参数1是当前对象依赖的节点
    //参数2是参数服务端的节点名称
    //服务通信没有通过话题来关联，参数服务是通过话题来关联
    //1.参数服务端启动后，底层封装了多个服务通信的服务端
    //2.每个服务端的话题，都是采用/服务端节点名称/xxx
    //3.参数客户端创建后，也会封装多个服务通信的客户端
    //4.这些客户端和服务端呼应，也要使用相同的话题
    param_client_=std::make_shared<rclcpp::SyncParametersClient>(this,"param_server_node_cpp");

  }
  //链接服务端
  bool connect_service()
  {
    while(!param_client_->wait_for_service(1s))
    {
      if(!rclcpp::ok())
      {
      RCLCPP_INFO(this->get_logger(),"中断链接");
        return false;
      }
      RCLCPP_INFO(this->get_logger(),"链接中...");
    }
    return true;
  }
  // 3-1.查询参数；
  void get_param()
  {
      RCLCPP_INFO(this->get_logger(),"-------------参数查询操作------------");
      std::string car_type=param_client_->get_parameter<std::string>("car_type");
      long int car_max_speed=param_client_->get_parameter<long int>("car_max_speed");
      RCLCPP_INFO(this->get_logger(),"car_type:%s",car_type.c_str());
      RCLCPP_INFO(this->get_logger(),"car_max_speed:%ld",car_max_speed);
    //获取多个参数

  }
  // 3-2.修改参数；
  void update_param()
  {
      RCLCPP_INFO(this->get_logger(),"-------------参数修改操作------------");
      auto params=param_client_->set_parameters({rclcpp::Parameter("car_type","Benz"),
      rclcpp::Parameter("car_max_speed",150)
      });
  }
  private:
  rclcpp::SyncParametersClient::SharedPtr param_client_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared <ParamClient>();
  auto flag=node->connect_service();
  if(!flag)
  {
    return 0;
  }
  node->get_param();
  node->update_param();
  node->get_param();
  rclcpp::shutdown();
  return 0;
}