/*  
  需求：编写服务端，接收客户端发送请求，提取其中两个整型数据，相加后将结果响应回客户端。
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建服务端；
      3-2.处理请求数据并响应结果。
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。
*/
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"
/* srv/AddInts.srv 
C++会生成头文件为
先把 AddInts 变成全小写：addints
再按驼峰拆成单词并加下划线：add_ints
最后加后缀 .hpp
*/
using base_interfaces_demo::srv::AddInts;
using std::placeholders::_1;
using std::placeholders::_2;
class AddIntsServer : public rclcpp::Node {
public:
  AddIntsServer() : Node("add_ints_server_node") {
    RCLCPP_INFO(this->get_logger(),"服务端节点创建 ！");
    server_=this->create_service<AddInts>("add_ints",std::bind(&AddIntsServer::add,this,_1,_2));
    
  }
  private:
  rclcpp::Service<AddInts>::SharedPtr server_;
  void add(const AddInts::Request::SharedPtr req,const AddInts::Response:: SharedPtr res){
    // 3-2.处理请求数据并响应结果。
    res->sum=req->num1+req->num2;
    RCLCPP_INFO(this->get_logger(),"%d+%d=%d",req->num1,req->num2,res->sum);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto server = std::make_shared<AddIntsServer>();
  rclcpp::spin(server);
  rclcpp::shutdown();
  return 0;
}