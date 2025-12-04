/*  
  需求：编写客户端，发送两个整型变量作为请求数据，并处理响应结果。
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建客户端；
      3-2.等待服务连接；
      3-3.组织请求数据并发送；
    4.创建对象指针调用其功能,并处理响应；
    5.释放资源。

*/
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"
using base_interfaces_demo::srv::AddInts;
using namespace std::chrono_literals;
class AddIntsClient : public rclcpp::Node {
public:
  AddIntsClient() : Node("add_ints_client_node") {
    RCLCPP_INFO(this->get_logger(), "客户端节点创建 ！");
    client_=this->create_client<AddInts>("add_ints");
  }
    // 链服务器，如果成功返回true,错误返回false
    bool connect_server()
    {
        //在制定时间链接服务器，如果链接上就返回true，否则返回false
        //client->wait_for_service(1s)函数
        //这里循环为1s为超时时间链接服务器，直到链接到服务器才推出循环
        while(!client_->wait_for_service(1s))
        {
            //在建立链接中的过程中，按下ctrl+c是不会结束进程的，因为
            //当你按下 Ctrl+C 的时候，操作系统确实给进程发了 SIGINT
            //但你的程序根本没有机会去处理这个信号，
            // 因为它只会一圈圈卡在 while 里面——你没有任何条件是“收到 SIGINT 要退出”的
            //所以要对ctrl+c进行处理
            //使用rclcpp::ok()函数进行判断全局状态标志
            //按下ctrl+c就会捕获SIGINT,将状态设置为false
            if(!rclcpp::ok())
            {
            RCLCPP_INFO(this->get_logger(),"强行终止客户端");

                return false;
            }
            RCLCPP_INFO(this->get_logger(),"服务连接中，请稍候...");
        }
        return true;
    }
    //发送请求
    //编写请求函数--参数是两个整形的数据，返回值是提交请求的服务端的返回结果
    rclcpp::Client<AddInts>::FutureAndRequestId send_request(int num1,int num2)
    {
        auto request=std::make_shared<AddInts::Request>();
        request->num1=num1;
        request->num2=num2;
        return client_->async_send_request(request);
    }

  private:
  rclcpp::Client<AddInts>::SharedPtr client_;
};

int main(int argc, char ** argv) {
    if(argc!=3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"请提交两个整形数字");
        return 1;
    }
  rclcpp::init(argc, argv);
  auto client = std::make_shared<AddIntsClient>();
  //调用客户端对象的连接服务器功能
  bool flag=client->connect_server();
  //根据连接结果做进一步处理
  if(!flag)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务器连接失败，程序退出 ！");
    return 0;
  }
  //发送请求
//调用请求函数，接受并处理响应
auto future =client->send_request(atoi(argv[1]),atoi(argv[2]));
//处理响应
//“一边帮你跑回调，一边等这个 future 完成，直到成功 / 超时 / 被打断。”
if(rclcpp::spin_until_future_complete(client,future)==rclcpp::FutureReturnCode::SUCCESS)
{
    RCLCPP_INFO(client->get_logger(),"响应成功！sum=%d",future.get()->sum);
    
}
else{
    RCLCPP_INFO(client->get_logger(),"响应失败！");
}
  rclcpp::shutdown();
  return 0;
}