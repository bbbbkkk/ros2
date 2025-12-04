/*  
  需求：编写动作服务端实习，可以提取客户端请求提交的整型数据，并累加从1到该数据之间的所有整数以求和，
       每累加一次都计算当前运算进度并连续反馈回客户端，最后，在将求和结果返回给客户端。
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建动作服务端；
      3-2.处理请求数据；
      3-3.处理取消任务请求；
      3-4.生成连续反馈。
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。

*/
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"
using std::placeholders::_1;
using std::placeholders::_2;
using base_interfaces_demo::action::Progress;

class ProgressActionServer : public rclcpp::Node {
public:
  ProgressActionServer() : Node("progress_action_server_node_cpp") {
    RCLCPP_INFO(this->get_logger(), "action服务端创建！");
    // 3-1.创建动作服务端；

    // 3-2.处理请求数据；
      // 3-3.处理取消任务请求；
      // 3-4.生成连续反馈。
      // create_server<ActionT, NodeT>(NodeT node, 
      //   const std::string &name, 
      //   rclcpp_action::Server<ActionT>::GoalCallback handle_goal, 
      //   rclcpp_action::Server<ActionT>::CancelCallback handle_cancel, 
      //   rclcpp_action::Server<ActionT>::AcceptedCallback handle_accepted, c
      //   onst rcl_action_server_options_t &options = rcl_action_server_get_default_options(), 
      //   rclcpp::CallbackGroup::SharedPtr group = nullptr)
    server_=rclcpp_action::create_server<Progress>(this,
      "get_sum",
      std::bind(&ProgressActionServer::handle_goal,this,_1,_2),
      std::bind(&ProgressActionServer::handle_cancel,this,_1),
      std::bind(&ProgressActionServer::handle_accepted,this,_1)
    );
  }
  private:
    rclcpp_action::Server<Progress>::SharedPtr server_;
    // 3-2.处理请求数据；
    // std::function<rclcpp_action::GoalResponse (const rclcpp_action::GoalUUID &, std::shared_ptr<const ActionT::Goal>)> 
     rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Progress::Goal> goal){
      //判断的数字是否大于1,是就接受，不是就拒绝
      (void)uuid;
      if(goal->num<=1)
      {
        RCLCPP_INFO(this->get_logger(),"提交的目标值必须大于1！");

        return rclcpp_action::GoalResponse::REJECT;
      }
      RCLCPP_INFO(this->get_logger(),"提交的目标值合法！");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    // 3-3.处理取消任务请求；
    // std::function<rclcpp_action::CancelResponse (std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)>
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle){
      (void)goal_handle;
      RCLCPP_INFO(this->get_logger(),"接受任务取消！");
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle){
      //1.生成连续反馈返回客户端
      // 无返回值 
      //参数 std::shared_ptr<base_interfaces_demo::action::Progress_Feedback> feedback_msg)
      // goal_handle->publish_feedback();
      //首先获取目标值，然后遍历，遍历中进行累加，且每次循环一次就计算进度，并且连续返回反馈发布
      int num=goal_handle->get_goal()->num;
      int sum=0;
      auto feedback=std::make_shared<Progress::Feedback>();
      rclcpp::Rate rate(1.0);
      //创建一个“频率为 1Hz 的节拍器”,每隔一秒之后在执行循环
      auto result=std::make_shared<Progress::Result>();
      for(int i=0;i<=num;i++)
      {
        sum+=i;
        double progress=i/(double)num;
        feedback->progress=progress;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(),"连续反馈中，进度:%.2f",progress);
        //判断是否接收到了取消请求，收到就终止程序return
        // goal_handle->canceled();
        if(goal_handle->is_canceling())//检查这个 goal 有没有收到客户端发来的“取消请求”
        {
          result->sum=sum;
          goal_handle->canceled(result);
          // 告诉 action 框架：这个 goal 已经以“被取消”的状态结束了，并把 result 一起发给客户端。
          RCLCPP_INFO(this->get_logger(),"任务取消！");
          return;

        }
        rate.sleep();//休眠一秒
      }

      //2.生成最终相应结果
      //参数 std::shared_ptr<base_interfaces_demo::action::Progress_Result>
      // goal_handle->succeed();
      if(rclcpp::ok())
      {
        result->sum=sum;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(),"连续反馈中，最终结果:%d",sum);
      }
    }
    // 3-4.生成连续反馈。
    // std::function<void (std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)> 
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle){
      //这是一个耗时操作，所以开启一个线程来主逻辑防止阻塞
      //第 1 个参数：要在线程里执行的“函数”；
      // 后面所有参数：这个函数调用时用到的实参

      std::thread(std::bind(&ProgressActionServer::execute,this,goal_handle)).detach();

    }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProgressActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}