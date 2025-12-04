/*  
  需求：编写动作客户端实现，可以提交一个整型数据到服务端，并处理服务端的连续反馈以及最终返回结果。
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建动作客户端；
      3-2.发送请求；
      3-3.处理目标发送后的反馈；
      3-4.处理连续反馈；
      3-5.处理最终响应。
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。
*/
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"
using namespace std::chrono_literals;
using base_interfaces_demo::action::Progress;
using std::placeholders::_1;
using std::placeholders::_2;


class ProgressActionClient : public rclcpp::Node {
public:
  ProgressActionClient() : Node("progress_action_client_node_cpp") {
    RCLCPP_INFO(this->get_logger(), "action 客户端创建！");
  //     3-1.创建动作客户端；
    client_=rclcpp_action::create_client<Progress>(this,"get_sum");
  }
  //     3-2.发送请求；
   void send_goal(int num){
    //(1)连接服务器
    if(!client_->wait_for_action_server(10s))
    {
      RCLCPP_ERROR(this->get_logger(),"服务器连接失败！");
      return;
    }
    //（2）发送具体请求
    // async_send_goal(const base_interfaces_demo::action::Progress::Goal &goal, 
    //const rclcpp_action::Client<...>::SendGoalOptions &options)
    auto goal=Progress::Goal();
    goal.num=num;
    rclcpp_action::Client<Progress>::SendGoalOptions options;
    options.goal_response_callback=std::bind(&ProgressActionClient::goal_response_callback,this,_1);
    options.feedback_callback=std::bind(&ProgressActionClient::feedback_callback,this,_1,_2);
    options.result_callback=std::bind(&ProgressActionClient::result_callback,this,_1);
    auto future =client_->async_send_goal(goal,options);
    }
  //     3-3.处理目标发送后的反馈
  /*using GoalHandle = ClientGoalHandle<ActionT>;
  using GoalResponseCallback = std::function<void (typename GoalHandle::SharedPtr)>;
  */
    void goal_response_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle){
      //如果不被处理，返回就是一个nullptr
      if(!goal_handle)
      {
        RCLCPP_INFO(this->get_logger(),"目标请求被服务端拒绝！");
      }
      else {
        RCLCPP_INFO(this->get_logger(),"目标处理中...");

      }


    }
  //     3-4.处理连续反馈；
  /*using FeedbackCallback =
    std::function<void (
        typename ClientGoalHandle<ActionT>::SharedPtr,
        const std::shared_ptr<const Feedback>)>;
  */
    void feedback_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle,
                           const std::shared_ptr<const Progress::Feedback> feedback){
          (void)goal_handle;
          double progress=feedback->progress;
          RCLCPP_INFO(this->get_logger(),"当前进度：%d%%",(int)(progress*100));
    }
  //     3-5.处理最终响应。
  //using ResultCallback = std::function<void (const WrappedResult & result)>;
    void result_callback(const rclcpp_action::ClientGoalHandle<Progress>::WrappedResult & result){
      // result.code result中存在一个状态码
      //rclcpp_action::ResultCode表示结果的状态码来进行比较
      if (result.code==rclcpp_action::ResultCode::SUCCEEDED)
      {
          RCLCPP_INFO(this->get_logger(),"最终结果：%ld",result.result->sum);
      }
      else if(result.code==rclcpp_action::ResultCode::ABORTED)
          {
            RCLCPP_INFO(this->get_logger(),"结果被中断！");
          }
      else if(result.code==rclcpp_action::ResultCode::CANCELED)
      {
            RCLCPP_INFO(this->get_logger(),"被取消！");
      }
      else {
            RCLCPP_INFO(this->get_logger(),"未知异常");
      }
        
    }
  
  private:
  rclcpp_action::Client<Progress>::SharedPtr client_;
};

int main(int argc, char ** argv) {
  if(argc!=2)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"请提交一个整形数据！");
    return 1;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProgressActionClient>();
  node->send_goal(atoi(argv[1]));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}