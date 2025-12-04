/*  
  需求：将雷达感知到的障碍物的坐标点数据（相对于 laser 坐标系），
       转换成相对于底盘坐标系（base_link）的坐标点。

  步骤：
    1.包含头文件；
    2.初始化 ROS 客户端；
    3.定义节点类；
      3-1.创建tf缓存对象指针；
      3-2.创建tf监听器；
      3-3.创建坐标点订阅方并订阅指定话题；
      3-4.创建消息过滤器过滤被处理的数据；
      3-5.为消息过滤器注册转换坐标点数据的回调函数。
    4.调用 spin 函数，并传入对象指针；
    5.释放资源。

*/
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_interface.h"
#include "tf2_ros/create_timer_ros.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
using namespace std::chrono_literals;
class TFPointListener : public rclcpp::Node {
public:
  TFPointListener() : Node("tf_point_listener_node") {
    RCLCPP_INFO(this->get_logger(), "坐标变换实现");
    buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
    timer_=std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()
    );
    buffer_->setCreateTimerInterface(timer_);
    listener_=std::make_shared<tf2_ros::TransformListener>(*buffer_);
    point_sub.subscribe(this,"point");
    /*
        F & f,                                                        订阅对象
        BufferT & buffer,                                             坐标监听缓存
        const std::string & target_frame,                             base_link
        uint32_t queue_size,                                          10
        const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logging,
        const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & node_clock,
        std::chrono::duration<TimeRepT, TimeT> buffer_timeout =
        std::chrono::duration<TimeRepT, TimeT>::max())
    */
    filter_=std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>>(
        point_sub,
        *buffer_,
        "base_link",
        10,
        this->get_node_logging_interface(),
        this->get_node_clock_interface(),
        1s
    );
    //解析数据
    filter_->registerCallback(&TFPointListener::tranform_point,this);
    
}
  private:
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::shared_ptr<tf2_ros::CreateTimerInterface> timer_;
  message_filters::Subscriber<geometry_msgs::msg::PointStamped> point_sub;
  std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>> filter_;
  void tranform_point(const geometry_msgs::msg::PointStamped & ps)
  {
    //坐标点变换
    //必须包含头文件
    auto out=buffer_->transform(ps,"base_link");
    RCLCPP_INFO(this->get_logger(),"父级坐标系：%s,坐标：（%.2f,%.2f,%.2f）",
    out.header.frame_id.c_str(),
    out.point.x,
    out.point.y,
    out.point.z
  );
  }
};  

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFPointListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}