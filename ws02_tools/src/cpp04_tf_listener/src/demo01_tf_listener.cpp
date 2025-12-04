/*  
  需求：订阅 laser 到 base_link 以及 camera 到 base_link 的坐标系关系，
       并生成 laser 到 camera 的坐标变换。
  步骤：
    1.包含头文件；
    2.初始化 ROS 客户端；
    3.定义节点类；
      3-1.创建tf缓存对象指针；
      3-2.创建tf监听器；
      3-3.按照条件查找符合条件的坐标系并生成变换后的坐标帧。
    4.调用 spin 函数，并传入对象指针；
    5.释放资源。

*/

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
using namespace std::chrono_literals;

class TFListener : public rclcpp::Node {
public:
  TFListener() : Node("tf_listener_node") {
    RCLCPP_INFO(this->get_logger(), "监听创建");
    buffer_=std::make_unique<tf2_ros::Buffer>(this->get_clock());
    listener_=std::make_shared<tf2_ros::TransformListener>(*buffer_,this);
    timer_=this->create_wall_timer(1s,std::bind(&TFListener::on_timer,this));
  }
  private:
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  void on_timer()
  {
    //3-3.按照条件查找符合条件的坐标系并生成变换后的坐标帧。
    /*1/6
    geometry_msgs::msg::TransformStamped lookupTransform
    (const std::string &target_frame, 父级
    const std::string &source_frame, 子级
    const tf2::TimePoint &time)*/
    try
    {
      auto ts=buffer_->lookupTransform("camera","laser",tf2::TimePointZero);
      RCLCPP_INFO(this->get_logger(),"----------转换完成的坐标帧信息-----------");
      RCLCPP_INFO(this->get_logger(),
      "新坐标帧：父坐标系：%s,子坐标系：%s,偏移量（%.2f,%.2f,%.2f）",
      ts.header.frame_id.c_str(),
      ts.child_frame_id.c_str(),
      ts.transform.translation.x,
      ts.transform.translation.y,
      ts.transform.translation.z);

    }
    catch(const tf2::LookupException& e)
    {
      RCLCPP_INFO(this->get_logger(),"异常提示：%s",e.what());
    }
    
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}