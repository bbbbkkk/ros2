/*  
  需求：编写静态坐标变换程序，执行时传入两个坐标系的相对位姿关系以及父子级坐标系id，
       程序运行发布静态坐标变换。
       ros2 run 包 可执行程序 x y z roll pitch yaw frame child_frame
  步骤：
    1.包含头文件；
    2.判断终端传入的参数是否合法；
          argc=9?
    3.初始化 ROS 客户端；
    4.定义节点类；
      4-1.创建静态坐标变换发布方；
      4-2.组织并发布消息。
    5.调用 spin 函数，并传入对象指针；
    6.释放资源。

*/
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform.hpp"
  //将欧拉角转换为四元数需要包含的头文件
#include "tf2/LinearMath/Quaternion.h"

class TFStaticBroadcaster : public rclcpp::Node {
public:
  TFStaticBroadcaster(char*argv[]) : Node("tf_static_broadcaster_node") {
    RCLCPP_INFO(this->get_logger(), "静态广播器创建！");
    /*StaticTransformBroadcaster<NodeT, AllocatorT>(NodeT &&node, 
    const rclcpp::QoS &qos = tf2_ros::StaticBroadcasterQoS(), 
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> &options = [](void){...}())*/
    broadcaster_=std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    pub_static_tf(argv);
  }
  private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  void pub_static_tf(char*argv[])
  {
    geometry_msgs::msg::TransformStamped tranform;
    tranform.header.stamp=this->now();
    // frame父级坐标系
    tranform.header.frame_id=argv[7];
    //子级坐标系
    // tranform.child_frame_id=argv[8];
    tranform.child_frame_id=argv[8];
    //偏移量
    tranform.transform.translation.x=atof(argv[1]);
    tranform.transform.translation.y=atof(argv[2]);
    tranform.transform.translation.z=atof(argv[3]);
    //设置四元数
    //将欧拉角转换为四元数
    tf2::Quaternion qtn;
    qtn.setRPY(atof(argv[4]),atof(argv[5]),atof(argv[6]));
    tranform.transform.rotation.x=qtn.getX();
    tranform.transform.rotation.y=qtn.getY();
    tranform.transform.rotation.z=qtn.getZ();
    //发布出去
    broadcaster_->sendTransform(tranform);
  }
};

int main(int argc, char ** argv) {
  if(argc!=9)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"传入的参数不合法！");
    return 1;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFStaticBroadcaster>(argv);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
