/*
    需求：编写参数服务端，设置并操作参数。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.声明参数(增)；
            3-2.查询参数；
            3-3.修改参数；
            3-4.删除参数。
        4.创建节点对象指针，调用参数操作函数，并传递给spin函数；
        5.释放资源。

*/
#include "rclcpp/rclcpp.hpp"
// 1.包含头文件；
//这里允许删除参数，那么2就需要通过NodeOptions声明
class ParamServer : public rclcpp::Node {
public:

  ParamServer() : Node("param_server_node_cpp",
    rclcpp::NodeOptions().allow_undeclared_parameters(true))
    {
        RCLCPP_INFO(this->get_logger(), "参数服务端创建 ！");
    }
    // 3-1.声明参数(增)；
        void declare_param()
        {
            RCLCPP_INFO(this->get_logger(),"----------增----------");
            this->declare_parameter("car_type","BMW328");
            this->declare_parameter("price",16000);
            this->declare_parameter("car_max_speed",111);
        }
        //     3-2.查询参数；
        void get_param()
        {
            RCLCPP_INFO(this->get_logger(),"----------查----------");
            // this->get_parameter
            auto car=this->get_parameter("car_type");
            RCLCPP_INFO(this->get_logger(),"car_type:%s",car.value_to_string().c_str());
            // this->get_parameters 可以返回几个键值对应的value
            auto params=this->get_parameters({"car_type","price","car_max_speed"});
            for(auto &&param:params)
            {
                RCLCPP_INFO(this->get_logger(),"(%s=%s)",param.get_name().c_str(),param.value_to_string().c_str());
            }
            // // this->has_parameter判断是否包含
                RCLCPP_INFO(this->get_logger(),"是否包含car_name %d",this->has_parameter("car_name")); 
        }
        //     3-3.修改参数；
        void update_param()
        {
            RCLCPP_INFO(this->get_logger(),"----------增----------");
            // this->set_parameter（rclcpp::Parameter(k,v)） 参数是k和v
            //可以设计新的键值对,前提是在定义类时的第二个参数
            // rclcpp::NodeOptions().allow_undeclared_parameters(true)
            this->set_parameter(rclcpp::Parameter("car_max_speed",120));
            RCLCPP_INFO(this->get_logger(),"car_max_speed:%ld",this->get_parameter("car_max_speed").as_int());
            this->set_parameter(rclcpp::Parameter("car_fix","shit"));
            RCLCPP_INFO(this->get_logger(),"car_fix:%s",this->get_parameter("car_fix").as_string().c_str());

        }
        //     3-4.删除参数。
        void delete_param()
        {
            RCLCPP_INFO(this->get_logger(),"----------增----------");
            this->undeclare_parameter("car_fix");//参数是k
            //不能删除声明的参数，这里就是price
            //可以删除未声明后面自己设置的参数
        }
    };

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParamServer>();
  node->declare_param();
  node->get_param();
  node->update_param();
  node->delete_param();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}