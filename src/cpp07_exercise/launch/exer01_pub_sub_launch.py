from launch import LaunchDescription
from launch_ros.actions import Node
#参数是终端执行命令，可以直接在终端执行
from launch.actions import ExecuteProcess
#注册时间，实现节点的顺序问题
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
def generate_launch_description():
    #1.启动两个turtlesim node,其中一个设置命名空间
    t1=Node(package="turtlesim",executable="turtlesim_node")
    t2=Node(package="turtlesim",executable="turtlesim_node",namespace="t2")


    #2. 控制第二个乌龟掉头
    rotate=ExecuteProcess(
        cmd = ["ros2 action send_goal /t2/turtle1/rotate_absolute \
            turtlesim/action/RotateAbsolute \"{'theta':3.14}\""],
        output="both",
        shell=True
    )
   
    #3.调用自定义节点，并且该节点调用顺序有要求（要在掉头完毕才能执行）
    exer01=Node(package="cpp07_exercise",executable="exer01_pub_sub")
    #掉头需要时间，不能在调头的间隔开始调用自定义节点的操作
    #怎么控制节点的执行顺序，通过注册事件
    #创建事件注册对象，在对象中生命针对那个对象节点，在哪个时间触发，执行哪中操作
    register_rotate_exit_event=RegisterEventHandler(
        event_handler= OnProcessExit(
            target_action=rotate,#目标节点
            on_exit=exer01#触发节点
        )
    )

    return LaunchDescription([t1,t2,rotate,register_rotate_exit_event])
