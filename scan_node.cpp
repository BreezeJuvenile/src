#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <termios.h> 
#include <unistd.h>  

class LidarProcessor : public rclcpp::Node
{
private:
    int biaozhi = 0;
    int dajiao;
    int ch = 0;
    int speed = 1530; // 基础速度
    int ztjs_L = 0;
    int oldjiaodu_L = 90; //角度中值

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

    geometry_msgs::msg::Pose2D point;
    geometry_msgs::msg::Twist cmd_vel;


    void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    

public:
    LidarProcessor();
    ~LidarProcessor();
};

LidarProcessor::LidarProcessor() : Node("lidar_processor")
{
    RCLCPP_INFO(this->get_logger(), "雷达数据分析节点开启");
    cmd_vel.angular.z = 90;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.linear.x = speed;
    sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 100, std::bind(&LidarProcessor::LidarCallback, this, std::placeholders::_1));
    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("car_cmd_vel", 100);
}

LidarProcessor::~LidarProcessor()
{
    RCLCPP_INFO(this->get_logger(), "雷达数据分析节点关闭");
}

void LidarProcessor::LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Lidar 回调函数处理逻辑
    float zuixiao_L = 80.0;
    float zuixiao_R = 80.0;
    int zhaodao_L = 0;
    int zhaodao_R = 0;
    int flag = 0;
    int jiaodu_L;
    int jiaodu_R;
    float Pi = 3.1415926;
    float sina;
    float dajiao;
    for (int i = 0; i < 1000; i++)
    {
        if (i > 0 && i < 208)  // 雷达检测角度范围限幅
        {
            float fMidDist = msg->ranges[i] * 100.0;
            if (fMidDist < 80 && fMidDist < zuixiao_L)
            {
                zuixiao_L = fMidDist;
                zhaodao_L = 1;
                jiaodu_L = i;
            }
        }
        if (i > 791)
        {
            float fMidDist = msg->ranges[i] * 100.0;
            if (fMidDist < 80 && fMidDist < zuixiao_R)
            {
                zuixiao_R = fMidDist;
                zhaodao_R = 1;
                jiaodu_R = i;
                flag = 0;
            }
        }
    }
    if (zhaodao_L)
    {
        sina = sin(jiaodu_L * 0.36 * Pi / 180);
        dajiao = (zuixiao_L * sina);
        if (dajiao < -50) { dajiao = -50; } // 舵机打角限幅
        if (dajiao > 50) { dajiao = 50; }
        if ((oldjiaodu_L - jiaodu_L) > 25)
        {
            ztjs_L = ztjs_L + 1;
        }
        oldjiaodu_L = jiaodu_L;
        flag = 1;
        RCLCPP_INFO(this->get_logger(), "左角度：%.1f 左距离：%.1f 打角：%d 速度：%d ztjs:  %d", jiaodu_L * 0.36, zuixiao_L, static_cast<int>(dajiao), speed, ztjs_L);
    }
    if (ztjs_L > 60)
            ch = 3;
    if (ch != 3 && zhaodao_L)
    {
        cmd_vel.linear.x = speed;
        cmd_vel.angular.z = dajiao - 50 + 80;
        vel_pub->publish(cmd_vel);
        flag = 1;
        RCLCPP_INFO(this->get_logger(), "正在前进");
    }
    else
    {
        cmd_vel.linear.x = 1500;
        cmd_vel.angular.z =  80;
        vel_pub->publish(cmd_vel);
        RCLCPP_INFO(this->get_logger(), "zhengzaitingche");
    }
    if (zhaodao_R && flag == 0 )
    {
        // 动态角速度调整，基准为 90，＞90 左转，＜90 右转
        float max_angular_z = 130;  // 最大左转角速度，避免过大
        float min_angular_z = 90;   // 最小角速度为 90，表示直行

        float max_distance = 80.0;  // 设置一个最大距离阈值
        float min_distance = 10.0;  // 最小距离阈值（太近时需要急转弯）

        // 动态计算角速度：根据右侧最小距离进行插值计算，距离越近，角速度越大
        float angular_z = max_angular_z - ((zuixiao_R - min_distance) / (max_distance - min_distance)) * (max_angular_z - min_angular_z);
        
        // 限制角速度在合理范围内
        if (angular_z > max_angular_z) angular_z = max_angular_z;
        if (angular_z < min_angular_z) angular_z = min_angular_z;

        // 动态线速度调整，基准为 1500 静止，1530 为正常前进速度
        float max_linear_x = 1530;   // 最大线速度
        float min_linear_x = 1500;   // 最小线速度，1500 表示静止

        // 动态计算线速度：障碍物越近，线速度越低
        float linear_x = max_linear_x - ((zuixiao_R - min_distance) / (max_distance - min_distance)) * (max_linear_x - min_linear_x);
        
        // 限制线速度在合理范围内，不能小于1500
        if (linear_x > max_linear_x) linear_x = max_linear_x;
        if (linear_x < min_linear_x) linear_x = min_linear_x;

        // 设置计算出的动态速度值
        cmd_vel.angular.z = angular_z;
        cmd_vel.linear.x = linear_x;

        vel_pub->publish(cmd_vel);

        // 打印调试信息，显示动态角速度和线速度
        RCLCPP_INFO(this->get_logger(), "右角度：%.1f  右距离：%.1f 动态角速度：%.1f 动态线速度：%.1f", 
                    jiaodu_R * 0.36, zuixiao_R, angular_z, linear_x);
    }

    
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessor>());
    rclcpp::shutdown();
    return 0;
}