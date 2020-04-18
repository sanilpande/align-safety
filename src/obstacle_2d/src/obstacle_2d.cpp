#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

class Chassis{
    public:
        ros::Subscriber lidar_sub;
        ros::Publisher obstacle_pub;

        Chassis(){
            // calculate maximum distance to detect obstacles
            double buffer_time = sense_buffer + comm_buffer + safe_buffer;
            double stopping_distance = (vel * vel * 0.5 / acc) + (buffer_time * vel);

            total_distance = stopping_distance + lidar_distance + safe_distance;
            std::cout << "Calculated distance: " << total_distance << std::endl;

            double angle = atan2(width, 2 * total_distance);
            std::cout << "Calculated angle: " << angle * 180 / 3.14 << std::endl;

            double range = 2.3561899662;
            double start = (range - angle) / 0.0065540750511;

            int s = int(start);
            for (int i = s; i < 720-s; i++){
                range_array[i] = total_distance;
            }
        }

        void obstacle_detection_callback(const sensor_msgs::LaserScan scan)
        {
            // iterating over all values since ideally all element in range should be non zero
            for (int i = 0; i < 720; i++){
                if (range_array[i] > scan.ranges[i]){
                    // std::cout << scan.ranges[i] << std::endl;
                    obstacle_flag = true;
                }
            }
            if (obstacle_flag){
                msg.linear.x = 0;
                obstacle_pub.publish(msg);
                std::cout << "Obstacle detected!\n";
                obstacle_flag = false;
            }

        }

    private:
        bool obstacle_flag;
        double range_array[720] = {0};
        geometry_msgs::Twist msg;

        double width = 1.355;
        double vel = 2;
        double acc = 2; // assuming acc == deceleration

        // distances are in meters
        double total_distance = 5;      // to be overwritten in constructor
        double lidar_distance = 0.1;
        double safe_distance = 0.3;

        // times are in seconds
        double sense_buffer = 1.0 / 40;    // since the lidar samples at 40 Hz
        double comm_buffer = 0.025;
        double safe_buffer = 0.1;
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_2d");
    ros::NodeHandle n;

    Chassis bot;
    bot.lidar_sub = n.subscribe("scan", 1000, &Chassis::obstacle_detection_callback, &bot);
    bot.obstacle_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ros::Rate loop_rate(50);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
