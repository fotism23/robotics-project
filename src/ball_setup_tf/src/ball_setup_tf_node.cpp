#include <ros/ros.h>
#include "geometry_msgs/Vector3.h"

geometry_msgs::Vector3 getVectorMessage(double currentTime){
    geometry_msgs::Vector3 vector;

    int dummyY = 1000 * cos(currentTime);
    int dummyZ = 1000 * sin(currentTime);




    vector.x = currentTime;
    // vector.y = (double) dummyY / (double) 1000.0;
    // vector.z = (double) dummyZ / (double) 1000.0;
    vector.y = 0.3;
    vector.z = 0.5;

    return vector;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ball_node");
    ros::NodeHandle n("~");
    ros::Publisher ball_tf = n.advertise<geometry_msgs::Vector3>("/cmd_vel_pos/ball_tf", 100);

    ros::Rate loop_rate(1);

    double currentTime = -1;
    double timeStep = 0.05;

    while (ros::ok()) {
        geometry_msgs::Vector3 msg = getVectorMessage(currentTime);
        ball_tf.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        currentTime += timeStep;

        if (currentTime >= -0.25)
            currentTime = -1;
    }
}
