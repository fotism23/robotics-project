#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

enum Joint {
    toe, foot, leg
};

double PI = 3.1415;

bool canMove = true;

tf::Transform getTransform(Joint joint, double angle) {
    tf::Transform transform;
    tf::Quaternion q_rot;

    switch (joint){
        case toe:
            q_rot = tf::createQuaternionFromRPY(0, 0, angle);
            transform.setOrigin( tf::Vector3(0.0, 0.0, 0.03));
            transform.setRotation(q_rot);
            return transform;
        case foot:
            q_rot = tf::createQuaternionFromRPY(angle, 0, 0);
            transform.setOrigin( tf::Vector3(0.0, 0.0, 0.03));
            transform.setRotation(q_rot);
    		return transform;
        case leg:
            q_rot = tf::createQuaternionFromRPY(angle, 0, 0);
            transform.setOrigin( tf::Vector3(0.0, 0.0, 0.4));
            transform.setRotation(q_rot);
            return transform;
    }

	return transform;
}

double* invKinematics(double desiredX, double desiredY, double desiredZ) {
    double* thetas = new double[3];

    // thetas[0] = atan2(desiredY, desiredX) - (M_PI / 2);
    // thetas[2] = acos((pow(desiredZ, 2) + pow(desiredY, 2) + pow(desiredX, 2) - 0.32) / 0.32);
    // thetas[1] = atan2(desiredX, desiredZ) - atan2(0.4 * sin(thetas[2]), 0.4 + 0.4 * cos(thetas[2]));



    thetas[0] = atan2(desiredY, desiredX) + PI / 2;
    double r = sqrt(pow(desiredY, 2) + pow(desiredX, 2));
    double d = sqrt(pow(desiredZ - 0.07, 2) + pow(r, 2));
    thetas[2] = acos((pow(d, 2) - 0.32) / 0.32);
    thetas[1] = atan2(r, desiredZ - 0.07) - atan2(0.4 + 0.4 * cos(thetas[2]), 0.4 * sin(thetas[2]));

    std::cout << thetas[0] << " " << thetas[1] << " " << thetas[2] << '\n';

    //thetas[0] = atan2(desiredY, desiredX);

    return thetas;
}

void moveArmToPos(double x, double y, double z) {
    std::cout << "Move Arm to : x = " << x << ", y = " << y << ", z = " << z << "\n";
    double* thetas = invKinematics(x, y, z);

    tf::Transform transform1 = getTransform(toe, thetas[0]);
    tf::Transform transform2 = getTransform(foot, thetas[1]);
    tf::Transform transform3 = getTransform(leg, thetas[2]);

    tf::Transform transform4;
    transform4.setOrigin( tf::Vector3(0.0, 0.0, 0.4));
    transform4.setRotation( tf::createQuaternionFromRPY(0, 0, 0));

    static tf::TransformBroadcaster broadcaster;

    broadcaster.sendTransform(
        tf::StampedTransform(
            transform1,
            ros::Time::now(), "toe", "foot"));

    broadcaster.sendTransform(
        tf::StampedTransform(
            transform2,
            ros::Time::now(), "foot", "leg"));

    broadcaster.sendTransform(
        tf::StampedTransform(
            transform3,
            ros::Time::now(), "leg", "arm"));

    broadcaster.sendTransform(
        tf::StampedTransform(
            transform4,
            ros::Time::now(), "arm", "finger"));

}

void moveBall(double x, double y, double z) {
    std::cout << "Move Ball to : x = " << x << ", y = " << y << ", z = " << z << "\n";
    static tf::TransformBroadcaster broadcaster_ball;

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x - 0.05, y, z));
    transform.setRotation( tf::createQuaternionFromRPY(0, 0, 0));

    broadcaster_ball.sendTransform(
        tf::StampedTransform(
            transform,
            ros::Time::now(), "world", "ball"));
}

void initArm() {
    moveArmToPos(-0.05, 0, 0.5);
}

void chatterCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    moveBall(msg->x, msg->y, msg->z);

    if (msg->x >= -0.4)
        moveArmToPos(msg->x, msg->y, msg->z);
    else
        initArm();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cmd_vel_pos/ball_tf", 100, chatterCallback);
    ros::spin();
    initArm();
    return 0;
}
