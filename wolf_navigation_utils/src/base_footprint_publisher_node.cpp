#include <Eigen/Core>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>


class BasefootprintHeightEstimator
{
private:
    
public:
    BasefootprintHeightEstimator();
};

BasefootprintHeightEstimator::BasefootprintHeightEstimator()
{
}


class ImuSubscriber
{
private:
    sensor_msgs::Imu imu_m;
    ros::Time prev_t_m;

public:
    ImuSubscriber();
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
    sensor_msgs::Imu getImuData();
};


ImuSubscriber::ImuSubscriber()
{
    prev_t_m = ros::Time::now();
}

sensor_msgs::Imu ImuSubscriber::getImuData()
{
    return imu_m;
}

void ImuSubscriber::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
    if(prev_t_m != imu->header.stamp)
    {
        imu_m = *imu;
        prev_t_m = imu->header.stamp;
    }
}

// TODO: rimuovi tutte le magic variable e usa yaml files per parametri
int main(int argc, char *argv[])
{

    ros::init(argc, argv, "base_footprint_publisher_node");
    ros::NodeHandle n;

    ImuSubscriber imu_sub;
    ros::Subscriber imu_sub = n.subscribe("/T265_camera/gyro/sample", 100, &ImuSubscriber::imuCallback, &imu_sub);

    // variable initialization
    double estimated_z;
    Eigen::Matrix3d tmp_R;
    Eigen::Quaterniond tmp_q;

    ros::Time t_prev;
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped basefoot_T_base;

    basefoot_T_base.header.frame_id = "base_footprint";
    basefoot_T_base.child_frame_id  = "base_link";

    ros::Rate publishing_rate(250);

    while (ros::ok())
    {
        ros::Time t = ros::Time::now();

        if(t != t_prev) // Avoid publishing duplicated transforms
        {
        // TODO: READ BASELINK from odom 

        // TODO: COMPUTE ESTIMATED Z FROM CONTACT POINTS OF WHEEL

        // TODO: create BASE TRANSFORM
        // Set coordinates
        basefoot_T_base.transform.translation.x = 0.0;
        basefoot_T_base.transform.translation.y = 0.0;
        basefoot_T_base.transform.translation.z = estimated_z;
        basefoot_T_base.transform.rotation.w    = tmp_q.w();
        basefoot_T_base.transform.rotation.x    = tmp_q.x();
        basefoot_T_base.transform.rotation.y    = tmp_q.y();
        basefoot_T_base.transform.rotation.z    = tmp_q.z();
        // Set transform header
        basefoot_T_base.header.seq++;
        basefoot_T_base.header.stamp = t;

        br.sendTransform(basefoot_T_base);

        }

        publishing_rate.sleep();
    }

    return 0;
}

