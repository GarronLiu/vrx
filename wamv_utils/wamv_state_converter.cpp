#include <algorithm>
#include <cmath>
#include <iterator>
#include <string>

#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class WamvStateConverter
{
public:
    WamvStateConverter() : nh_(), private_nh_("~"), tf_broadcaster_()
    {
        private_nh_.param("model_states_topic", model_states_topic_, std::string("/gazebo/model_states"));
        private_nh_.param("odom_topic", odom_topic_, std::string("/wamv/odom"));
        private_nh_.param("model_name", model_name_, std::string("wamv"));
        private_nh_.param("world_frame", world_frame_, std::string("world"));
        private_nh_.param("child_frame", child_frame_, std::string("base_link"));
        private_nh_.param("publish_tf", publish_tf_, true);
        private_nh_.param("twist_in_world", twist_in_world_, true);

        model_states_sub_ = nh_.subscribe(model_states_topic_, 10, &WamvStateConverter::modelStatesCallback, this,
                                          ros::TransportHints().tcpNoDelay());
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);

        ROS_INFO_STREAM("WAM-V state converter started. ModelStates: "
                        << model_states_topic_ << ", model: " << model_name_ << ", odometry: " << odom_topic_
                        << ", frame: " << world_frame_ << " -> " << child_frame_);
    }

private:
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        if (!msg)
        {
            return;
        }

        const auto model_it = std::find(msg->name.begin(), msg->name.end(), model_name_);
        if (model_it == msg->name.end())
        {
            ROS_WARN_THROTTLE(5.0, "Model '%s' was not found in %s", model_name_.c_str(), model_states_topic_.c_str());
            return;
        }

        const std::size_t index = static_cast<std::size_t>(std::distance(msg->name.begin(), model_it));
        if (index >= msg->pose.size())
        {
            ROS_WARN_THROTTLE(5.0, "ModelStates contains no pose for model '%s'", model_name_.c_str());
            return;
        }

        nav_msgs::Odometry odom;
        // gazebo_msgs/ModelStates has no std_msgs/Header. ros::Time::now() follows
        // /use_sim_time, so this is the appropriate timestamp for Gazebo playback.
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = world_frame_.empty() ? std::string("world") : world_frame_;
        odom.child_frame_id = child_frame_;
        odom.pose.pose = msg->pose[index];

        tf::Quaternion orientation(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                                   odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
        const double orientation_length = orientation.length();
        if (!std::isfinite(orientation_length) || orientation_length < 1e-9)
        {
            ROS_WARN_THROTTLE(5.0, "Ignoring model '%s' with an invalid orientation", model_name_.c_str());
            return;
        }
        orientation.normalize();
        odom.pose.pose.orientation.x = orientation.x();
        odom.pose.pose.orientation.y = orientation.y();
        odom.pose.pose.orientation.z = orientation.z();
        odom.pose.pose.orientation.w = orientation.w();

        if (index < msg->twist.size())
        {
            odom.twist.twist = msg->twist[index];
            if (twist_in_world_)
            {
                // Gazebo reports ModelStates velocity in the world frame. Odometry.twist
                // is expressed in child_frame_id, so rotate both linear and angular parts.
                const tf::Matrix3x3 world_to_body = tf::Matrix3x3(orientation).transpose();
                const tf::Vector3 linear_world(odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                                               odom.twist.twist.linear.z);
                const tf::Vector3 angular_world(odom.twist.twist.angular.x, odom.twist.twist.angular.y,
                                                odom.twist.twist.angular.z);
                const tf::Vector3 linear_body = world_to_body * linear_world;
                const tf::Vector3 angular_body = world_to_body * angular_world;

                odom.twist.twist.linear.x = linear_body.x();
                odom.twist.twist.linear.y = linear_body.y();
                odom.twist.twist.linear.z = linear_body.z();
                odom.twist.twist.angular.x = angular_body.x();
                odom.twist.twist.angular.y = angular_body.y();
                odom.twist.twist.angular.z = angular_body.z();
            }
        }
        else
        {
            ROS_WARN_THROTTLE(5.0, "ModelStates contains no twist for model '%s'", model_name_.c_str());
        }

        odom_pub_.publish(odom);

        if (publish_tf_)
        {
            // ModelStates has no timestamp. During a single Gazebo simulation
            // step, ros::Time::now() can therefore be repeated. Publishing a
            // dynamic TF twice with the same stamp triggers TF_REPEATED_DATA.
            if (has_last_tf_stamp_ && odom.header.stamp < last_tf_stamp_)
            {
                // Allow a Gazebo time reset to start a new TF sequence.
                has_last_tf_stamp_ = false;
            }
            if (!has_last_tf_stamp_ || odom.header.stamp > last_tf_stamp_)
            {
                tf::Transform transform;
                transform.setOrigin(
                    tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
                transform.setRotation(orientation);
                tf_broadcaster_.sendTransform(
                    tf::StampedTransform(transform, odom.header.stamp, odom.header.frame_id, odom.child_frame_id));
                last_tf_stamp_ = odom.header.stamp;
                has_last_tf_stamp_ = true;
            }
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber model_states_sub_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster tf_broadcaster_;

    std::string model_states_topic_;
    std::string odom_topic_;
    std::string model_name_;
    std::string world_frame_;
    std::string child_frame_;
    bool publish_tf_ = true;
    bool twist_in_world_ = true;
    bool has_last_tf_stamp_ = false;
    ros::Time last_tf_stamp_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wamv_state_converter");
    WamvStateConverter converter;
    ros::spin();
    return 0;
}
