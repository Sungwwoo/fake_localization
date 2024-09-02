#include <memory>
#include <chrono>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>
#include <message_filters/subscriber.h>

using namespace std::chrono_literals;
using namespace std::placeholders;
// Publish map-odom transformations as if there is running amcl node

class FakeLocalization: public rclcpp::Node{
public:
    FakeLocalization(): Node("fake_localization"){
        RCLCPP_INFO(this->get_logger(), "Launching fake_localization");
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface());
        tf_buffer_->setCreateTimerInterface(timer_interface);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        std::chrono::duration<int> buffer_timeout(1);
        // wait for tf_buffer initialization
        rclcpp::sleep_for(2s);


        std::map<std::string, double> double_params ={
            {"delta_x", 0.0},
            {"delta_y", 0.0},
            {"delta_yaw", 0.0},
            {"transform_tolerance", 0.1}
        };
        std::map<std::string, std::string> str_params = {
            {"odom_frame_id", "odom"},
            {"base_frame_id", "base_link"},
            {"global_frame_id", "map"}
        };

        this->declare_parameters("", double_params);
        this->declare_parameters("", str_params);
        delta_x_ = this->get_parameter("delta_x").as_double();
        delta_y_ = this->get_parameter("delta_y").as_double();
        delta_yaw_ = this->get_parameter("delta_yaw").as_double();
        transform_tolerance_ = this->get_parameter("transform_tolerance").as_double();
        odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
        base_frame_id_ = this->get_parameter("base_frame_id").as_string();
        global_frame_id_ = this->get_parameter("global_frame_id").as_string();

        RCLCPP_INFO(this->get_logger(), "Using odom_frame_id: %s", odom_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "Using base_frame_id: %s", base_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "Using global_frame_id: %s", global_frame_id_.c_str());
        particleCloud_.header.stamp = this->get_clock()->now();
        particleCloud_.header.frame_id = global_frame_id_;
        particleCloud_.poses.resize(1);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, -delta_yaw_);
        offsetTF_ = tf2::Transform(q, tf2::Vector3(-delta_x_, -delta_y_, 0.0));
        base_pose_received_ = false;


        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 1);
        pub_particleCloud_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particleCloud",1);

        // ground_truth
        sub_ground_truth_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "base_pose_ground_truth", 100,
            std::bind(&FakeLocalization::cbOdom, this, _1)
        );

        // odometry
        sub_odom_.subscribe(this, ""); // ?
        // message_filter object need to be passed into tf2_ros::MessageFilter object
        filter_odom_ = std::make_shared<tf2_ros::MessageFilter<nav_msgs::msg::Odometry>>(
            sub_odom_, *tf_buffer_, base_frame_id_, 100, this->get_node_logging_interface(), this->get_node_clock_interface()
        );
        filter_odom_->registerCallback(&FakeLocalization::update, this);

        // rviz2 initpose: message filter
        sub_initPose_.subscribe(this, "initpose");
        filter_initPose_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PoseWithCovarianceStamped>>(
            sub_initPose_, *tf_buffer_, global_frame_id_, 1, this->get_node_logging_interface(), this->get_node_clock_interface()
        );
        filter_initPose_->registerCallback(&FakeLocalization::cbInitPose, this);

        RCLCPP_INFO(this->get_logger(), "Successfully launched fake_localizaton");
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_particleCloud_;
    
    // ground_truth
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ground_truth_;
    
    // rviz2 initpose filter
    message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> sub_initPose_;
    std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PoseWithCovarianceStamped>> filter_initPose_;
    
    // odom filter
    message_filters::Subscriber<nav_msgs::msg::Odometry> sub_odom_;
    std::shared_ptr<tf2_ros::MessageFilter<nav_msgs::msg::Odometry>> filter_odom_;

    // tf
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    bool base_pose_received_;
    double delta_x_, delta_y_, delta_yaw_, transform_tolerance_;
    std::string odom_frame_id_, base_frame_id_, global_frame_id_;

    nav_msgs::msg::Odometry basePose_;
    geometry_msgs::msg::PoseArray particleCloud_;
    geometry_msgs::msg::PoseWithCovarianceStamped currentPose_;
    tf2::Transform offsetTF_;

    void update(const nav_msgs::msg::Odometry& message){
        tf2::Transform tf;
        tf2::convert(message.pose.pose, tf);
        tf = offsetTF_ * tf;
        geometry_msgs::msg::TransformStamped odom_to_map;
        try{
            geometry_msgs::msg::TransformStamped inv;
            inv.header.frame_id = base_frame_id_;
            inv.header.stamp = message.header.stamp;
            tf2::convert(tf.inverse(), inv.transform);

            tf_buffer_->transform(inv, odom_to_map, odom_frame_id_);
        }
        catch (tf2::TransformException& ex){
            RCLCPP_ERROR(this->get_logger(), "Failed to transform: %s", ex.what());
        }

        geometry_msgs::msg::TransformStamped trans;

        // Cannot add duration to stamp directly
        // Need to create expanded rclcpp::Time and assign it to stamp
        trans.header.stamp = rclcpp::Time(message.header.stamp) + rclcpp::Duration::from_seconds(transform_tolerance_);
        trans.header.frame_id = global_frame_id_;
        trans.child_frame_id = message.header.frame_id;
        
        tf2::Transform odom_to_map_tf2;
        tf2::convert(odom_to_map.transform, odom_to_map_tf2);
        tf2::Transform odom_to_map_inv = odom_to_map_tf2.inverse();
        tf2::convert(odom_to_map_inv, trans.transform);

        tf_broadcaster_->sendTransform(trans);

        tf2::Transform current;
        tf2::convert(message.pose.pose, current);

        current = offsetTF_ * current;
        geometry_msgs::msg::Transform current_transform;
        tf2::convert(current, current_transform);

        currentPose_.header = message.header;
        currentPose_.header.frame_id = global_frame_id_;
        tf2::convert(current_transform.rotation, currentPose_.pose.pose.orientation);
        currentPose_.pose.pose.position.x = current_transform.translation.x;
        currentPose_.pose.pose.position.y = current_transform.translation.y;        
        currentPose_.pose.pose.position.z = current_transform.translation.z;

        pub_pose_->publish(currentPose_);
        particleCloud_.header = currentPose_.header;
        particleCloud_.poses[0] = currentPose_.pose.pose;
        pub_particleCloud_->publish(particleCloud_);    
    }

    // ???
    void cbOdom(const std::shared_ptr<nav_msgs::msg::Odometry> odom){
        std::shared_ptr<nav_msgs::msg::Odometry> new_odom = std::make_shared<nav_msgs::msg::Odometry>();
        *new_odom = *odom;
        new_odom->header.frame_id = odom_frame_id_;
        filter_odom_->add(new_odom);
    }

    void cbInitPose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg){
        tf2::Transform pose;
        tf2::convert(msg.pose.pose, pose);
        if (msg.header.frame_id != global_frame_id_)
            RCLCPP_WARN(this->get_logger(), "Frame ID of \"initalpose\" (%s) is different from global frame (%s)", msg.header.frame_id.c_str(), global_frame_id_.c_str());
        
        geometry_msgs::msg::TransformStamped base_pose;
        try{
            base_pose = tf_buffer_->lookupTransform(base_frame_id_, global_frame_id_, tf2::TimePointZero);
        }
        catch (tf2::TransformException& ex){
            RCLCPP_WARN(this->get_logger(), "Failed to get transform: %s", ex.what());
            return;
        }

        tf2::Transform base_pose_tf2;
        tf2::convert(base_pose.transform, base_pose_tf2);
        tf2::Transform delta = pose * base_pose_tf2;
        offsetTF_ *= delta;
    }
};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeLocalization>());
    rclcpp::shutdown();
    return 0;
}