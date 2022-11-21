#include <string>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

//ros utils
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <riptide_msgs2/srv/get_robot_xacro.hpp>

//gazebo untils
#include <gz/msgs.hh>
#include <gz/transport.hh>

using namespace std::placeholders;

#define ROBOT_NAME "tempest"

namespace uwrt_ros_gz{
    // the bridge between gazebo and ros
    class BridgeNode : public rclcpp::Node{
        
        //topics to publish to gazebo to 
        private: std::string gz_pub_topic_position = "/bridge/tempest/position";
        private: std::string gz_pub_topic_orentation = "/bridge/tempest/orientation";

        //topics to subcribe to ros from
        private: std::string ros_sub_topic_pose = "/tempest/simulator/pose";
        private: std::string ros_cli_service_xacro = "/tempest/load_xacro";

        //topics to subscribe to ros from (for tempest ghost)
        private: std::string ghost_ros_topic_pose = "/tempest/odometry/filtered";

        //topics to publish to gazebo to (for tempest ghost)
        private: std::string ghost_gz_pub_topic_position = "/bridge/ghost/position";
        private: std::string ghost_gz_pub_topic_orientation = "/bridge/ghost/orientation";

        //xacro data
        private: std::string robot_xacro = "no data";

        //gazebo node & accessories
        private: gz::transport::Node gz_node;
        private: gz::transport::Node::Publisher gz_tempest_position_publisher;
        private: gz::transport::Node::Publisher gz_tempest_orentation_publisher;

        //publishers for ghost
        private: gz::transport::Node::Publisher gz_tempest_ghost_position_publisher;
        private: gz::transport::Node::Publisher gz_tempest_ghost_orentation_publisher;
    
        //ros2 subscribers
        private: std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> ros_pose_subscriber;

        //ros2 ghost subscriber
        private: private: std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> ros_ghost_pose_subscriber;


        public: BridgeNode() : Node("bridge_node"){
            RCLCPP_INFO(this->get_logger(), "Creating ros2 gz bridge.");

            auto cb_tempest_pose = std::bind(&BridgeNode::tempestPoseCallback, this, std::placeholders::_1);
            ros_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(ros_sub_topic_pose, 10, cb_tempest_pose);

            //ghost
            auto cb_tempest_ghost_pose = std::bind(&BridgeNode::tempestGhostPoseCallback, this, std::placeholders::_1);
            ros_ghost_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(ghost_ros_topic_pose, 10, cb_tempest_ghost_pose);

            gz_tempest_position_publisher = gz_node.Advertise<gz::msgs::Vector3d>(gz_pub_topic_position);
            gz_tempest_orentation_publisher = gz_node.Advertise<gz::msgs::Quaternion>(gz_pub_topic_orentation);

            //ghost publishers
            gz_tempest_ghost_position_publisher = gz_node.Advertise<gz::msgs::Vector3d>(ghost_gz_pub_topic_position);
            gz_tempest_ghost_orentation_publisher = gz_node.Advertise<gz::msgs::Quaternion>(ghost_gz_pub_topic_orientation); 

            sleep(1);

            RCLCPP_INFO(this->get_logger(), "Ros-GZ bridge is ready. Listening for updates!");
        }

        public: void tempestPoseCallback(const geometry_msgs::msg::PoseStamped &_msg){
            //break down the msg
            geometry_msgs::msg::Pose pose = _msg.pose;
            geometry_msgs::msg::Point position = pose.position;
            geometry_msgs::msg::Quaternion orientation = pose.orientation;

            //send to gazebo
            publishToGazeboTempestPostion(position.x, position.y, position.z);
            publishToGazeboTempestOrentation(orientation.x, orientation.y, orientation.z, orientation.w);
        }

        public: void tempestGhostPoseCallback(const geometry_msgs::msg::PoseStamped &_msg){
            //break down the msg
            geometry_msgs::msg::Pose pose = _msg.pose;
            geometry_msgs::msg::Point position = pose.position;
            geometry_msgs::msg::Quaternion orientation = pose.orientation;

            //send to gazebo
            publishToGazeboTempestGhostPosition(position.x, position.y, position.z);
            publishToGazeboTempestGhostOrientation(orientation.x, orientation.y, orientation.z, orientation.w);
        }

        public: void publishToGazeboTempestPostion(float x, float y, float z){
            // //construct pose msg
            gz::msgs::Vector3d msg;
            msg.set_x(x);
            msg.set_y(y);
            msg.set_z(z);

            //publish
            gz_tempest_position_publisher.Publish(msg);

            //give it a sec -- REQUIRED
            sleep(.002);
        }

        public: void publishToGazeboTempestOrentation(float x, float y, float z, float w){
            gz::msgs::Quaternion msg;
            msg.set_w(w);
            msg.set_x(x);
            msg.set_y(y);
            msg.set_z(z);

            //publish
            gz_tempest_orentation_publisher.Publish(msg);

            //give it a sec -- REQUIRED
            sleep(.002);
        }

        //ghost publish functions

        public: void publishToGazeboTempestGhostPosition(float x, float y, float z){
            // contruct ghost post msg
            gz::msgs::Vector3d msg;
            msg.set_x(x);
            msg.set_y(y);
            msg.set_z(z);

            //publish
            gz_tempest_ghost_position_publisher.Publish(msg);

            //give it a sec -- REQUIRED
            sleep(0.002);
        }

        public: void publishToGazeboTempestGhostOrientation(float x, float y, float z, float w){
            //construct ghost orentation message
            gz::msgs::Quaternion msg;
            msg.set_w(w);
            msg.set_x(x);
            msg.set_y(y);
            msg.set_z(z);

            //Publish
            gz_tempest_ghost_orentation_publisher.Publish(msg);

            //give it a sec -- REQUIRED
            sleep(0.002);
        }

    };
}

//start the node
int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<uwrt_ros_gz::BridgeNode>());
    rclcpp::shutdown();

    return 0;
}