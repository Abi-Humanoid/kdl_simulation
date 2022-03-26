#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"

#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

class abi_legs {
    private:
        // Instance variables to store the current joint values of the legs
        double curr_rhip_y, curr_rhip_r, curr_rhip_p, curr_rknee_p, curr_rankle_p, curr_rankle_r,
            curr_lhip_y, curr_lhip_r, curr_lhip_p, curr_lknee_p, curr_lankle_p, curr_lankle_r;

        // Publishers for joint values
        ros::Publisher pb_ankle_pitch_L,
            pb_ankle_roll_L,
            pb_hip_pitch_L,
            pb_hip_roll_L,
            pb_hip_yaw_L,
            pb_knee_pitch_L,
            pb_ankle_pitch_R,
            pb_ankle_roll_R,
            pb_hip_pitch_R,
            pb_hip_roll_R,
            pb_hip_yaw_R,
            pb_knee_pitch_R;


        // Subscribers for joint values
        ros::Subscriber read_rhip_y,
            read_rhip_r,
            read_rhip_p,
            read_rknee_p,
            read_rankle_p,
            read_rankle_r,
            read_lhip_y,
            read_lhip_r,
            read_lhip_p,
            read_lknee_p,
            read_lankle_p,
            read_lankle_r;


        // KDL::Chains representing Abi's legs
        KDL::Chain r_leg, l_leg;

    void fread_rhip_y(const control_msgs::JointControllerState::ConstPtr &msg) {
        curr_rhip_y = msg->process_value;
    }

    void fread_rhip_r(const control_msgs::JointControllerState::ConstPtr &msg) {
        curr_rhip_r = msg->process_value;
    }

    void fread_rhip_p(const control_msgs::JointControllerState::ConstPtr &msg) {
        curr_rhip_p = msg->process_value;
    }

    void fread_rknee_p(const control_msgs::JointControllerState::ConstPtr &msg) {
        curr_rknee_p = msg->process_value;
    }

    void fread_rankle_p(const control_msgs::JointControllerState::ConstPtr &msg) {
        curr_rankle_p = msg->process_value;
    }

    void fread_rankle_r(const control_msgs::JointControllerState::ConstPtr &msg) {
        curr_rankle_r = msg->process_value;
    }

    void fread_lhip_y(const control_msgs::JointControllerState::ConstPtr &msg) {
        curr_lhip_y = msg->process_value;
    }

    void fread_lhip_r(const control_msgs::JointControllerState::ConstPtr &msg) {
        curr_lhip_r = msg->process_value;
        // std::cout << curr_lhip_r << "\n";
    }

    void fread_lhip_p(const control_msgs::JointControllerState::ConstPtr &msg) {
        curr_lhip_p = msg->process_value;
        // go_down(230);
    }

    void fread_lknee_p(const control_msgs::JointControllerState::ConstPtr &msg) {
        curr_lknee_p = msg->process_value;
    }

    void fread_lankle_p(const control_msgs::JointControllerState::ConstPtr &msg) {
        curr_lankle_p = msg->process_value;
    }

    void fread_lankle_r(const control_msgs::JointControllerState::ConstPtr &msg) {
        curr_lankle_r = msg->process_value;
    }


    public:
        abi_legs(ros::NodeHandle *n, std::string abi_urdf_string, std::string base, std::string l_endf, std::string r_endf) {
            
            // Import Abi as a KDL::Tree via kdl_parser
            KDL::Tree abi_tree;
            if (!kdl_parser::treeFromFile(abi_urdf_string, abi_tree)) {
                ROS_ERROR("Failed to construct kdl tree");
            }
            std::cout << "Number of joints in KDL tree: " << abi_tree.getNrOfJoints() << "\n";

            if (!abi_tree.getChain(base, l_endf, r_leg)) {
                std::cout << "Failed to obtain r_leg kinematics chain\n";
            }
            if (!abi_tree.getChain(base, r_endf, l_leg)) {
                std::cout << "Failed to obtain l_leg kinematics chain\n";
            }
            std::cout << "Number of joints in l_leg: " << l_leg.getNrOfJoints() << "\n";
            std::cout << "Number of joints in r_leg: " << r_leg.getNrOfJoints() << "\n";

            // Advertise publishing nodes
            pb_ankle_pitch_L = n->advertise<std_msgs::Float64>("abi/l_ankle_pitch_joint_position_controller/command", 10);
            pb_ankle_roll_L = n->advertise<std_msgs::Float64>("abi/l_ankle_roll_joint_position_controller/command", 10);
            pb_hip_pitch_L = n->advertise<std_msgs::Float64>("abi/l_hip_pitch_joint_position_controller/command", 10);
            pb_hip_roll_L = n->advertise<std_msgs::Float64>("abi/l_hip_roll_joint_position_controller/command", 10);
            pb_hip_yaw_L = n->advertise<std_msgs::Float64>("abi/l_hip_yaw_joint_position_controller/command", 10);
            pb_knee_pitch_L = n->advertise<std_msgs::Float64>("abi/l_knee_pitch_joint_position_controller/command", 10);
            pb_ankle_pitch_R = n->advertise<std_msgs::Float64>("abi/r_ankle_pitch_joint_position_controller/command", 10);
            pb_ankle_roll_R = n->advertise<std_msgs::Float64>("abi/r_ankle_roll_joint_position_controller/command", 10);
            pb_hip_pitch_R = n->advertise<std_msgs::Float64>("abi/r_hip_pitch_joint_position_controller/command", 10);
            pb_hip_roll_R = n->advertise<std_msgs::Float64>("abi/r_hip_roll_joint_position_controller/command", 10);
            pb_hip_yaw_R = n->advertise<std_msgs::Float64>("abi/r_hip_yaw_joint_position_controller/command", 10);
            pb_knee_pitch_R = n->advertise<std_msgs::Float64>("abi/r_knee_pitch_joint_position_controller/command", 10);

            // Configure subscriber nodes for joint values
            read_rhip_y = n->subscribe("abi/r_hip_yaw_joint_position_controller/state", 1000, &abi_legs::fread_rhip_y, this);
            read_rhip_r = n->subscribe("abi/r_hip_roll_joint_position_controller/state", 1000, &abi_legs::fread_rhip_r, this);
            read_rhip_p = n->subscribe("abi/r_hip_pitch_joint_position_controller/state", 1000, &abi_legs::fread_rhip_p, this);
            read_rknee_p = n->subscribe("abi/r_knee_pitch_joint_position_controller/state", 1000, &abi_legs::fread_rknee_p, this);
            read_rankle_p = n->subscribe("abi/r_ankle_pitch_joint_position_controller/state", 1000, &abi_legs::fread_rankle_p, this);
            read_rankle_r = n->subscribe("abi/r_ankle_roll_joint_position_controller/state", 1000, &abi_legs::fread_rankle_r, this);
            read_lhip_y = n->subscribe("abi/l_hip_yaw_joint_position_controller/state", 1000, &abi_legs::fread_lhip_y, this);
            read_lhip_r = n->subscribe("abi/l_hip_roll_joint_position_controller/state", 1000, &abi_legs::fread_lhip_r, this);
            read_lhip_p = n->subscribe("abi/l_hip_pitch_joint_position_controller/state", 1000, &abi_legs::fread_lhip_p, this);
            read_lknee_p = n->subscribe("abi/l_knee_pitch_joint_position_controller/state", 1000, &abi_legs::fread_lknee_p, this);
            read_lankle_p = n->subscribe("abi/l_ankle_pitch_joint_position_controller/state", 1000, &abi_legs::fread_lankle_p, this);
            read_lankle_r = n->subscribe("abi/l_ankle_roll_joint_position_controller/state", 1000, &abi_legs::fread_lankle_r, this);
        }

        void go_down() {
            std::cout << "Going down!\n";
            std_msgs::Float64 radi;
            // radi.data = 1;
            // pb_ankle_pitch_R.publish(radi);
            // pb_ankle_pitch_L.publish(radi);
            for (int k = 0; k < 800; k++) {
                std_msgs::Float64 radi;
                // Increase knee pitch
                radi.data = curr_rknee_p+0.001;
                curr_rknee_p += 0.001;
                pb_knee_pitch_R.publish(radi);
                radi.data = curr_lknee_p+0.001;
                curr_lknee_p += 0.001;
                pb_knee_pitch_L.publish(radi);
                // Decrease hip pitch
                radi.data = curr_rhip_p-0.001;
                curr_rhip_p -= 0.001;
                pb_hip_pitch_R.publish(radi);
                radi.data = curr_lhip_p-0.001;
                curr_lhip_p -= 0.001;
                pb_hip_pitch_L.publish(radi);
            }
            std::cout << "Went down!";
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "kdl_node");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    // Extract urdf_file from parameters
    std::string abi_urdf_string;
    ros::param::get("abi_urdf_file", abi_urdf_string);

    // Create instance of abi_legs
    abi_legs abi(&nh, abi_urdf_string, "base_link", "l_ankle_roll_link__1__1", "r_ankle_roll_link__1__1");

    rate.sleep();

    // Go down!
    abi.go_down();

    ros::spin();
    return 1;
}