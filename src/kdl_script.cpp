#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

class abi_legs {
    private:
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
            pb_knee_pitch_R,
            pb_com,
            pb_centroid,
            pb_com_x,
            pb_centroid_x,
            pb_com_y,
            pb_centroid_y,
            pb_l_foot,
            pb_r_foot;

        // KDL::Chains representing Abi's legs
        KDL::Chain r_leg, l_leg;

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
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "kdl_node");
    ros::NodeHandle nh;
    ros::Rate rate(100);

    // Extract urdf_file from parameters
    std::string abi_urdf_string;
    ros::param::get("abi_urdf_file", abi_urdf_string);

    // Create instance of abi_legs
    abi_legs abi(&nh, abi_urdf_string, "base_link", "l_ankle_roll_link__1__1", "r_ankle_roll_link__1__1");

    ros::spin();
    return 1;
}