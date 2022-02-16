#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

class abi_tree {
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
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "kdl_node");
    ros::NodeHandle nh;
    ros::Rate rate(100);

    // Import Abi via kdl_parser
    KDL::Tree abi_tree;
    std::string abi_urdf_string;
    std::cout << "FAIL";
    if (!kdl_parser::treeFromFile("/home/craig/tim_abi_ws/src/abi_description/abi.urdf", abi_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
    }

   ros::spin();
   return 1;
}