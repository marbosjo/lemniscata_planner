/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <dynamic_reconfigure/server.h>
#include <lemniscata_planner/LemniscataConfig.h>

using std::string;

#ifndef LEMNISCATA_PLANNER_CPP
#define LEMNISCATA_PLANNER_CPP

namespace lemniscata_planner {

    class LemniscataPlanner : public nav_core::BaseGlobalPlanner {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;

            ros::Publisher plan_pub_;
            ros::Publisher plan_poses_pub_;
            ros::Publisher lemniscata_pub_;
            bool initialized_;

            std::vector<geometry_msgs::PoseStamped> current_lemniscata_;
            geometry_msgs::PoseStamped current_goal_;
            size_t current_subgoal_index_;

            dynamic_reconfigure::Server<lemniscata_planner::LemniscataConfig> *dyn_rec_;
            dynamic_reconfigure::Server<lemniscata_planner::LemniscataConfig>::CallbackType dyn_rec_callback_;
            
            void reconfigureCallback(LemniscataConfig& config, uint32_t level);

            double alpha_;
            double beta_;
            size_t sampling_points_;
            size_t path_size_;
            bool keep_goal_orientation_;

            bool must_regenerate_lemniscata_;

        public:

            LemniscataPlanner();
            LemniscataPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            ~LemniscataPlanner();
            /** overridden classes from interface nav_core::BaseGlobalPlanner **/
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            void generateLemniscata(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& lemniscata);

            bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan
                    );
            void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    };
};
#endif
