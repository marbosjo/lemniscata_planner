#include <pluginlib/class_list_macros.h>
#include <lemniscata_planner/lemniscata_planner.h>
#include <tf/transform_datatypes.h>

//register this planner as a BaseGlobalPlanner plugin

using namespace std;

//Default Constructor
namespace lemniscata_planner {

    LemniscataPlanner::LemniscataPlanner () : nh_(""), private_nh_("~"), initialized_(false){

    }

    LemniscataPlanner::LemniscataPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : nh_(""), private_nh_("~"), initialized_(false) {
        initialize(name, costmap_ros);
    }


    void LemniscataPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        plan_pub_ = private_nh_.advertise<nav_msgs::Path>("plan", 1);
        plan_poses_pub_ = private_nh_.advertise<geometry_msgs::PoseArray>("plan_poses", 1);
        initialized_ = true;
    }

    bool LemniscataPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan )
    {
        double alpha = 3;
        int path_points = 20;
        int remove_n_points = 5;


        double initial_yaw = tf::getYaw(goal.pose.orientation);

        plan.resize(path_points-remove_n_points);
        for (size_t i = 0; i < path_points-remove_n_points; i++) {
            double t = i*2.0*M_PI/(path_points-1);
           
            double lemniscata_x = alpha * std::sqrt(2.0) * std::cos(t) / (std::pow(std::sin(t), 2.0) + 1.0);
            double lemniscata_y = alpha * std::sqrt(2.0) * std::cos(t) * std::sin(t) / (std::pow(std::sin(t), 2.0) + 1);
            
            
            plan[i].header.frame_id = goal.header.frame_id; 
//            plan[i].pose.position.x = goal.pose.position.x + alpha * std::sqrt(2.0) * std::cos(t) / (std::pow(std::sin(t), 2.0) + 1.0);
//            plan[i].pose.position.y = goal.pose.position.y + alpha * std::sqrt(2.0) * std::cos(t) * std::sin(t) / (std::pow(std::sin(t), 2.0) + 1);
            plan[i].pose.position.x = goal.pose.position.x + std::cos(initial_yaw) * lemniscata_x - std::sin(initial_yaw) * lemniscata_y;
            plan[i].pose.position.y = goal.pose.position.y + std::sin(initial_yaw) * lemniscata_x + std::cos(initial_yaw) * lemniscata_y;

        }

        for (size_t i = 0; i < path_points; i++) {
            size_t next = i + 1;
            if (next == path_points)
                next = 0;

            double yaw = std::atan2(plan[next].pose.position.y - plan[i].pose.position.y, plan[next].pose.position.x - plan[i].pose.position.x);
            plan[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }

        publishPlan(plan);
        return true;
    }
    void LemniscataPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
    {
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        //create a message for the plan 
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        geometry_msgs::PoseArray poses_path;
        poses_path.poses.resize(path.size());

        if(!path.empty())
        {
            gui_path.header.frame_id = path[0].header.frame_id;
            gui_path.header.stamp = path[0].header.stamp;

            poses_path.header.frame_id = path[0].header.frame_id;
            poses_path.header.stamp = path[0].header.stamp;
            
        }


        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for(unsigned int i=0; i < path.size(); i++){
            gui_path.poses[i] = path[i];
            poses_path.poses[i] = path[i].pose;
        }

        plan_pub_.publish(gui_path);
        plan_poses_pub_.publish(poses_path);
    }
};

PLUGINLIB_EXPORT_CLASS(lemniscata_planner::LemniscataPlanner, nav_core::BaseGlobalPlanner)
