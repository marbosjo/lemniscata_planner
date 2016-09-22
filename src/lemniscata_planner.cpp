#include <pluginlib/class_list_macros.h>
#include <lemniscata_planner/lemniscata_planner.h>
#include <tf/transform_datatypes.h>

#include <limits>

//register this planner as a BaseGlobalPlanner plugin

using namespace std;

//Default Constructor
namespace lemniscata_planner {

    bool arePoseStampedEqual(const geometry_msgs::PoseStamped &p, const geometry_msgs::PoseStamped &q)
    {
        return (p.pose.position.x == q.pose.position.x) &&
        (p.pose.position.y == q.pose.position.y) &&
        (p.pose.position.z == q.pose.position.z) &&
        (p.pose.orientation.x == q.pose.orientation.x) &&
        (p.pose.orientation.y == q.pose.orientation.y) &&
        (p.pose.orientation.z == q.pose.orientation.z) &&
        (p.pose.orientation.w == q.pose.orientation.w);
    }

    LemniscataPlanner::LemniscataPlanner () : nh_(""), private_nh_("~"), initialized_(false){

    }

    LemniscataPlanner::LemniscataPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : nh_(""), private_nh_("~"), initialized_(false) {
        initialize(name, costmap_ros);
    }

    LemniscataPlanner::~LemniscataPlanner()
    {
        if (dyn_rec_)
            delete dyn_rec_;
    }


    void LemniscataPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        plan_pub_ = private_nh_.advertise<nav_msgs::Path>("plan", 1);
        plan_poses_pub_ = private_nh_.advertise<geometry_msgs::PoseArray>("plan_poses", 1);
        lemniscata_pub_ = private_nh_.advertise<nav_msgs::Path>("lemniscata", 1);

        dyn_rec_ = new dynamic_reconfigure::Server<lemniscata_planner::LemniscataConfig>(ros::NodeHandle("~/lemniscata_planner"));
        dyn_rec_callback_ = boost::bind(&LemniscataPlanner::reconfigureCallback, this, _1, _2);
        dyn_rec_->setCallback(dyn_rec_callback_);
        
        alpha_ = 3.0;
        beta_ = 3.0;
        sampling_points_ = 20;
        path_size_ = 5;
        keep_goal_orientation_ = false;
        
        initialized_ = true;
        must_regenerate_lemniscata_ = true;

    }

    void LemniscataPlanner::reconfigureCallback(LemniscataConfig& config, uint32_t level)
    {
        alpha_ = config.alpha;
        beta_ = config.beta;
        sampling_points_ = config.sampling_points;
        path_size_ = config.path_size;
        keep_goal_orientation_ = config.keep_goal_orientation;

        must_regenerate_lemniscata_ = true;
    }


    void LemniscataPlanner::generateLemniscata(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& lemniscata)
    {
        //double alpha = 3;
        //size_t sampling_points = 20;

        double initial_yaw = tf::getYaw(goal.pose.orientation);

        lemniscata.resize(sampling_points_);
        for (size_t i = 0; i < sampling_points_; i++) {
            double t = i*2.0*M_PI/(sampling_points_-1);
           
            double lemniscata_x = alpha_ * std::sqrt(2.0) * std::cos(t) / (std::pow(std::sin(t), 2.0) + 1.0);
            double lemniscata_y = beta_ * std::sqrt(2.0) * std::cos(t) * std::sin(t) / (std::pow(std::sin(t), 2.0) + 1);
            
            lemniscata[i].header.frame_id = goal.header.frame_id; 
            lemniscata[i].pose.position.x = goal.pose.position.x + std::cos(initial_yaw) * lemniscata_x - std::sin(initial_yaw) * lemniscata_y;
            lemniscata[i].pose.position.y = goal.pose.position.y + std::sin(initial_yaw) * lemniscata_x + std::cos(initial_yaw) * lemniscata_y;

        }

        for (size_t i = 0; i < sampling_points_; i++) {
            if (keep_goal_orientation_) {
                lemniscata[i].pose.orientation = goal.pose.orientation;
            }
            else { // orientation smooth orientation
                size_t next = (i + 1) % sampling_points_;
                double yaw = std::atan2(lemniscata[next].pose.position.y - lemniscata[i].pose.position.y, lemniscata[next].pose.position.x - lemniscata[i].pose.position.x);
                lemniscata[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            }
        }

        must_regenerate_lemniscata_ = false;
    }

    bool LemniscataPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan )
    {
        if (!arePoseStampedEqual(goal, current_goal_)) {
            current_goal_ = goal;
            must_regenerate_lemniscata_ = true;
        }
        
        if (must_regenerate_lemniscata_) {
            generateLemniscata(current_goal_, current_lemniscata_);

            size_t closest_point_to_start_index = 0;
            double closest_distance_to_start = std::numeric_limits<double>::max();

            for (size_t i = 0; i < current_lemniscata_.size(); i++) {
                double distance_to_start = std::pow(current_lemniscata_[i].pose.position.x - start.pose.position.x, 2.0) + std::pow(current_lemniscata_[i].pose.position.y - start.pose.position.y, 2.0);
                if (distance_to_start < closest_distance_to_start) {
                    closest_distance_to_start = distance_to_start;
                    closest_point_to_start_index = i;
                }
            }
            current_subgoal_index_ = closest_point_to_start_index;
        }
        else {
            size_t next_subgoal_index_ = (current_subgoal_index_ + 1) % current_lemniscata_.size();

            double distance_to_current_subgoal = std::pow(current_lemniscata_[current_subgoal_index_].pose.position.x - start.pose.position.x, 2.0) + std::pow(current_lemniscata_[current_subgoal_index_].pose.position.y - start.pose.position.y, 2.0);
            double distance_to_next_subgoal = std::pow(current_lemniscata_[next_subgoal_index_].pose.position.x - start.pose.position.x, 2.0) + std::pow(current_lemniscata_[next_subgoal_index_].pose.position.y - start.pose.position.y, 2.0);

            if (distance_to_next_subgoal < distance_to_current_subgoal)
                current_subgoal_index_ = next_subgoal_index_;
        }


//        size_t plan_size = 5;
        plan.resize(path_size_);
        for (size_t i = 0; i < path_size_; i++) {
            size_t idx = (current_subgoal_index_ + i) % current_lemniscata_.size();
            plan[i] = current_lemniscata_[idx];
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

        nav_msgs::Path lemniscata_path;
        lemniscata_path.poses.resize(current_lemniscata_.size());
        
        if(!path.empty())
        {
            gui_path.header = path[0].header;
            poses_path.header = path[0].header;
            lemniscata_path.header = path[0].header;
        }


        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (size_t i = 0; i < path.size(); i++){
            gui_path.poses[i] = path[i];
            poses_path.poses[i] = path[i].pose;
        }

        for (size_t i = 0; i < current_lemniscata_.size(); i++) {
            lemniscata_path.poses[i] = current_lemniscata_[i];
        }

        plan_pub_.publish(gui_path);
        plan_poses_pub_.publish(poses_path);
        lemniscata_pub_.publish(lemniscata_path);
    }
};

PLUGINLIB_EXPORT_CLASS(lemniscata_planner::LemniscataPlanner, nav_core::BaseGlobalPlanner)
