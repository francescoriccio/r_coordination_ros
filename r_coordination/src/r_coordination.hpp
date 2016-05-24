#ifndef _TOPOLOGICAL_NODE_HPP_
#define _TOPOLOGICAL_NODE_HPP_

#include <coordinator/coordinator.h>
#include <utils/utils.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <boost/thread/mutex.hpp>
#include "tf/transform_listener.h"

#include <opencv2/highgui/highgui.hpp>

#include <tcp_interface/RCOMMessage.h>

#define RANDOM_WALK
//#define MEM_RND_WALK
//#define MSG_RECEIVED
//#define ONLY_CLEAR_AREA_EVENT

//#define COORD_VIS
//#define DUMP_FILE
#define VIS_DOOR_STATUS
#define NEAR_TARGET_THRESHOLD 10 //[pixels]
#define TIMEOUT_THRESHOLD 300 //[s]


class RCoordination
{
private:
    enum MsgType
    {
        pose = 0,
        event
    };
    ros::NodeHandle n;
    std::string robotname;
    unsigned int robot_Id;
    int task_Id;
    unsigned int path_id;
    int robots_num;
    std::string move_task_state;
    cv::Point3f robot_pose;
    cv::Point3f end_current_target_pose;
    Coordinator* coordinator;

    std::vector<Utils::Point3f> buffer_robot_poses;
    std::vector<Utils::Point3f> robot_poses;

    std_msgs::String path_to_publish;

    cv::Mat map;
    cv::Mat vis;

    float resolution;
    float origin_x;
    float origin_y;

    ros::Publisher path_pub;
    ros::Publisher tcp_pub;

    ros::Subscriber map_sub;
    ros::Subscriber move_sub;
    ros::Subscriber tcp_sub;

    boost::mutex buffer_pose_mutex;
    tf::TransformListener* listener;
    ros::Time target_time;

    std::fstream dump;
    //// SIM
    //simulate event "humans tell"
    std::vector<Utils::Point2f> simulated_event_poses;
    std::map<std::string,bool> door_state;
    int human_info_trigger;
    int door_trigger;
    std::vector<Utils::Point2f> sim_target_poses;
    unsigned int target_trigger;
    boost::mutex target_trigger_mutex;

public:
    void generateContextualEvent(std::vector<Utils::Point2f>* _sim_events); //SIM

    RCoordination();
    ~RCoordination(){}

    void visualize();
    void dump_data();
    void init();

    unsigned int getRobotId(std::string robot_n);
    inline std_msgs::String getPathString(){ return path_to_publish; }
    inline ros::Publisher getPathPub(){ return path_pub; }
    inline ros::Publisher getTcpPub(){ return tcp_pub; }
    inline cv::Mat getMap(){ return map; }

    Utils::Point3f getTeamRobotPose(unsigned int r_i);
    void getRobotPose();
    void tcpSubscriber(const tcp_interface::RCOMMessage::ConstPtr& _tcp_msg);
    void mapSubscriber(const nav_msgs::OccupancyGrid::ConstPtr& _map);
    void moveSubscriber(const std_msgs::String::ConstPtr& _msg);

    bool isDoorOpen(Utils::Point2f door);
    bool isTeamConnected();
    bool reassignTask(unsigned int n_task_id);
    bool reassignTask(cv::Point3f end_task_pose);
    void eventDetector();

    void updateRandomWalk();
    void updateEvents(ContextMiddleware::Event local_event, std::string label = "");
    void updateRobotPoses();
    void update();
};

#endif
