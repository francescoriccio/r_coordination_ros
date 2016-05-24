#include "r_coordination.hpp"

void RCoordination::generateContextualEvent(std::vector<Utils::Point2f>* _sim_events)
{
    _sim_events->push_back(Utils::Point2f(348,389)); ///DIAG_B1
//    _sim_events->push_back(Utils::Point2f(589,1200)); ///DIAG_1_floor
}

RCoordination::RCoordination(): n("~")
{
    n.param<std::string>("robot_name", robotname, "robot0");
    n.param("robots_num", robots_num, 1);

    map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, &RCoordination::mapSubscriber, this);
    move_sub = n.subscribe<std_msgs::String>("/" + robotname + "/way_point_navigation/feedbackMotion",
                                             1000, &RCoordination::moveSubscriber, this);
    tcp_sub = n.subscribe<tcp_interface::RCOMMessage>("/RCOMMessage", 1000, &RCoordination::tcpSubscriber, this);

    path_pub = n.advertise<std_msgs::String>("targetPose", 1000);
    tcp_pub = n.advertise<tcp_interface::RCOMMessage>("/RCOMMessage",1000);

    Utils::println(robotname + " active.", Utils::Yellow);
    robot_Id = getRobotId(robotname);
    Utils::print("Robot Id: ", Utils::Yellow); std::cerr<< robot_Id <<std::endl;

    init();
    usleep(1e6);
    ros::spinOnce();

    if(map.rows != 0 && map.cols != 0) coordinator = new Coordinator(robots_num, map, robot_Id);
    else
    {
        ROS_INFO("Map not loaded correctly.");
        exit(-1);
    }

}

void RCoordination::visualize()
{
//    if( cv::waitKey(33) != 27 )
    {
        cv::cvtColor(map, vis, CV_GRAY2RGB);
//        if(target_trigger==sim_target_poses.size()-1)
//        {
//            for(unsigned int e=0; e<simulated_event_poses.size(); ++e)
//                Utils::drawEvent(vis,e,Utils::co2cv_point(simulated_event_poses.at(e)));
//        }

        getRobotPose();
        float delta_x = std::fabs(robot_pose.x - sim_target_poses.at(target_trigger).x);
        float delta_y = std::fabs(robot_pose.y - sim_target_poses.at(target_trigger).y);
        if(delta_x < 50 && delta_y < 50)
//        if(target_trigger<sim_target_poses.size())
            Utils::drawTarget(vis,0,Utils::co2cv_point(sim_target_poses.at(target_trigger)));

        coordinator->status(vis);
//        cv::namedWindow("vis " + robotname, cv::WINDOW_NORMAL);
        cv::imshow("vis " + robotname,vis);
        cv::waitKey(1);
    }
}

void RCoordination::dump_data()
{
    if( dump.is_open() && isTeamConnected())
    {
        for(unsigned int rp=0; rp<robot_poses.size(); ++rp)
        {
            int distances[] = {0,0,0};
            coordinator->getTopologicalGraphHandler()->getDistancesFromPoint(robot_poses.at(rp).x,robot_poses.at(rp).y, robot_poses.at(rp).z, distances);

            dump << robot_poses.at(rp).x<<" "<<robot_poses.at(rp).y<<" "<<robot_poses.at(rp).z<<" ";
            dump << distances[0]<<" "<< distances[1]<<" "<< distances[2]<<" ";
        }
        dump << std::endl;
    }
}

void RCoordination::init()
{
    // initialize the listner
    while(ros::Time::now() == ros::Time(0));
    this->listener = new tf::TransformListener();

    // initialize robot poses vector
    end_current_target_pose = robot_pose;
    for(unsigned int rp=0; rp<robots_num; ++rp)
    {
        buffer_robot_poses.push_back(Utils::Point3f());
        robot_poses.push_back(Utils::Point3f());
    }

    move_task_state = "INIT";
    task_Id = -1;
    path_id = 0;

    human_info_trigger =-1;
    door_trigger=-1;

#ifdef DUMP_FILE
    dump.open("/home/sapienzbot/Desktop/dump.txt", std::fstream::out | std::fstream::app);
#endif

    ///* SIM -------------------------------------------------------*/
    // init simulated humans
    generateContextualEvent(&simulated_event_poses);

    target_trigger=0;
    target_time = ros::Time::now();

    door_state.insert(std::make_pair<std::string,bool>("door0", false));
    door_state.insert(std::make_pair<std::string,bool>("door1", false));
    door_state.insert(std::make_pair<std::string,bool>("door2", false));
    door_state.insert(std::make_pair<std::string,bool>("door3", false)); //office
    door_state.insert(std::make_pair<std::string,bool>("door4", true)); // restroom
    door_state.insert(std::make_pair<std::string,bool>("door5", true)); // phd
    door_state.insert(std::make_pair<std::string,bool>("door6", false));
    door_state.insert(std::make_pair<std::string,bool>("door7", false));
    door_state.insert(std::make_pair<std::string,bool>("door9", false));
    door_state.insert(std::make_pair<std::string,bool>("door10",true)); // printer
    door_state.insert(std::make_pair<std::string,bool>("door11",false));
    door_state.insert(std::make_pair<std::string,bool>("door12",false));
    door_state.insert(std::make_pair<std::string,bool>("door13",false));
    door_state.insert(std::make_pair<std::string,bool>("door14",false));
    door_state.insert(std::make_pair<std::string,bool>("door15",false));
    door_state.insert(std::make_pair<std::string,bool>("door25",false));
    door_state.insert(std::make_pair<std::string,bool>("door26",false));

    ///DIAG_1_floor
//    sim_target_poses.push_back(Utils::Point2f(589,1300)); // office
//    sim_target_poses.push_back(Utils::Point2f(855,1283));
//    sim_target_poses.push_back(Utils::Point2f(855,1090));
//    sim_target_poses.push_back(Utils::Point2f(342,1100));
//    sim_target_poses.push_back(Utils::Point2f(590,1200));
//    sim_target_poses.push_back(Utils::Point2f(339,391));
//    sim_target_poses.push_back(Utils::Point2f(360,790));

    /// DIAG_B1
    sim_target_poses.push_back(Utils::Point2f(380,464)); // office
//    sim_target_poses.push_back(Utils::Point2f(651,437));
//    sim_target_poses.push_back(Utils::Point2f(426,365));
//    sim_target_poses.push_back(Utils::Point2f(646,237));
//    sim_target_poses.push_back(Utils::Point2f(130,365));
//    sim_target_poses.push_back(Utils::Point2f(705,371));

    /// DISlabs
//    sim_target_poses.push_back(Utils::Point2f(115,310));
//    sim_target_poses.push_back(Utils::Point2f(195,247));
//    sim_target_poses.push_back(Utils::Point2f(70,370));
}

unsigned int RCoordination::getRobotId(std::string robot_n)
{
    for(unsigned int s=0;s<robot_n.length(); ++s)
    {
        if(isdigit(robot_n[s]))
            return (unsigned int)(robot_n[s]- '0');
    }
}

Utils::Point3f RCoordination::getTeamRobotPose(unsigned int r_i)
{
    Utils::Point3f robot_i_pose(buffer_robot_poses.at(r_i).x, buffer_robot_poses.at(r_i).y, buffer_robot_poses.at(r_i).z);
    return robot_i_pose;
}

void RCoordination::getRobotPose()
{
    tf::StampedTransform transform;

    if(resolution != -1)
    {
        try
        {
            listener->waitForTransform("/" + robotname + "/map", "/" + robotname + "/base_link", ros::Time(0), ros::Duration(1.0));
            listener->lookupTransform("/map", "/" + robotname + "/base_link", ros::Time(0), transform);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        robot_pose = cv::Point3f((transform.getOrigin().x() - origin_x)/resolution,
                                 map.size().height - (transform.getOrigin().y() - this->origin_y)/resolution,
                                 tf::getYaw(transform.getRotation()));
    }
}

void RCoordination::tcpSubscriber(const tcp_interface::RCOMMessage::ConstPtr& _tcp_msg)
{
    if(strcmp(_tcp_msg->robotsender.c_str(), robotname.c_str()) != 0 && strcmp(_tcp_msg->robotreceiver.c_str(), robotname.c_str()) == 0)
    {
        std::string msg;
        msg = _tcp_msg->value;
        std::istringstream iss(msg);

        int count = 0;
        Utils::Point3f tmp_p;
        RCoordination::MsgType msg_type;
        ContextMiddleware::Event::Type event_type;

#ifdef MSG_RECEIVED
        Utils::print(_tcp_msg->robotsender+" ",Utils::Yellow); Utils::println(msg,Utils::Red);
#endif

        while (iss)
        {
            float p;
            iss >> p;
            if(count == 0) msg_type = (RCoordination::MsgType) p;

            if(msg_type == RCoordination::pose)
            {
                if(count == 1) tmp_p.x = p;
                else if(count == 2) tmp_p.y = p;
                else if(count ==3) tmp_p.z = p;
            }
            else if(msg_type == RCoordination::event )
            {
                if(count == 1) event_type = (ContextMiddleware::Event::Type) p;
                else if(count == 2) tmp_p.x = p;
                else if(count == 3) tmp_p.y = p;
            }
            ++count;
        }
        if(msg_type == RCoordination::pose)
        {
            buffer_pose_mutex.lock();
            buffer_robot_poses.at(getRobotId(_tcp_msg->robotsender)) = tmp_p;
            buffer_pose_mutex.unlock();

            updateEvents( ContextMiddleware::Event(ContextMiddleware::Event::area_cleared, Utils::Point2f(tmp_p.x,tmp_p.y)) );
        }
        else if(msg_type == RCoordination::event )
            updateEvents( ContextMiddleware::Event(event_type, Utils::Point2f(tmp_p.x,tmp_p.y)) );
    }
}

void RCoordination::moveSubscriber(const std_msgs::String::ConstPtr& _msg)
{
    move_task_state = _msg->data;
}

void RCoordination::mapSubscriber(const nav_msgs::OccupancyGrid::ConstPtr& _map)
{
    int width = _map->info.width;
    int height = _map->info.height;

    map = cv::Mat(height, width, CV_8U);
    resolution = _map->info.resolution;
    origin_x = _map->info.origin.position.x;
    origin_y = _map->info.origin.position.y;

    ROS_INFO("Map received.");

    for(int i = 0, i_rev = height - 1; i < height;++i, --i_rev)
    {
        for(int j = 0; j < width; j++)
        {
            switch(_map->data[i_rev*width + j])
            {
            default:
            case -1:
                this->map.data[i*width + j] = 150;
                break;
            case 0:
                this->map.data[i*width + j] = 255;
                break;
            case 100:
                this->map.data[i*width + j] = 0;
                break;
            }
        }
    }

    ROS_INFO("Image extracted from map.");
}

bool RCoordination::isDoorOpen(Utils::Point2f door)
{
    for(unsigned int d=0; d<coordinator->getTopologicalGraphHandler()->getDoors()->size(); ++d)
    {
        if(norm2D(door,Utils::Point2f(coordinator->getTopologicalGraphHandler()->getDoors()->at(d).x,
                                           coordinator->getTopologicalGraphHandler()->getDoors()->at(d).y))< 5)
        {
            // if nardi is not in his office the door is closed
            if(coordinator->getTopologicalGraphHandler()->getNodes()->at( coordinator->getTopologicalGraphHandler()->
                                    getClosestNodeId(coordinator->getTopologicalGraphHandler()->getDoors()->at(d).x,
                                                     coordinator->getTopologicalGraphHandler()->getDoors()->at(d).y))->room_label=="door2")
            {
                if( (double)(ros::Time::now() - target_time).toSec() > 30 ) // DEMO purposes
                    if(target_trigger==0) return true;
                else return false;
            }

            // phd room door state changes
            if(coordinator->getTopologicalGraphHandler()->getNodes()->at( coordinator->getTopologicalGraphHandler()->
                                    getClosestNodeId(coordinator->getTopologicalGraphHandler()->getDoors()->at(d).x,
                                                     coordinator->getTopologicalGraphHandler()->getDoors()->at(d).y))->room_label=="door5")
            {
                if( (double)(ros::Time::now() - target_time).toSec() > 60 ) // DEMO purposes
                    return false;
                else return true;
            }

            return door_state[coordinator->getTopologicalGraphHandler()->getNodes()->at( coordinator->getTopologicalGraphHandler()->
                        getClosestNodeId(coordinator->getTopologicalGraphHandler()->getDoors()->at(d).x,
                                         coordinator->getTopologicalGraphHandler()->getDoors()->at(d).y))->room_label];
        }
    }

    return false;
}

bool RCoordination::isTeamConnected()
{
    for(unsigned int tc=0; tc<robot_poses.size(); ++tc)
    {
        if(robot_poses.at(tc).x == .0f) return false;
    }

    return true;
}

bool RCoordination::reassignTask(unsigned int n_task_id)
{
    if(n_task_id != task_Id)
    {
        move_task_state = "INIT";
        return true;
    }
    else return false;

}

bool RCoordination::reassignTask(cv::Point3f end_task_pose)
{
    if(end_current_target_pose != end_task_pose)
    {
        move_task_state = "INIT";
        return true;
    }
    else return false;
}

void RCoordination::eventDetector()
{
    tcp_interface::RCOMMessage tcp_event_msg;
    tcp_event_msg.robotsender = robotname;
    tcp_event_msg.robotreceiver = "all";

    // local area cleared
    updateEvents(ContextMiddleware::Event(ContextMiddleware::Event::area_cleared, Utils::Point2f(robot_pose.x,robot_pose.y)));

    // if found target which we are looking for
    if(target_trigger<sim_target_poses.size())
    {
        if(Utils::norm2D(Utils::Point2f(robot_pose.x,robot_pose.y), sim_target_poses.at(target_trigger)) < NEAR_TARGET_THRESHOLD)
        {
            tcp_event_msg.value = Utils::to_string(RCoordination::event) + " " +
                    Utils::to_string(ContextMiddleware::Event::robot_succeed) + " " +
                    Utils::to_string(robot_pose.x) + " " +
                    Utils::to_string(robot_pose.y);
            tcp_pub.publish(tcp_event_msg);

            updateEvents( ContextMiddleware::Event(ContextMiddleware::Event::robot_succeed, Utils::Point2f(robot_pose.x,robot_pose.y)));
        }
    }

#ifdef ONLY_CLEAR_AREA_EVENT
    return;
#endif

    // if near door and detect
    for(unsigned int d=0; d<coordinator->getTopologicalGraphHandler()->getDoors()->size(); ++d)
    {
        Utils::Point2f door(
                    coordinator->getTopologicalGraphHandler()->getDoors()->at(d).x,
                    coordinator->getTopologicalGraphHandler()->getDoors()->at(d).y);

        if(Utils::norm2D(Utils::Point2f(robot_pose.x,robot_pose.y),door) < NEAR_TARGET_THRESHOLD)
        {

            if(door_trigger == coordinator->getTopologicalGraphHandler()->getClosestNodeId(door.x,door.y)) break;
            door_trigger = coordinator->getTopologicalGraphHandler()->getClosestNodeId(door.x,door.y);

#ifdef VIS_DOOR_STATUS
            Utils::print("Door "+Utils::to_string(d)+ ": ");
#endif
            if(!isDoorOpen(door))
            {
#ifdef VIS_DOOR_STATUS
                Utils::println("closed", Utils::Blue);
#endif
                tcp_event_msg.value = Utils::to_string(RCoordination::event) + " " +
                        Utils::to_string(ContextMiddleware::Event::closed_door) + " " +
                        Utils::to_string(robot_pose.x) + " " +
                        Utils::to_string(robot_pose.y);
                tcp_pub.publish(tcp_event_msg);

                updateEvents(ContextMiddleware::Event(ContextMiddleware::Event::closed_door, Utils::Point2f(robot_pose.x,robot_pose.y)));
            }
            else
            {
#ifdef VIS_DOOR_STATUS
                Utils::println("open", Utils::Yellow);
#endif
//                tcp_event_msg.value = Utils::to_string(RCoordination::event) + " " +
//                        Utils::to_string(ContextMiddleware::Event::open_door) + " " +
//                        Utils::to_string(robot_pose.x) + " " +
//                        Utils::to_string(robot_pose.y);
//                tcp_pub.publish(tcp_event_msg);

                updateEvents(ContextMiddleware::Event(ContextMiddleware::Event::open_door, Utils::Point2f(robot_pose.x,robot_pose.y)));
            }
            break;
        }
    }

    // if near a person event which tells a location
    if(target_trigger == sim_target_poses.size()-1 && 0) /// SIMULATION purposes
    {
        for( unsigned int se=0; se<simulated_event_poses.size(); ++se )
        {
            if(human_info_trigger ==
                    coordinator->getTopologicalGraphHandler()->getClosestNodeId(simulated_event_poses.at(se).x,simulated_event_poses.at(se).y)) break;
            human_info_trigger = coordinator->getTopologicalGraphHandler()->getClosestNodeId(simulated_event_poses.at(se).x,simulated_event_poses.at(se).y);

            if(Utils::norm2D(Utils::Point2f(robot_pose.x,robot_pose.y), simulated_event_poses.at(se)) < NEAR_TARGET_THRESHOLD)
            {
                Utils::println("A person told me to have seen the target near the printer!",Utils::Magenta);
                tcp_event_msg.value = Utils::to_string(RCoordination::event) + " " +
                        Utils::to_string(ContextMiddleware::Event::human_info) + " " +
                        Utils::to_string(robot_pose.x) + " " +
                        Utils::to_string(robot_pose.y);
                tcp_pub.publish(tcp_event_msg);

                updateEvents( ContextMiddleware::Event(ContextMiddleware::Event::human_info, Utils::Point2f(robot_pose.x,robot_pose.y)), "tprinter" );
            }
        }
    }
}

void RCoordination::updateRandomWalk()
{
    // timeout check
    if(target_time != ros::Time(0))
    {
        if((double)(ros::Time::now() - target_time).toSec() > TIMEOUT_THRESHOLD)
        {
            Utils::println("Timeout elapsed.", Utils::Red);
            target_time = ros::Time::now();
            ++target_trigger;
        }
    }

    // send robot pose
    tcp_interface::RCOMMessage tcp_robot_pose_msg;
    tcp_robot_pose_msg.robotsender = robotname;
    tcp_robot_pose_msg.robotreceiver = "all";
    tcp_robot_pose_msg.value = Utils::to_string(RCoordination::pose) + " " +
            Utils::to_string(robot_pose.x) + " " +
            Utils::to_string(robot_pose.y) + " " +
            Utils::to_string(robot_pose.z);
    tcp_pub.publish(tcp_robot_pose_msg);

    // update team poses
    updateRobotPoses();

    if(isTeamConnected())
    {
        //detect local occurring events
        eventDetector();

        coordinator->updateRobotPoses(robot_poses);
        coordinator->updateContexts();

        if( Utils::norm2D(Utils::cv2co_point(robot_pose).translation(),
                               Utils::cv2co_point(end_current_target_pose).translation()) < NEAR_TARGET_THRESHOLD
                || norm(end_current_target_pose) == 0)
        {
#ifdef MEM_RND_WALK
            Utils::Point2f rnd_target(coordinator->getRandomPTarget());
#else
            Utils::Point2f rnd_target(coordinator->getRandomTarget());
#endif
            end_current_target_pose = cv::Point3f(rnd_target.x, rnd_target.y, 0.f);
            ++path_id;
            Utils::print("Current Random Target for robot: ");std::cerr<<robot_Id<<" "<< end_current_target_pose <<std::endl;

            std::vector<cv::Point3f> path;
            coordinator->getTopologicalGraphHandler()->findPath(
                        coordinator->getTopologicalGraphHandler()->getNodes()->at(
                            coordinator->getTopologicalGraphHandler()->getClosestNodeId(robot_pose.x,robot_pose.y) ),
                        coordinator->getTopologicalGraphHandler()->getNodes()->at(
                            coordinator->getTopologicalGraphHandler()->getClosestNodeId(end_current_target_pose.x,end_current_target_pose.y) ),
                        &path);

            path_to_publish.data = std::string("Path_-"+Utils::to_string(path_id)+" 0");
            for(unsigned int i=0; i<path.size(); ++i)
            {
                path_to_publish.data +=
                        + " " + Utils::to_string(
                            (path.at(i).x *resolution) + origin_x)
                        + " " + Utils::to_string(
                            -(path.at(i).y -map.size().height) *resolution + origin_y)
                        + " " + Utils::to_string(
                            path.at(i).z);
            }
            path_pub.publish(path_to_publish);
        }
    }
    else Utils::println(robotname + " is up and waits for teammates!", Utils::Cyan);


#ifdef COORD_VIS
    if(robot_Id == 0)
        visualize();
#endif

#ifdef DUMP_FILE
    if(robot_Id == 1)
        dump_data();
#endif
}

void RCoordination::updateEvents(ContextMiddleware::Event local_event, std::string label)
{
    if(local_event.type == ContextMiddleware::Event::robot_succeed)
    {
        if(target_time != ros::Time(0))
        {
//            Utils::print("Target found in: ", Utils::Green);
//            std::cerr<< (double)(ros::Time::now() - target_time).toSec();
//            Utils::println(" sec.", Utils::Green);

            target_time = ros::Time::now();
//            ++target_trigger;
        }
    }
    coordinator->readEvent(local_event.type, local_event.pos, label);
}

void RCoordination::updateRobotPoses()
{
    getRobotPose();
    buffer_pose_mutex.lock();
    for(unsigned int rp=0; rp<robot_poses.size(); ++rp)
    {
        if(rp == robot_Id) robot_poses.at(robot_Id) = Utils::cv2co_point(robot_pose);
        else robot_poses.at(rp) = getTeamRobotPose(rp);
    }
    buffer_pose_mutex.unlock();
}

void RCoordination::update()
{
    // timeout check
    if(target_time != ros::Time(0))
    {
        std::cerr<<"Sim time (sec): "<<(double)(ros::Time::now() - target_time).toSec()<<std::endl;
        if((double)(ros::Time::now() - target_time).toSec() > TIMEOUT_THRESHOLD)
        {
            Utils::println("Timeout elapsed.", Utils::Red);
            target_time = ros::Time::now();
            ++target_trigger;
        }
    }

    // send robot pose
    tcp_interface::RCOMMessage tcp_robot_pose_msg;
    tcp_robot_pose_msg.robotsender = robotname;
    tcp_robot_pose_msg.robotreceiver = "all";
    tcp_robot_pose_msg.value = Utils::to_string(RCoordination::pose) + " " +
            Utils::to_string(robot_pose.x) + " " +
            Utils::to_string(robot_pose.y) + " " +
            Utils::to_string(robot_pose.z);
    tcp_pub.publish(tcp_robot_pose_msg);

    // update team poses
    updateRobotPoses();

    if(isTeamConnected())
    {
        // detect local events
        eventDetector();

        // update coordinator
        coordinator->updateRobotPoses(robot_poses);
        coordinator->updateContexts();
        coordinator->update();

        try
        {
            // handling waypoint navigator
            bool reassign_id = false;
            bool reassign_path = false;
            if( reassignTask(coordinator->getMappingFunction(robot_Id)->begin()->first) )
            {
                reassign_id = true;
                task_Id = -1;
            }
            if( reassignTask(coordinator->getMappingFunction(robot_Id)->begin()->second.at(
                                 coordinator->getMappingFunction(robot_Id)->begin()->second.size()-1)) )
            {
                reassign_path = true;
                task_Id = -1;
            }

            if(task_Id == -1)
            {
                if(reassign_id) task_Id = coordinator->getMappingFunction(robot_Id)->begin()->first;
                if(reassign_path) end_current_target_pose = coordinator->getMappingFunction(robot_Id)->begin()->second.at(
                            coordinator->getMappingFunction(robot_Id)->begin()->second.size()-1);
                ++path_id;

                path_to_publish.data = std::string("Path_-"+Utils::to_string(path_id)+" 0");
                for(unsigned int i=0; i<coordinator->getMappingFunction(robot_Id)->begin()->second.size(); ++i)
                {
                    path_to_publish.data +=
                            + " " + Utils::to_string(
                                (coordinator->getMappingFunction(robot_Id)->begin()->second.at(i).x *resolution) + origin_x)
                            + " " + Utils::to_string(
                                -(coordinator->getMappingFunction(robot_Id)->begin()->second.at(i).y -map.size().height) *resolution + origin_y)
                            + " " + Utils::to_string(
                                coordinator->getMappingFunction(robot_Id)->begin()->second.at(i).z);
                }
                path_pub.publish(path_to_publish);
            }
        }
        catch(...)
        {
            Utils::println("Coordinator output does not full fill the requirements.",Utils::Red);
            exit(-1);
        }
    }
    else Utils::println(robotname + " is up and waits for teammates!", Utils::Cyan);

#ifdef COORD_VIS
    if(robot_Id == 0)
        visualize();
#endif
}

int main(int argc, char** argv)
{
    //TODO coordinator->getContextMiddlewareHandler()->getRandomPTarget()->pos
    ros::init(argc, argv, "topological_graph");

    RCoordination coordination_node;
    ros::Rate loop_rate(5); // [Hz]

    while(ros::ok())
    {
        srand(time(NULL));

#ifndef RANDOM_WALK
        coordination_node.update();
#else
        coordination_node.updateRandomWalk();
#endif

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
