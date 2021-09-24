#include "freicar_map/thrift_map_proxy.h"
#include "freicar_map/planning/lane_follower.h"
#include "freicar_map/planning/lane_star.h"
#include "freicar_map/logic/right_of_way.h"

#include "map_core/freicar_map_helper.h"
#include "map_core/freicar_map_config.h"

#include "freicar_common/shared/planner_cmd.h"
#include "freicar_common/WayPoint.h"

#include <ros/package.h>
#include <ros/ros.h>
#include <cstdio>
#include <thread>
#include <ctime>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/Bool.h"
#include "yaml.h"
#include "yaml-cpp/yaml.h"




#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/transform_storage.h>
#include <tf2/buffer_core.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <freicar_common/FreiCarAgentLocalization.h>
#include <freicar_common/FreiCarControl.h>


#include <map_core/freicar_map.h>
#include <freicar_map/planning/lane_follower.h>
#include <raiscar_msgs/ControlCommand.h>

freicar_common::FreiCarAgentLocalization p_current_row;
freicar_common::FreiCarAgentLocalization enemy_agent, p_observed_agent;
//tf2_ros::Buffer tf_buffer_;
#define DENSIFICATION_LIMIT 0.22 // meters
ros::Subscriber goal_reached;
std_msgs::Bool goal_bool;
std_msgs::Bool overtake;
freicar::mapobjects::Point3D p_current;
freicar::mapobjects::Lane *current_lane;
ros::Publisher snm_pub;
ros::Publisher snm_pub1;
ros::Publisher jun_pub;
freicar_common::FreiCarControl HLC_msg;
bool HLC_bool;
bool junction_arrived;
unsigned char HLC_enum;
std::string car_name;
//freicar::mapobjects::Uuid uuid_;
//float offset_;
//freicar::mapobjects::;  //velocity
auto& map_instance = freicar::map::Map::GetInstance();


int count=0;
int start_flag =0;
int continue_flag =0;
int junction_id = -100;

static geometry_msgs::Point ToGeometryPoint(const freicar::mapobjects::Point3D& pt) {
    geometry_msgs::Point rt;
    rt.x = pt.x();
    rt.y = pt.y();
    rt.z = pt.z();
    return rt;
}
/* WayPoint service request handler
   Edge cases: if the closest point to current_pos is at index i of lane L
		1) current_pos -> L[i] goes backward, L[i+1] goes forward, L goes on for a couple of steps
		2) current_pos -> L[i] goes backward, L ends at index i
*/
bool HandleWayPointRequest(freicar_common::WayPointRequest &req, freicar_common::WayPointResponse &resp) {
    auto command = static_cast<freicar::enums::PlannerCommand>(req.command);
    auto current_pos = freicar::mapobjects::Point3D(req.current_position.x, req.current_position.y, req.current_position.z);
    auto plan = freicar::planning::lane_follower::GetPlan(current_pos, command, req.distance,req.node_count);
    for (size_t i = 0; i < plan.steps.size(); ++i) {
        resp.points.emplace_back(ToGeometryPoint(plan.steps[i].position));
        resp.description.emplace_back(static_cast<unsigned char>(plan.steps[i].path_description));
    }
    return plan.success;
}

/* debugging function to publish plans (either from the lane_star or lane_follower planners) */
void PublishPlan (freicar::planning::Plan& plan, double r, double g, double b, int id, const std::string& name, ros::Publisher& pub) {
    visualization_msgs::MarkerArray list;
    visualization_msgs::Marker *step_number = new visualization_msgs::Marker[plan.size()];
    int num_count = 0;
    visualization_msgs::Marker plan_points;
    plan_points.id = id;
    plan_points.ns = name;
    plan_points.header.stamp = ros::Time();
    plan_points.header.frame_id = "map";
    plan_points.action = visualization_msgs::Marker::ADD;
    plan_points.type = visualization_msgs::Marker::POINTS;
    plan_points.scale.x = 0.03;
    plan_points.scale.y = 0.03;
    plan_points.pose.orientation = geometry_msgs::Quaternion();
    plan_points.color.b = b;
    plan_points.color.a = 0.7;
    plan_points.color.g = g;
    plan_points.color.r = r;
    geometry_msgs::Point p;
    for (size_t i = 0; i < plan.size(); ++i) {
        step_number[i].id = ++num_count + id;
        step_number[i].pose.position.x = p.x = plan[i].position.x();
        step_number[i].pose.position.y = p.y = plan[i].position.y();
        p.z = plan[i].position.z();
        step_number[i].pose.position.z = plan[i].position.z() + 0.1;

        step_number[i].pose.orientation = geometry_msgs::Quaternion();
        step_number[i].ns = name + "_nums";
        step_number[i].header.stamp = ros::Time();
        step_number[i].header.frame_id = "map";
        step_number[i].action = visualization_msgs::Marker::ADD;
        step_number[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        step_number[i].text = std::to_string(i);
        step_number[i].scale.z = 0.055;
        step_number[i].color = plan_points.color;
        list.markers.emplace_back(step_number[i]);
        plan_points.points.emplace_back(p);
    }
    list.markers.emplace_back(plan_points);
    pub.publish(list);
    delete[] step_number;
};

void callback_car_localization (freicar_common::FreiCarAgentLocalization msg)
{
//    return msg;
//
    p_current.SetX(msg.current_pose.transform.translation.x);
    p_current.SetY(msg.current_pose.transform.translation.y);
    p_current_row = msg;

}

void callback_car_bestpart (geometry_msgs::PoseArray msg)
{

    if(!msg.poses.empty()){
        p_current.SetX(msg.poses.at(0).position.x);
        p_current.SetY(msg.poses.at(0).position.y);
    }
}
void callback_overtake (std_msgs:: Bool msg){
    if (msg.data == true)
    {
        overtake.data = true;
    }
    else
    {
        overtake.data = false;
    }
}

void callback_HLC (freicar_common::FreiCarControl msg)
{
    HLC_msg.command = msg.command;
    HLC_msg.name = msg.name;
    HLC_bool = true;

    std::shared_ptr<ros::NodeHandle> node_handle1 = std::make_shared<ros::NodeHandle>();
    node_handle1->getParam("carname", car_name);
    if(HLC_msg.command == "start" && HLC_msg.name == car_name)
    {
        HLC_enum = 3;
    }
    else if (HLC_msg.command == "straight" && HLC_msg.name == car_name)
    {
        HLC_enum = 3;
    }
    else if (HLC_msg.command == "left" && HLC_msg.name == car_name)
    {
        HLC_enum = 1;
    }
    else if (HLC_msg.command == "right" && HLC_msg.name == car_name)
    {
        HLC_enum = 2;
    }
    else
    {
        HLC_enum = 4;
    }
}

void callback_goal_reached (std_msgs::Bool msg)
{
    if (msg.data == true)
    {
        goal_bool.data = true;
    }
    else
    {
        goal_bool.data = false;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_framework");
    std::shared_ptr<ros::NodeHandle> node_handle = std::make_shared<ros::NodeHandle>();


    ROS_INFO("map framework node started...");


    // starting map proxy
    freicar::map::ThriftMapProxy map_proxy("127.0.0.1", 9091, 9090);
    bool new_map = false;
    //std::string filename = ros::package::getPath("freicar_map") + "/maps/thriftmap_fix.aismap";
    std::string filename;
    if (!ros::param::get("/map_path", filename)) {
        ROS_ERROR("could not find parameter: map_path! map initialization failed.");
        return 0;
    }

    if (!map_proxy.LoadMapFromFile(filename)) {
        ROS_INFO("could not find thriftmap file: %s", filename.c_str());
        map_proxy.StartMapServer();
        // stalling main thread until map is received
        ROS_INFO("waiting for map...");
        while(freicar::map::Map::GetInstance().status() == freicar::map::MapStatus::UNINITIALIZED) {
            ros::Duration(1.0).sleep();
        }
        ROS_INFO("map received");
        new_map = true;
    }
    srand(300);
    // needed some delay before using the map
    ros::Duration(2.0).sleep();
    if (new_map) {
        // thrift creates a file on failed reads
        remove(filename.c_str());
        map_proxy.WriteMapToFile(filename);
        ROS_INFO("saved new map");
    }
    // NOTES:
    // 1) NEVER post process before saving. kinda obvious but still
    // 2) increasing densification limit past a certain point or turning it off
    //    WILL introduce bugs. 22cm seems to be balanced.
    // 	  Possible bug with densification disabled:
    // 		  2 closest lane point to (3.30898, 1.46423, 0) belong to a junction
    // 		  despite obviously belonging to d4c7ecc5-0aa9-49a8-8642-5f8ebb965592
    freicar::map::Map::GetInstance().PostProcess(DENSIFICATION_LIMIT);
    srand(time(NULL));
    using namespace freicar::planning::lane_star;
    ros::Publisher tf = node_handle->advertise<visualization_msgs::MarkerArray>("planner_debug", 10, true);

    using freicar::mapobjects::Point3D;
    LaneStar planner(100);
    // auto varz = freicar::map::Map::GetInstance().FindClosestLanePoints(0.0f, 0.0f, 0.0f, 2);
//     auto plan1 = planner.GetPlan(Point3D(1.95487, 3.73705, 0), -3.13996f, Point3D(3.3805, 0.721756, 0), -1.14473f, 0.30);


//     auto plan2 = planner.GetPlan(Point3D(3.3805, 0.721756, 0), -2.99003f, Point3D(6.10168, 1.2007, 0), -2.21812f, 0.30);

    using freicar::mapobjects::Point3D;
//    LaneStar planner(100);
    using freicar::enums::PlannerCommand;



    std::thread debug_thread([&]() {
        using namespace std::chrono_literals;
        while (ros::ok()) {
            planner.ResetContainers();
            std::shared_ptr<ros::NodeHandle> node_handle1 = std::make_shared<ros::NodeHandle>();
            node_handle1->getParam("carname", car_name);

            ros::Subscriber sub4 = node_handle->subscribe("best_particle", 1, callback_car_bestpart);
            ros::Subscriber sub = node_handle->subscribe(car_name+"/goal_reached", 1, callback_goal_reached);
            ros::Subscriber HL_sub = node_handle->subscribe("/freicar_commands", 1, callback_HLC);
            ros::Subscriber sub_overtake = node_handle->subscribe("/overtake", 1, callback_overtake);

            auto &map = freicar::map::Map::GetInstance();
            const freicar::mapobjects::Lane *current_lane;
            std::vector<freicar::mapobjects::Point3D> current_points;

            // if a new agent is spawning, get a random start position, go from there
            //p_current.SetCoords(p_current.x(), p_current.y(), p_current.z());

            auto p_closest = map.FindClosestLanePoints(p_current.x(),
                                                       p_current.y(),
                                                       p_current.z(),
                                                       1)[0].first;
            current_lane = map.FindLaneByUuid(p_closest.GetLaneUuid());


            auto  stop_line = current_lane->GetStopLine();
//            auto  junction_line = current_lane->IsJunctionLane();
            auto Road_sign = current_lane->GetRoadSigns();
//            auto sign_type = current_lane->GetRoadSigns().at(0)->GetSignType();

//            std::cout << " stop_line" << stop_line << std::endl;

            snm_pub = node_handle->advertise<std_msgs::Bool>("Stop_sign", 1);
            snm_pub1 = node_handle->advertise<std_msgs::Bool>("HLC_stop", 1);

            if(!Road_sign.empty()) {
//                std::cout << " Road_sign" << Road_sign.at(0) << std::endl;
                auto sign_type = current_lane->GetRoadSigns().at(0)->GetSignType();
//                std::cout << " Sign Type" << sign_type << std::endl;
                if((sign_type == "Stop" && stop_line != 0)){   //TODO STOP
                    std::cout << "  I want to stop" << std::endl;
                    std_msgs::Bool stop_flag;
//                    ros::Duration(0.2).sleep();
                    stop_flag.data = true;
//                    ros::Duration(0.5).sleep();
//                    rate.sleep();
                    snm_pub.publish(stop_flag);
                    ros::Duration(1.5).sleep();
                    stop_flag.data = false;

                    snm_pub.publish(stop_flag);
                    ros::Duration(5).sleep();
                    //continue;
                }
            }

            std_msgs::Bool HLCstop_flag;
            if(HLC_msg.command == "stop" && HLC_msg.name == car_name) {

                HLCstop_flag.data = true;
                snm_pub1.publish(HLCstop_flag);
            }
            if (HLC_msg.command == "start" && HLC_msg.name == car_name) {
                    HLCstop_flag.data = false;
                    snm_pub1.publish(HLCstop_flag);
            }

            if(HLC_msg.command == "start" && HLC_msg.name == car_name && count == 0)
            {
                start_flag = 1;
                count=1;
            }

            jun_pub = node_handle->advertise<std_msgs::Bool>("JUNC_sign", 1);
            std::string current_l_uuid = p_closest.GetLaneUuid();
            std_msgs::Bool junction_flag;

            if (map.GetUpcomingJunctionID(current_l_uuid) != -1) {
                auto  junction_left = current_lane->JUNCTION_LEFT;
                junction_flag.data = true;
                jun_pub.publish(junction_flag);
                junction_flag.data = false;
                jun_pub.publish(junction_flag);
                junction_arrived = true;
                //std::cout << "on a lane that leads to a junction" << std::endl;

            } else {
                auto* current_lane = map.FindLaneByUuid(current_l_uuid);
                auto* next_lane = current_lane->GetConnection(freicar::mapobjects::Lane::STRAIGHT);
                auto next_l_uuid = next_lane->GetUuid().GetUuidValue();
//                if (next_lane && map.GetUpcomingJunctionID(next_l_uuid) != -1)
//                {
//                    std::cout << "two lanes away from a junction" << std::endl;
//                    junction_arrived = true;
//                } else {
//                    std::cout << "not really close to a junction" << std::endl;
//                }
            }

            //Position of all the roadsign
//            auto var = map.getSigns();
//            for (int i = 0; i < var.size(); ++i) {
//                std::cout << "Map signs string "<< i << " "<< var[i].GetUuid().GetUuidValue() << std::endl;
//            std::cout << "x signs: " << var[i].GetPosition().x() << std::endl;
//            std::cout << "y signs: " << var[i].GetPosition().y() << std::endl;
//            std::cout << "z signs: " << var[i].GetPosition().z() << std::endl;
//            }

//            float g = sog->GetOffset();
            std::float_t spawn_x=0, spawn_y=0, spawn_z=0, spawn_heading=0;
            std::shared_ptr<ros::NodeHandle> node_handle2 = std::make_shared<ros::NodeHandle>();
            node_handle2->getParam("/freicar_"+car_name+"_carla_proxy/spawn/x", spawn_x);
            node_handle2->getParam("/freicar_"+car_name+"_carla_proxy/spawn/y", spawn_y);
            node_handle2->getParam("/freicar_"+car_name+"_carla_proxy/spawn/z", spawn_z);
            node_handle2->getParam("/freicar_"+car_name+"_carla_proxy/spawn/heading", spawn_heading);


            if (p_current.x()+p_current.y()!=0 && start_flag==1)
            {
                start_flag = 0;
                //std::cout<<"Path set first time"<<std::endl;
                auto plan1 = freicar::planning::lane_follower::GetPlan(Point3D(spawn_x, spawn_y , 0), freicar::enums::PlannerCommand{HLC_enum}, 15,30);
                PublishPlan(plan1, 1.0, 0.1, 0.4, 300, "plan_1", tf);


            }
           // auto p_closest_plan = map.FindClosestLanePoints(p_current.x(),
//                                                             p_current.y(),
//                                                             p_current.z(),
//                                                             1)[0].first;

//             if (map.GetUpcomingJunctionID(current_l_uuid)!=-1  && map.GetUpcomingJunctionID(current_l_uuid) != junction_id)
//             {

//                 static tf::TransformListener enemy_listener;
//                 static tf::StampedTransform enemy_transform;





//                 try{
//                     enemy_listener.lookupTransform("map", "enemy_agent",
//                                                    ros::Time(0), enemy_transform);

//                     enemy_agent.current_pose.transform.translation.x = enemy_transform.getOrigin().x();
//                     enemy_agent.current_pose.transform.translation.y = enemy_transform.getOrigin().y();
//                     enemy_agent.current_pose.transform.translation.z = 0.0f;

//                     enemy_agent.current_pose.transform.rotation.x = 0.0;
//                     enemy_agent.current_pose.transform.rotation.y = 0.0;
//                     enemy_agent.current_pose.transform.rotation.z = 0.0;
//                     enemy_agent.current_pose.transform.rotation.w = 1.0;

//                     auto enemy_approximate_lane_points = map.FindClosestLanePoints(enemy_agent.current_pose.transform.translation.x,
//                                                                                    enemy_agent.current_pose.transform.translation.y,
//                                                                                    enemy_agent.current_pose.transform.translation.z,
//                                                                                    1)[0].first;

//                     std::string enemy_lane_uuid = enemy_approximate_lane_points.GetLaneUuid();
//                     enemy_agent.lane_uuid = enemy_lane_uuid;

//                     std::vector<freicar::logic::JunctionAgent> enemy_agents;
//                     enemy_agents.push_back(enemy_agent);


//                 }
//                 catch (tf::TransformException ex){
//                     ROS_ERROR("%s",ex.what());
//                     ros::Duration(1.0).sleep();
//                 }

//                 junction_id = map.GetUpcomingJunctionID(current_l_uuid);

//                // auto plan2 = freicar::planning::lane_follower::GetPlan(Point3D(p_closest_plan.x(),p_closest_plan.y() , 0),freicar::enums::PlannerCommand{HLC_enum}, 15,30);

// //                auto plan2 = freicar::planning::lane_follower::GetPlan(Point3D(p_closest_plan.x(),p_closest_plan.y() , 0), freicar::enums::PlannerCommand{HLC_enum}, 15,25);
//                 // reeset HLC
//                // HLC_enum = 4;

// //                auto plan2 = freicar::planning::lane_follower::GetPlan(Point3D(previous_lane_points.x(),previous_lane_points.y() , 0), freicar::enums::PlannerCommand{HLC_enum}, 15,25);
//                 //PublishPlan(plan2, 1.0, 0.1, 0.4, 300, "plan_1", tf);
//                // junction_arrived = false;
//                 HLC_bool = false;

//             }

            if(goal_bool.data == true) {
//                std::cout<<"HOW MANY TIMES IS THIS CALLED ?"<<std::endl;
//
                auto plan = freicar::planning::lane_follower::GetPlan(Point3D(p_current.x(), p_current.y() , 0), freicar::enums::PlannerCommand{HLC_enum}, 15,30); //TODO
//                PublishPlan(plan, 1.0, 0.1, 0.4, 300, "plan_1", tf);
//
//                HLC_bool = false;
//                continue_flag=0;
//                junction_arrived = false;
                //auto plan = freicar::planning::lane_follower::GetPlan(Point3D(p_closest_plan.x(),p_closest_plan.y() , 0), freicar::enums::PlannerCommand{HLC_enum}, 15,30); //TODO
                ///TEST
//                 auto previous_lane_points = map.FindClosestLanePoints(p_closest_plan.x(),
//                                                                       p_closest_plan.y(),
//                                                                       0,
//                                                                       1)[0].first;
//                 const freicar::mapobjects::Lane *previous_lane;
//                 previous_lane = map.FindLaneByUuid(previous_lane_points.GetLaneUuid());

                ///TEST
//                auto plan = freicar::planning::lane_follower::GetPlan(Point3D(p_closest_plan.x(),p_closest_plan.y() , 0), freicar::enums::PlannerCommand{HLC_enum}, 15,25); //TODO
                PublishPlan(plan, 1.0, 0.1, 0.4, 300, "plan_1", tf);
                HLC_bool = false;
                junction_arrived = false;
            }

//             if (overtake.data == true) {
//                 overtake.data = false;
//                 //std::cout << "x postion " << p_current.x() << std::endl;
//                 //std::cout << "current lane " << current_lane->GetUuid().GetUuidValue() << std::endl;
//                 //std::cout << "I'm here or not" << goal_bool.data << std::endl;
//                 //OPPOSITE LANE FROM CAR
//                 auto *opposite_lane = current_lane->GetConnection(freicar::mapobjects::Lane::OPPOSITE);
//                 auto opposite_l_uuid = opposite_lane->GetUuid().GetUuidValue();
//                 std::cout << "opposite lane " << opposite_l_uuid << std::endl;
//                 //float width = opposite_lane->GetWidth();
//                 //auto opposite_points = opposite_lane->GetPoints();
//                 //fake plan
//                 //CREATE A PLAN
//                 auto plan4 = freicar::planning::lane_follower::GetPlan(
//                         Point3D(p_closest.x(), p_closest.y(), p_closest.z()), freicar::enums::STRAIGHT, 10, 30);
//                 std::cout << "postion x  before starting points " << plan4[0].position.x() << std::endl;
//                 //2.5 METERS FROM MY POSITION
//                 Point3D starting_plan = Point3D(plan4[4].position.x(), plan4[4].position.y(),
//                                                 plan4[4].position.z());

//                 auto q_closest = map.FindClosestLanePoints(starting_plan.x(),
//                                                            starting_plan.y(),
//                                                            starting_plan.z(),
//                                                            1)[0].first;
//                 //CURRENT LANE OF THE POINT AT 2.5 METERS AWAY
//                 const freicar::mapobjects::Lane *current_lane2;
//                 current_lane2 = map.FindLaneByUuid(p_closest.GetLaneUuid());
//                 //OPPOSITE LANE OF THAT POINT
//                 auto *opposite_lane2 = current_lane2->GetConnection(freicar::mapobjects::Lane::OPPOSITE);
//                 auto opposite_points = opposite_lane2->GetPoints();

//                 std::cout << starting_plan.x() << std::endl;
//                 Point3D closest_point;
//                 //std::cout<< closest_point.x() << std::endl;
//                 std::cout << opposite_points.size() << std::endl;
//                 float dist = 2.0f;
//                 auto opposite_l_uuid2 = opposite_lane2->GetUuid().GetUuidValue();
//                 std::cout << "uuid other lane: " << opposite_l_uuid2 << std::endl;
//                 std::cout << "dist" << dist << std::endl;
//                 //ITERATE OVER ALL POINTS IN THE OPPOSITE LANE TO GET THE CLOSEST
//                 for (auto &opposite_point : opposite_points) {
//                     std::cout << "dist inside "
//                               << opposite_point.ComputeDistance(starting_plan.x(), starting_plan.y(), starting_plan.z()) << std::endl;
//                     if (opposite_point.ComputeDistance(starting_plan.x(), starting_plan.y(), starting_plan.z()) < dist) {
//                         closest_point = opposite_point;
//                         dist = opposite_point.ComputeDistance(starting_plan.x(), starting_plan.y(), starting_plan.z());
//                         std::cout << "dist inside " << dist << std::endl;

//                     }

//                 }


//                 //std::cout << "closest" << closest_point.x() << std::endl;
//                 //FAKE PLAN TO OVERTAKE
//                 auto planovertake2 = freicar::planning::lane_follower::GetPlan(
//                         Point3D(closest_point.x(), closest_point.y(), closest_point.z()), freicar::enums::STRAIGHT,
//                         2.5, 5);

//                 //SET THE NEW POINT TO THE PLAN
//                 plan4[0].position.SetX(planovertake2[4].position.x());
//                 plan4[0].position.SetY(planovertake2[4].position.y());
//                 plan4[0].position.SetZ(planovertake2[4].position.z());
//                 plan4[1].position.SetX(planovertake2[3].position.x());
//                 plan4[1].position.SetY(planovertake2[3].position.y());
//                 plan4[1].position.SetZ(planovertake2[3].position.z());
//                 plan4[2].position.SetX(planovertake2[2].position.x());
//                 plan4[2].position.SetY(planovertake2[2].position.y());
//                 plan4[2].position.SetZ(planovertake2[2].position.z());
//                 plan4[3].position.SetX(planovertake2[1].position.x());
//                 plan4[3].position.SetY(planovertake2[1].position.y());
//                 plan4[3].position.SetZ(planovertake2[1].position.z());
//                 plan4[4].position.SetX(planovertake2[0].position.x());
//                 plan4[4].position.SetY(planovertake2[0].position.y());
//                 plan4[4].position.SetZ(planovertake2[0].position.z());
//                 //PUBLISH PLAN
//                 PublishPlan(plan4, 1.0, 0.1, 0.4, 300, "plan_1", tf);
//                 //overtaking_done = false;
//             }



            std::this_thread::sleep_for(1s);
            freicar::logic::JunctionAgent::Intent{0};
            freicar::logic::JunctionAgent agent_junction = freicar::logic::JunctionAgent(p_current_row) ;

//             p_observed_agent.current_pose = p_current_row.current_pose;
//             p_observed_agent.current_pose.transform.translation.x = 2.1;
//             p_observed_agent.current_pose.transform.translation.y = 3.26;
//             p_observed_agent.current_pose.transform.translation.z = 0.0f;

//             p_observed_agent.current_pose.transform.rotation.x = 0.0;
//             p_observed_agent.current_pose.transform.rotation.y = 0.0;
//             p_observed_agent.current_pose.transform.rotation.z = 0.0;
//             p_observed_agent.current_pose.transform.rotation.w = 1.0;

//             p_observed_agent.velocity = p_current_row.velocity ;
//             p_observed_agent.velocity.x = 0.0 ;
//             p_observed_agent.velocity.y =  0.0;
//             p_observed_agent.velocity.z =  0.0;

//             p_observed_agent.name = "observed_1";
//             p_observed_agent.lane_offset = 0.0f;

//             auto p_closest_observed = map.FindClosestLanePoints(p_observed_agent.current_pose.transform.translation.x,
//                                                                 p_observed_agent.current_pose.transform.translation.y,
//                                                                 p_observed_agent.current_pose.transform.translation.z,
//                                                                 1)[0].first;
//             std::string current_l_uuid_observed = p_closest_observed.GetLaneUuid();
//             p_observed_agent.lane_uuid = current_l_uuid_observed;

//            std::cout<<"our agent name"<<p_current_row.name<<std::endl;
//            std::cout<<"observed agent name"<<p_observed_agent.name<<std::endl;

            freicar::logic::JunctionAgent observed_agent = freicar::logic::JunctionAgent(p_observed_agent) ;



            std::vector<freicar::logic::JunctionAgent> observed_agents;
            observed_agents.push_back(observed_agent);

            std::vector<freicar::logic::JunctionAgent> agents;
            agents.push_back(agent_junction);

            if (freicar::enums::PlannerCommand{HLC_enum} == 1)
            {
                agent_junction.intent = freicar::logic::JunctionAgent::Intent::GOING_LEFT;
            }
            else if (freicar::enums::PlannerCommand{HLC_enum} == 2){
                agent_junction.intent = freicar::logic::JunctionAgent::Intent::GOING_RIGHT;
            }
            else {
                agent_junction.intent = freicar::logic::JunctionAgent::Intent::GOING_STRAIGHT;
            }
            //observed_agent.intent = freicar::logic::JunctionAgent::Intent::GOING_RIGHT;
            //Dummy agent
            //const freicar_common::FreiCarAgentLocalization& dummy1 =

            // std::cout<<"Junction Agent  =" << agent_junction.IsOnRightHandSideOf(agent_junction) << std::endl;
//            std::cout<<"Junction Agent Offset = "<<freicar::logic::JunctionAgent::IsOnRightHandSideOf(freicar::logic::JunctionAgent(p_current_row))<<std::endl;
//            std::cout<<"Junction Agent  =" << p_current_row.current_pose << p_current_row.velocity << std::endl;
            bool one ;
            bool two;
            std::string three;
            std::tie(one, two, three) = freicar::logic::GetRightOfWay(agent_junction, observed_agents, false, false);
        }
    });


    ROS_INFO("visualizing map @ 0.1 Hz");
    std::thread map_vis_thread([&]() {
        using namespace std::chrono_literals;
        while (ros::ok()) {

            freicar::map::Map::GetInstance().SendAsRVIZMessage(0.01, 0.01, node_handle);
            // ROS_INFO("visualized map");
            std::this_thread::sleep_for(10s);
        }
    });
    // starting waypoint service
    ros::ServiceServer waypoint_service = node_handle->advertiseService("waypoint_service", HandleWayPointRequest);
    std::cout << "waypoint_service active" << std::endl;
    ros::spin();
    std::cout << "\njoining threads ..." << std::endl;
    map_vis_thread.join();

    return 0;
}
