#include <ros/ros.h>
#include <vector>
#include "check_interest_points/StartMission.h"
#include "check_interest_points/DebugMSG.h"
#include "waypoint_following/WaypointRegistration.h"
#include "waypoint_following/DebugMSG.h"
#include "waypoint_following/followingStrategyBaseClass.h"
#include "geometry_msgs/Point.h"
#include <iostream>
#include <ctime>
#include <fstream>
#include <string>
#include "std_msgs/String.h"
#include <jsoncpp/json/json.h>
#include <signal.h>

struct Point {
    float x;
    float y;
    float z;
};

XmlRpc::XmlRpcValue list;
ros::ServiceClient client;
//check_interest_points::StartMission srv;
waypoint_following::WaypointRegistration srv;
std::ofstream debug_file;

int prevID = -1;
int prevMsgID = -1;

void upCallback(const std_msgs::String::ConstPtr& msg){
    ROS_WARN("%s", msg->data.c_str());

    int id = -1;
    char flag;
    int msgID = -1;

    Json::Value root;
    Json::Reader reader;

    bool parsingSuccessful = reader.parse(msg->data.c_str(), root); // parse process
    if (!parsingSuccessful)
    {
        ROS_WARN("Failed to parse");
    } else {
        std::string test;
        if(root.isObject()){
            test = root["nID"].asString();
            sscanf(test.c_str(), "%d", &id);
            test = root["f"].asString();
            sscanf(test.c_str(), "%c", &flag);
            test = root["msgID"].asString();
            sscanf(test.c_str(), "%d", &msgID);
        }else{
            id = -1;
        }

        ROS_INFO("%s", test.c_str());
        

        if ((id != -1) && (flag == 'u')){
            if((id != prevID) || ((id == prevID) && (msgID != prevMsgID))) {

                int idx = -1;
                for (int i=0; i<list.size(); i++){
                    if (int(list[i]["id"]) == id){
                        idx = i;
                        break;
                    }
                }

                XmlRpc::XmlRpcValue sublist = list[idx];

                XmlRpc::XmlRpcValue location;
                location = sublist["location"];
                ROS_ASSERT(location.getType() == XmlRpc::XmlRpcValue::TypeArray);

                XmlRpc::XmlRpcValue loc = location[0];
                int x = loc["x"];
                int y = loc["y"];


                // send coordiantes to some service as a mission update
                ROS_WARN("(%d,%d)", x, y);

                std::vector<std::vector<waypoint_following::WaypointMSG> > test_list;
                std::vector<waypoint_following::WaypointMSG> aux_test;
                waypoint_following::WaypointMSG aux;
                aux.x=y;aux.y=x;aux.z=-1.6;aux.frame="ned";aux.inspectTime=3;
                aux_test = {aux};
                test_list.push_back(aux_test);

                using namespace std;
                srv.request.waypoints = test_list[0];
                if (client.call(srv)){
                    ROS_INFO("Response: %s", srv.response.success ? "Success" : "Error");
                } else {
                    ROS_ERROR("4 - Failed to call service start_mission");
                }

                prevID = id;
                prevMsgID = msgID;
            }
        }else{
            ROS_WARN("Error while receiving msg or msg not wanted!");
        }
    }
}

void debugCallback(const waypoint_following::DebugMSG::ConstPtr& debug_msg)
{
  debug_file << std::to_string(debug_msg->t) << "," << std::to_string(debug_msg->pos[0]) << "," << std::to_string(debug_msg->pos[1]) << "," << std::to_string(debug_msg->pos_ref[0]) << "," << std::to_string(debug_msg->pos_ref[1]) << "," << std::to_string(debug_msg->v[0]) << "," << std::to_string(debug_msg->v[1] ) << "," << std::to_string(debug_msg->v_ref[0]) << "," << std::to_string(debug_msg->v_ref[1]) << "\n";
  //ROS_WARN("%f %f %f %f %f", debug_msg->t, debug_msg->pos[0], debug_msg->pos[1], debug_msg->pos_ref[0], debug_msg->pos_ref[1]);
}

void mySigintHandler(int sig){
    debug_file.close();
    ros::shutdown();
}

int main(int argc, char ** argv){

    time_t now = time(0);
    tm *ltm = localtime(&now);

    ros::init(argc, argv,"mission_planner_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;

    signal(SIGINT, mySigintHandler);

    n.getParam("wsn_config/nodes", list);
    ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    //client = n.serviceClient<check_interest_points::StartMission>("topics/services/start_mission");

    client = n.serviceClient<waypoint_following::WaypointRegistration>("services/waypoint_registration");
    

    ros::Subscriber sub = n.subscribe("debug", 1000, debugCallback);
    ros::Subscriber wsn_up = n.subscribe("/gateway/ul", 1000, upCallback);

    std::string file_name = "/home/xavier/ros_debug_files/" + 
                            std::to_string(ltm->tm_mday) + "_" + std::to_string(ltm->tm_mon + 1) + "_" + 
                            std::to_string(ltm->tm_hour) + "_" + std::to_string(ltm->tm_min) + "_" +
                            "debug_mission" + ".csv";
    debug_file.open(file_name);

    ros::spin();
    return 0;
}
