#include <ros/ros.h>
#include <robot_msgs/imagePosition.h>
#include <robot_msgs/recognitionActionAction.h>
#include <robot_msgs/detectedObject.h>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <robot_msgs/checkObjectInMap.h>
#include <robot_msgs/useMazeFollower.h>
#include <stdlib.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>

typedef actionlib::SimpleActionClient<robot_msgs::recognitionActionAction> Client;

class MoveMaster {

public:
    ros::NodeHandle n;
    MoveMaster() : ac("object_recognition", true) {
        n = ros::NodeHandle();
        ROS_INFO("Waiting for action server to start...");
        ROS_INFO("Action server started, sending goal.");
        img_position_sub = n.subscribe("/object_detection/object_position", 1, &MoveMaster::imageDetectedCallback, this);
        obj_position_sub = n.subscribe("/goal_obj", 15, &MoveMaster::objectCallback, this);
        path_follower_client = n.serviceClient<robot_msgs::useMazeFollower>("/use_path_follower");
        mapping_off_pub = n.advertise<std_msgs::Bool>("/map_done", 1);


        ac.waitForServer();

        PHASE=1;
        if(n.hasParam("PHASE")){
            n.getParam("PHASE",PHASE);
        }

        if (PHASE == 1) {
            check_map_client = n.serviceClient<robot_msgs::checkObjectInMap>("/object_in_map");
            maze_follower_client = n.serviceClient<robot_msgs::useMazeFollower>("/use_maze_follower");
            timer = n.createTimer(ros::Duration(2*60), &MoveMaster::timerCallback, this, true);
            srv_maze.request.go = true;
            (maze_follower_client.call(srv_maze));

        } else if (PHASE == 2) {
            srv_path.request.go = true;
            path_follower_client.call(srv_path);
        } else {

        }
        start_time=ros::Time::now();
    }

    ~MoveMaster() {}
    void checktimeleft(){
        ros::Duration time_gone = ros::Time::now()-start_time;
        if(PHASE==1 && time_gone.sec > 60*5){
            ROS_INFO("TIME IS OVER");
        }
        if(PHASE==2 && time_gone.sec > 60*3){
            ROS_INFO("TIME IS OVER");
        }
    }

    void recognitionAction() {
        robot_msgs::recognitionActionGoal goal;

        ac.sendGoal(goal,
                    Client::SimpleDoneCallback(),
                    Client::SimpleActiveCallback(),
                    Client::SimpleFeedbackCallback());

        bool finished_in_time = ac.waitForResult(ros::Duration(4.0));
        if(ac.getState()==actionlib::SimpleClientGoalState::ABORTED){
            ROS_INFO("Call was Aborted.");
        }
        if (finished_in_time) {
            ROS_INFO("Object identified before time out.");
        } else {
            ROS_INFO("Action did not finish before time out.");
        }
        ac.cancelGoal();
    }

    void imageDetectedCallback(const robot_msgs::imagePosition &msg) {
        if(PHASE==1){
        srv_object.request.point = msg.point;

            if (check_map_client.call(srv_object)) {
                ROS_INFO("Succesfully called checkObjectInMap service.");
                if (!srv_object.response.inMap) {
                    if((ros::Time::now()-laststop).sec < stopfrequency ){
                        ROS_INFO("Allready detected something recently, dont stop");
                        return;
                    }
                    //Send to wallfollower to stop
                    srv_maze.request.go = false;
                    if (maze_follower_client.call(srv_maze)) {
                        ROS_INFO("Succesfully called useMazeFollower service.");
                        recognitionAction();
                        laststop=ros::Time::now();
                      }
                      else
                      {
                        ROS_ERROR("Failed to call turn useMazeFollower service.");
                      }
                    //Send to wallfollower to start
                    srv_maze.request.go = true;
                    if (maze_follower_client.call(srv_maze)) {
                        ROS_INFO("Succesfully called useMazeFollower service.");
                      }
                      else
                      {
                        ROS_ERROR("Failed to call turn useMazeFollower service.");
                      }
                }
              }
              else
              {
                ROS_ERROR("Failed to call turn checkObjectInMap service.");
              }
        } else if (PHASE == 2) {
            return;
        }
    }

    void objectCallback(const robot_msgs::detectedObject &obj_msg)
    {
        object = obj_msg;
        objects.push_back(object);
    }

    void timerCallback(const ros::TimerEvent&) {
        ROS_INFO("Three minutes have past");
        PHASE=3;
        srv_maze.request.go = false;
        if (maze_follower_client.call(srv_maze)) {
            ROS_INFO("Succesfully called useMazeFollower service.");
            callPathFollower();
            map_off.data = true;
            mapping_off_pub.publish(map_off);
        } else {
            ROS_ERROR("Failed to call turn useMazeFollower service.");
        }
    }

    void callPathFollower(){
        srv_path.request.go = true;
        if (path_follower_client.call(srv_path)) {
            ROS_INFO("Succesfully called pathFollower service.");
        } else {
            ROS_ERROR("Failed to call pathFollower service.");
        }
    }


private:
    Client ac;
    ros::Subscriber img_position_sub;
    ros::Publisher mapping_off_pub;
    ros::Subscriber obj_position_sub;
    ros::ServiceClient check_map_client;
    ros::ServiceClient maze_follower_client;
    ros::ServiceClient path_follower_client;
    robot_msgs::checkObjectInMap srv_object;
    robot_msgs::useMazeFollower srv_maze;
    robot_msgs::useMazeFollower srv_path;
    robot_msgs::detectedObject object;
    std_msgs::Bool map_off;
    std::vector<robot_msgs::detectedObject> objects;
    ros::Timer timer;
    ros::Time laststop;
    ros::Time start_time;
    static const float stopfrequency = 1.5;
    int PHASE;

};

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "move_master");

/*    if (argc == 1) {
        ROS_INFO("Too few input arguments. Choose phase 1 or 2.");
        return -1;
    } else if (argc == 2) {
        if (atoi(argv[1]) == 1) {
            ROS_INFO("Starts phase 1");
        } else if (atoi(argv[1]) == 2) {
            ROS_INFO("Starts phase 2");
        } else {
            ROS_INFO("Choice of phase out of range.");
            return -1;
        }
    } else {
        ROS_INFO("Too many input arguments.");
        return -1;
    }*/

    MoveMaster move;
    ros::Rate loop_rate(10.0);

    while(ros::ok()) {
        ros::spinOnce();
        move.checktimeleft();
        loop_rate.sleep();
    }

    return 0;
}
