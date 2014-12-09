#include <ros/ros.h>
#include <robot_msgs/imagePosition.h>
#include <robot_msgs/recognitionActionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_msgs/checkObjectInMap.h>
#include <robot_msgs/useMazeFollower.h>
#include <stdlib.h>
#include <nav_msgs/OccupancyGrid.h>

typedef actionlib::SimpleActionClient<robot_msgs::recognitionActionAction> Client;

class MoveMaster {

public:
    ros::NodeHandle n;
    MoveMaster() : ac("object_recognition", true) {
        n = ros::NodeHandle();
        ROS_INFO("Waiting for action server to start...");
        //ac.waitForServer();
        ROS_INFO("Action server started, sending goal.");
        img_position_sub = n.subscribe("/object_detection/object_position", 1, &MoveMaster::imageDetectedCallback, this);
        check_map_client = n.serviceClient<robot_msgs::checkObjectInMap>("/object_in_map");
        maze_follower_client = n.serviceClient<robot_msgs::useMazeFollower>("/use_maze_follower");
        path_follower_client = n.serviceClient<robot_msgs::useMazeFollower>("/use_path_follower");
        timer = n.createTimer(ros::Duration(2.5*60), &MoveMaster::timerCallback, this, true);
        //cost_map_sub = n.subscribe("/cost_map", 1, &MoveMaster::costMapCallback, this);
    }

    ~MoveMaster() {}

    void recognitionAction() {
        robot_msgs::recognitionActionGoal goal;

        ac.sendGoal(goal,
                    Client::SimpleDoneCallback(),
                    Client::SimpleActiveCallback(),
                    Client::SimpleFeedbackCallback());

        bool finished_in_time = ac.waitForResult(ros::Duration(5.0));
        if (finished_in_time) {
            ROS_INFO("Object identified before time out.");
        } else {
            ROS_INFO("Action did not finish before time out.");
        }
        ac.cancelGoal();
    }

    void imageDetectedCallback(const robot_msgs::imagePosition &msg) {
        srv_object.request.point = msg.point;
        if (check_map_client.call(srv_object)) {
            ROS_INFO("Succesfully called checkObjectInMap service.");
            if (!srv_object.response.inMap) {
                //Send to wallfollower to stop
                srv_maze.request.go = false;
                if (maze_follower_client.call(srv_maze)) {
                    ROS_INFO("Succesfully called useMazeFollower service.");
                    recognitionAction();
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

    }

    void timerCallback(const ros::TimerEvent&) {
        ROS_INFO("Three minutes have past");
        srv_maze.request.go = false;
        if (maze_follower_client.call(srv_maze)) {
            ROS_INFO("Succesfully called useMazeFollower service.");
            srv_path.request.go = true;
            if (path_follower_client.call(srv_path)) {
                ROS_INFO("Succesfully called pathFollower service.");
            } else {
                ROS_ERROR("Failed to call turn pathFollower service.");
            }
        } else {
            ROS_ERROR("Failed to call turn useMazeFollower service.");
        }


    }

    /*void costMapCallback(const nav_msgs::OccupancyGrid &msg) {
        cost_map = msg;
    }*/

private:
    Client ac;
    ros::Subscriber img_position_sub;
    ros::ServiceClient check_map_client;
    ros::ServiceClient maze_follower_client;
    ros::ServiceClient path_follower_client;
    robot_msgs::checkObjectInMap srv_object;
    robot_msgs::useMazeFollower srv_maze;
    robot_msgs::useMazeFollower srv_path;
    ros::Timer timer;
    //ros::Subscriber cost_map_sub;


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
        loop_rate.sleep();
    }

    return 0;
}
