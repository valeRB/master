#include <ros/ros.h>
#include <robot_msgs/imagePosition.h>
#include <robot_msgs/recognitionActionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_msgs/checkObjectInMap.h>
#include <robot_msgs/useMazeFollower.h>

typedef actionlib::SimpleActionClient<robot_msgs::recognitionActionAction> Client;

class MoveMaster {

public:
    ros::NodeHandle n;
    MoveMaster() : ac("object_recognition", true) {
        n = ros::NodeHandle();
        ROS_INFO("Waiting for action server to start...");
        //      nb  ac.waitForServer();
        ROS_INFO("Action server started, sending goal.");
        img_position_sub = n.subscribe("/object_recognition/object_position", 1, &MoveMaster::imageDetectedCallback, this);
        check_map_client = n.serviceClient<robot_msgs::checkObjectInMap>("/object_in_map");
        maze_follower_client = n.serviceClient<robot_msgs::useMazeFollower>("/use_maze_follower");
    }

    ~MoveMaster() {}

    void recognitionAction() {
        robot_msgs::recognitionActionGoal goal;

        ac.sendGoal(goal,
                    Client::SimpleDoneCallback(),
                    Client::SimpleActiveCallback(),
                    Client::SimpleFeedbackCallback());

        bool finished_in_time = ac.waitForResult(ros::Duration(10.0));
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

private:
    Client ac;
    ros::Subscriber img_position_sub;
    ros::ServiceClient check_map_client;
    ros::ServiceClient maze_follower_client;
    robot_msgs::checkObjectInMap srv_object;
    robot_msgs::useMazeFollower srv_maze;


};

int main(int argc, char **argv) {

    ros::init(argc, argv, "move_master");

    MoveMaster move;

    ros::Rate loop_rate(10.0);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
