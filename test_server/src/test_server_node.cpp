#include <ros/ros.h>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include <test_server/testAction.h>


class testAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<test_server::testAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  test_server::testFeedback feedback_;
  test_server::testResult result_;

public:

  testAction(std::string name) :
    as_(nh_, name, boost::bind(&testAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~testAction(void)
  {
  }

  void executeCB(const test_server::testGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1); //rate of the action servor
    bool success = true;

    // start executing the action
    for(int i=1; i<=5; i++) //this will most likely be while !done in the behaviors
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok()) //Important check so the server can be stopped while acting
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.progress = i;
      ROS_INFO("%i: ", i);
      as_.publishFeedback(feedback_); //option to give feedback on how far it has gone
      r.sleep();
    }

    if(success)
    {
      //result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_); //result given to the done callback in the client
    }
  }


};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_server"); //!!!VERY IMPORTANT this is the name the client connects to

    std::string server_name;
    ros::NodeHandle n("~");
    n.param<std::string>("node_name", server_name, "not_loaded");

    std::cout << ros::this_node::getName() << std::endl;
    std::cout << server_name << std::endl;
    //testAction server(ros::this_node::getName());
    testAction server(server_name);
    ROS_INFO("server started");
    ros::spin();

    return 0;
}
