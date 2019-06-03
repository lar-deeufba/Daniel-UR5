#ifndef HOME_SKILL_SERVER
#define HOME_SKILL_SERVER

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <home_skill_msgs/HomeSkillAction.h>

class HomeSkill
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<home_skill_msgs::HomeSkillAction> as_;
  std::string action_name_;
  home_skill_msgs::HomeSkillFeedback feedback_;
  home_skill_msgs::HomeSkillResult result_;

public:
  HomeSkill(std::string name);
  ~HomeSkill(void);
  void executeCB(const home_skill_msgs::HomeSkillGoalConstPtr &goal);
  void feedback(float percentage);
  void set_succeeded(std::string outcome = "succeeded");
  void set_aborted(std::string outcome = "aborted");
  bool check_preemption();
};

#endif // HOME_SKILL_SERVER
