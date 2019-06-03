#ifndef INIT_SKILL_SERVER
#define INIT_SKILL_SERVER

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <init_skill_msgs/InitSkillAction.h>

class InitSkill
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<init_skill_msgs::InitSkillAction> as_;
  std::string action_name_;
  init_skill_msgs::InitSkillFeedback feedback_;
  init_skill_msgs::InitSkillResult result_;

public:
  InitSkill(std::string name);
  ~InitSkill(void);
  void executeCB(const init_skill_msgs::InitSkillGoalConstPtr &goal);
  void feedback(float percentage);
  void set_succeeded(std::string outcome = "succeeded");
  void set_aborted(std::string outcome = "aborted");
  bool check_preemption();
};

#endif // INIT_SKILL_SERVER
