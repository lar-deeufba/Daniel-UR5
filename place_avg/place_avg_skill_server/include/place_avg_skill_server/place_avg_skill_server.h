#ifndef PLACE_AVG_SKILL_SERVER
#define PLACE_AVG_SKILL_SERVER

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <place_avg_skill_msgs/PlaceAvgSkillAction.h>

class PlaceAvgSkill
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<place_avg_skill_msgs::PlaceAvgSkillAction> as_;
  std::string action_name_;
  place_avg_skill_msgs::PlaceAvgSkillFeedback feedback_;
  place_avg_skill_msgs::PlaceAvgSkillResult result_;

public:
  PlaceAvgSkill(std::string name);
  ~PlaceAvgSkill(void);
  void executeCB(const place_avg_skill_msgs::PlaceAvgSkillGoalConstPtr &goal);
  void feedback(float percentage);
  void set_succeeded(std::string outcome = "succeeded");
  void set_aborted(std::string outcome = "aborted");
  bool check_preemption();
};

#endif // PLACE_AVG_SKILL_SERVER
