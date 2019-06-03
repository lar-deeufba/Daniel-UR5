#ifndef PICK_PRINTER_SKILL_SERVER
#define PICK_PRINTER_SKILL_SERVER

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pick_printer_skill_msgs/PickPrinterSkillAction.h>

class PickPrinterSkill
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<pick_printer_skill_msgs::PickPrinterSkillAction> as_;
  std::string action_name_;
  pick_printer_skill_msgs::PickPrinterSkillFeedback feedback_;
  pick_printer_skill_msgs::PickPrinterSkillResult result_;

public:
  PickPrinterSkill(std::string name);
  ~PickPrinterSkill(void);
  void executeCB(const pick_printer_skill_msgs::PickPrinterSkillGoalConstPtr &goal);
  void feedback(float percentage);
  void set_succeeded(std::string outcome = "succeeded");
  void set_aborted(std::string outcome = "aborted");
  bool check_preemption();
};

#endif // PICK_PRINTER_SKILL_SERVER
