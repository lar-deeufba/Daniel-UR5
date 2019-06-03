#include <pick_printer_skill_server/pick_printer_skill_server.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_printer_skill");
  ros::NodeHandle nh("~");
  std::string skill_name;
  nh.param<std::string>("SkillName", skill_name, "PickPrinterSkill");
  PickPrinterSkill pick_printer(skill_name);
  ros::spin();

  return 0;
}
