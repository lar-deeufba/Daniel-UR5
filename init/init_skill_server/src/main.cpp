#include <init_skill_server/init_skill_server.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "init_skill");
  ros::NodeHandle nh("~");
  std::string skill_name;
  nh.param<std::string>("SkillName", skill_name, "InitSkill");
  InitSkill init(skill_name);
  ros::spin();

  return 0;
}
