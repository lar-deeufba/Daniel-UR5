#include <home_skill_server/home_skill_server.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "home_skill");
  ros::NodeHandle nh("~");
  std::string skill_name;
  nh.param<std::string>("SkillName", skill_name, "HomeSkill");
  HomeSkill home(skill_name);
  ros::spin();

  return 0;
}
