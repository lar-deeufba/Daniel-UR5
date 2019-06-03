#include <place_avg_skill_server/place_avg_skill_server.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "place_avg_skill");
  ros::NodeHandle nh("~");
  std::string skill_name;
  nh.param<std::string>("SkillName", skill_name, "PlaceAvgSkill");
  PlaceAvgSkill place_avg(skill_name);
  ros::spin();

  return 0;
}
