# Daniel-UR5
These are the four UR5 Skills used to pick and places objects on a 3D printer while the arm is mounted on the Husky mobile robot.
The skills works as follow:

1. Init Skill: Initialize the arm and the gripper, leaves the home position and move to a pre grasp position. End with failure if can't reach the pre grasp position or collide with anything in the environment.
2. Pick Printer Skill: Pick the object from the printer. End with failure if can't pick the object from the printer, can't find the printer, can't reach the object or collide with anything in the environment. If grasps the object, end with success.
3. Place Avg Skill: Places the object on the AVG based on a april tag position. End with failure if can't reach the object, collideswith anything in the environment or the gripper doesn't open. End with success if release the object correctly. 
4. Home Skill: Move to the home position. End with failure if collide with anything in the environment.
