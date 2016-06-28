using MappingRobots
using Base.Dates

robot = Robot(Brick("/home/pi/Ev3"))
local_map = run_mapping(robot, timeout=Second(30))

