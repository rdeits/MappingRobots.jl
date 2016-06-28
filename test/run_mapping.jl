using MappingRobots
using Base.Dates

timeout = ARGS ? Second(parse(Int, ARGS[1])) : Second(30)

robot = Robot(Brick("/home/pi/Ev3"))
local_map = run_mapping(robot, timeout=timeout)
