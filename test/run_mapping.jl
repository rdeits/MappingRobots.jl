using MappingRobots
using Base.Dates

timeout = length(ARGS) > 0 ? Second(parse(Int, ARGS[1])) : Second(30)

robot = Robot(Brick("/home/pi/Ev3"))
local_map = run_mapping(robot, timeout=timeout)

open("scan_points.csv", "w") do f
    writedlm(f, local_map.points, ',')
end

open("path.csv", "w") do f
    writedlm(f, [[t.offset[1] for t in local_map.path] [t.offset[2] for t in local_map.path]], ',')
end
