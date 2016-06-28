using MappingRobots
using Base.Dates
using Gadfly

timeout = length(ARGS) > 0 ? Second(parse(Int, ARGS[1])) : Second(30)

robot = Robot(Brick("/home/pi/Ev3"))
local_map = run_mapping(robot, timeout=timeout)

plt = plot(layer(x=[p[1] for p in local_map.points],
                 y=[p[2] for p in local_map.points], Geom.Point),
           layer(x=[t.offset[1] for t in local_map.path],
                 y=]t.offset[2] for t in local_map.path], Geom.Point, Geom.line))
draw(SVG("map.svg", 6inch, 6inch), plt)
