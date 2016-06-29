# MappingRobots: A Simple Julia Lego Mindstorms Demo

This project is a simple demonstration of the [Ev3dev.jl](https://github.com/rdeits/Ev3dev.jl) package, which provides Julia bindings to the `ev3dev` operating system for the Lego Mindstorms EV3. With it, you can run a simple two-wheeled robot that uses its ultrasound sensor to avoid walls and build a map of its environment as it drives.

# Requirements

You will need `Ev3dev.jl` for the `ev3dev` bindings and `StateMachineIterators.jl` for the robot behavior tools. [Ev3dev.jl](https://github.com/rdeits/Ev3dev.jl) has detailed instructions for setting up your EV3 with Julia.

To install all of the Julia packages you'll need, you can run:

```julia
Pkg.add("AffineTransforms")
Pkg.clone("git://github.com/rdeits/Ev3dev.jl.git")
Pkg.clone("git://github.com/rdeits/StateMachineIterators.jl.git")
Pkg.clone("git://github.com/rdeits/MappingRobots.jl.git")
```

# Hardware

This package assumes you've built the two-wheeled mapping robot included in `lego_models/mapping_robot.lxf`. The model can be opened with the free [Lego Digital Designer](http://ldd.lego.com/en-us/). 
