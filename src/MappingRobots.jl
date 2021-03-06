__precompile__()

module MappingRobots

using Ev3dev
using Base.Dates
using StateMachineIterators
using AffineTransforms

import Base: start, next, done, +

export run_mapping,
       Sides,
       Map,
       Robot,
       Brick

type Sides{T}
    right::T
    left::T
end

start(sides::Sides) = :right
next(sides::Sides, state::Symbol) = (state == :right) ? (sides.right, :left) : (sides.left, :done)
done(sides::Sides, state::Symbol) = (state == :done)

type Odometer
    motor::Ev3dev.Device
    ticks_per_revolution
    meters_per_tick
    last_position
    total_distance
end

function Odometer(motor::Ev3dev.Device, meters_per_revolution::Real)
    ticks_per_revolution = motor.io.count_per_rot()
    meters_per_tick = meters_per_revolution / ticks_per_revolution
    current_position = motor.io.position()
    Odometer(motor, ticks_per_revolution, meters_per_tick, current_position, 0.0)
end

function update!(odo::Odometer)
    current_position = odo.motor.io.position()
    delta = current_position - odo.last_position
    odo.last_position = current_position
    new_distance = delta * odo.meters_per_tick
    odo.total_distance += new_distance
    odo.total_distance
end

type State
    pose::AffineTransform
    last_wheel_distances::Sides
    last_orientation::Number
end

type SensorData
    gyro::Number
    ultrasound::Number
    total_wheel_distances::Sides
    head_angle::Number

    SensorData() = new()
end

type MappingSensors
    gyro::GyroSensor
    ultrasound::UltrasoundSensor
    odos::Sides{Odometer}
end

type RobotConfig
    brick::Brick
    meters_per_revolution::Number
    motor_port_names::Sides
    distance_between_wheels::Number
    T_origin_to_ultrasound::AffineTransform
end

type Robot
    config::RobotConfig
    motors::Sides{LargeMotor}
    head::MediumMotor
    sensors::MappingSensors
end

function Robot(config::RobotConfig)
    motors = Sides(find_device_at_address(LargeMotor, config.brick, config.motor_port_names.right),
                   find_device_at_address(LargeMotor, config.brick, config.motor_port_names.left))
    head = find_devices(MediumMotor, config.brick)[1]
    odos = Sides(Odometer(motors.right, config.meters_per_revolution), Odometer(motors.left, config.meters_per_revolution))
    gyro = find_devices(GyroSensor, config.brick)[1]
    ultrasound = find_devices(UltrasoundSensor, config.brick)[1]
    sensors = MappingSensors(gyro, ultrasound, odos)
    Robot(config, motors, head, sensors)
end

type BehaviorInputs
    robot::Robot
    time::TimePeriod
    state::State
    sensor_data::SensorData
end

function update_sensors!(inputs::BehaviorInputs)
    inputs.sensor_data.gyro = -scaled_values(inputs.robot.sensors.gyro)[1] * pi / 180
    inputs.sensor_data.ultrasound = scaled_values(inputs.robot.sensors.ultrasound)[1] / 100
    inputs.sensor_data.total_wheel_distances = Sides(map(update!, inputs.robot.sensors.odos)...)
    inputs.sensor_data.head_angle = -inputs.robot.head.io.position() * 12 / 36 * pi / 180
end

function update_state!(inputs::BehaviorInputs)
    angle_change = inputs.sensor_data.gyro - inputs.state.last_orientation
    wheel_distances = [getfield(inputs.sensor_data.total_wheel_distances, field) - getfield(inputs.state.last_wheel_distances, field) for field in [:right, :left]]
    inputs.state.pose *= tformrigid([angle_change, mean(wheel_distances), 0])
    inputs.state.last_wheel_distances = inputs.sensor_data.total_wheel_distances
    inputs.state.last_orientation = inputs.sensor_data.gyro
end

type Map
    points::Vector{Tuple{Real, Real}}
    path::Vector{AffineTransform}
end

Map() = Map(Tuple{Real,Real}[], AffineTransform[])

function prep!(robot::Robot)
    robot.head.io.speed_sp(130)
    map(stop, robot.motors)
    stop(robot.head)
	map(m -> m.io.position(0), robot.motors)
end

function shutdown!(robot::Robot)
    map(stop, robot.motors)
    stop(robot.head)
end

include("mapping_behaviors.jl")


function run_mapping(robot::Robot; timeout=Second(30), initial_pose=tformeye(2))
    machines = mapping_state_machines(timeout)

    state = State(initial_pose,
                  Sides(0.0, 0.0),
                  -scaled_values(robot.sensors.gyro)[1] * pi / 180)
    start_time = unix2datetime(time())

    behavior_inputs = BehaviorInputs(robot, Millisecond(0), state, SensorData())
    prep!(robot)
    update_sensors!(behavior_inputs)
    update_state!(behavior_inputs)
    local_map = Map()

    iters = [StateMachineIterator(machine, () -> behavior_inputs) for machine in machines]
    try
        for behaviors in zip(iters...)
            behavior_inputs.time = unix2datetime(time()) - start_time
            update_sensors!(behavior_inputs)
            update_state!(behavior_inputs)
            if behavior_inputs.sensor_data.ultrasound < 2
                new_map_point = behavior_inputs.state.pose * robot.config.T_origin_to_ultrasound * tformrotate(behavior_inputs.sensor_data.head_angle) * tformtranslate([behavior_inputs.sensor_data.ultrasound, 0])
                push!(local_map.points, (new_map_point.offset...))
            end
            push!(local_map.path, behavior_inputs.state.pose)
            for behavior in behaviors
                mapping_behaviors[behavior](behavior_inputs)
            end
        end
    finally
        shutdown!(robot)
    end
    local_map
end

function Robot(brick::Brick)
    meters_per_revolution = 37.2 * 2.54 / 100 / 5 # 37.2 inches in 5 revolutions
    motor_ports = Sides("outD", "outB")
    distance_between_wheels = 4.5
    T_origin_to_ultrasound = tformtranslate(0.0254 * [2.0, 0.0])
    Robot(RobotConfig(brick,
                      meters_per_revolution,
                      motor_ports,
                      distance_between_wheels,
                      T_origin_to_ultrasound))
end

end
