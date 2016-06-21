module MappingRobots

using Ev3
using Base.Dates
# import Ev3: port_name,
#             values,
#             position,
#             position_sp,
#             count_per_rot,
#             speed_sp,
#             speed_regulation,
#             polarity,
#             command,
#             stop,
#             time_sp

using StateMachineIterators
using AffineTransforms

import Base: start, next, done, +

export run_mapping,
       Sides,
       Map

type Sides{T}
    right::T
    left::T
end

start(sides::Sides) = :right
function next(sides::Sides, state)
    if state == :right
        next_state = :left
    else
        next_state = :done
    end
    getfield(sides, state), next_state
end
done(sides::Sides, state) = (state == :done)


type Odometer
    motor::Ev3.Device
    ticks_per_revolution
    meters_per_tick
    last_position
    total_distance
end

function Odometer(motor::Ev3.Device, meters_per_revolution::Real)
    ticks_per_revolution = motor.attr.count_per_rot()
    meters_per_tick = meters_per_revolution / ticks_per_revolution
    current_position = motor.attr.position()
    Odometer(motor, ticks_per_revolution, meters_per_tick, current_position, 0.0)
end

function update!(odo::Odometer)
    current_position = odo.motor.attr.position()
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
    inputs.sensor_data.head_angle = -position(inputs.robot.head) * 12 / 36 * pi / 180
end


# function update_input!(robot::Robot, t, state::State, input::SensorData)
#     input.gyro = -values(robot.sensors.gyro)[1] * pi / 180
#     input.ultrasound = values(robot.sensors.ultrasound)[1] / 100
#     input.total_wheel_distances = Sides(map(update!, robot.sensors.odos)...)
#     input.head_angle = -position(robot.head) * 12 / 36 * pi / 180
# end

function update_state!(inputs::BehaviorInputs)
    angle_change = inputs.sensor_data.gyro - inputs.state.last_orientation
    wheel_distances = [getfield(inputs.sensor_data.total_wheel_distances, field) - getfield(inputs.state.last_wheel_distances, field) for field in [:right, :left]]
    inputs.state.pose *= tformrigid([angle_change, mean(wheel_distances), 0])
    inputs.state.last_wheel_distances = inputs.sensor_data.total_wheel_distances
    inputs.state.last_orientation = inputs.sensor_data.gyro
end

# function update_state!(robot::Robot, t, state::State, input::SensorData)
#     angle_change = input.gyro - state.last_orientation
#     wheel_distances = [getfield(input.total_wheel_distances, field) - getfield(state.last_wheel_distances, field) for field in [:right, :left]]
#     state.pose *= tformrigid([angle_change, mean(wheel_distances), 0])
#     state.last_wheel_distances = input.total_wheel_distances
#     state.last_orientation = input.gyro
# end

type Map
    points::Vector{Tuple{Real, Real}}
    path::Vector{AffineTransform}
end

Map() = Map(Tuple{Real,Real}[], AffineTransform[])

function prep!(robot::Robot)
    # robot.motors.right.attr.speed_regulation("on")
    # robot.motors.left.attr.speed_regulation("on")
    robot.head.attr.speed_sp(130)
    # speed_regulation(robot.motors.right, "on")
    # speed_regulation(robot.motors.left, "on")
    # speed_regulation(robot.head, "on")
    # time_sp(robot.motors.right, 1000)
    # time_sp(robot.motors.left, 1000)
    # speed_sp(robot.head, 130)
end

function shutdown!(robot::Robot)
    map(Ev3.stop, robot.motors)
    stop(robot.head)
end

include("mapping_behaviors.jl")


function run_mapping(robot::Robot; timeout=Second(30), initial_pose=tformeye(2))
    machines = mapping_state_machines(timeout)

    state = State(initial_pose,
                  Sides(0.0, 0.0),
                  -scaled_values(robot.sensors.gyro)[1] * pi / 180)
    sensor_data = SensorData()
    start_time = unix2datetime(time())

    behavior_inputs = BehaviorInputs(robot, Millisecond(0), state, sensor_data)
    local_map = Map()
    prep!(robot)

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
#
#
#     try
#         while !all(current_behaviors .== behaviors.final)
#             t = time() - start_time
#             update_sensors!(behavior_inputs)
#             update_state!(behavior_inputs)
#             if input.ultrasound < 2
#                 new_map_point = state.pose * robot.config.T_origin_to_ultrasound * tformrotate(input.head_angle) * tformtranslate([input.ultrasound, 0])
#                 push!(local_map.points, (new_map_point.offset...))
#             end
#             push!(local_map.path, state.pose)
#             current_behaviors = map(b -> next(b, robot, t, state, input), current_behaviors)
#             map(b -> b.action(robot, t, state, input), current_behaviors)
#         end
#     finally
#         shutdown!(robot)
#     end
#
#     local_map
# end

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

# function construct_remote_robot(hostname)
#     meters_per_revolution = 37.2 * 2.54 / 100 / 5
#     # 37.2 inches in 5 revolutions
#     gyro_port = "in4"
#     us_port = "in1"
#     motor_ports = Sides("outD", "outB")
#     head_port = "outA"
#     distance_between_wheels = 4.5
#     T_origin_to_ultrasound = tformtranslate(0.0254 * [2.0, 0.0])
#     config = RobotConfig(hostname,
#                          meters_per_revolution,
#                          gyro_port,
#                          us_port,
#                          motor_ports,
#                          head_port,
#                          distance_between_wheels,
#                          T_origin_to_ultrasound)
#
#     robot = Robot(config)
#     robot
# end

end
