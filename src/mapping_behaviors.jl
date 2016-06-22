
# function drive_forward(robot, t, state, input)
function drive_forward(inputs::BehaviorInputs)
    run_continuous(inputs.robot.motors.right, 60)
    run_continuous(inputs.robot.motors.left, 60)
    # speed_sp(robot.motors.right, 60)
    # speed_sp(robot.motors.left, 60)
    # command(robot.motors.right, "run-timed")
    # command(robot.motors.left, "run-timed")
end

function turn_right(inputs::BehaviorInputs)
# function turn_right(robot, t, state, input)
    run_continuous(inputs.robot.motors.right, -70)
    run_continuous(inputs.robot.motors.left, 30)
    # speed_sp(robot.motors.right, -70)
    # speed_sp(robot.motors.left, 30)
    # command(robot.motors.right, "run-timed")
    # command(robot.motors.left, "run-timed")
end

# function stop(robot, t, state, input)
function stop(inputs::BehaviorInputs)
    Ev3.stop(inputs.robot.motors.right, "brake")
    Ev3.stop(inputs.robot.motors.left, "brake")
    Ev3.stop(inputs.robot.head, "coast")
end

# function look_right(robot, t, state, input)
function look_right(inputs::BehaviorInputs)
    servo_absolute(inputs.robot.head, Degrees(3 * 80))
    # position_sp(robot.head, 80 * 3)
    # command(robot.head, "run-to-abs-pos")
end

# function look_left(robot, t, state, input)
function look_left(inputs::BehaviorInputs)
    servo_absolute(inputs.robot.head, Degrees(-3 * 80))
    # position_sp(robot.head, -80 * 3)
    # command(robot.head, "run-to-abs-pos")
end

const mapping_behaviors = Dict(:forward => drive_forward,
                               :turn_right => turn_right,
                               :look_right => look_right,
                               :look_left => look_left,
                               :stop => stop,
			       :done => (x) -> nothing)

function mapping_state_machines(timeout=Second(30))
    driving_machine = StateMachine(:forward, :done,
        [(:forward, (inputs) -> inputs.sensor_data.ultrasound < 0.25, :turn_right),
         (:turn_right, (inputs) -> inputs.sensor_data.ultrasound > 0.5, :forward),
         (:forward, (inputs) -> inputs.time > timeout, :stop),
         (:turn_right, (inputs) -> inputs.time > timeout, :stop),
         (:stop, (inputs) -> true, :done)])

    head_machine = StateMachine(:look_right, :done,
        [(:look_right, (inputs) -> inputs.sensor_data.head_angle < -pi/4, :look_left),
         (:look_left, (inputs) -> inputs.sensor_data.head_angle > pi/4, :look_right)])

    (driving_machine, head_machine)
end

#
#     FORWARD = Behavior(drive_forward)
#     TURN_RIGHT = Behavior(turn_right)
#     STOP = Behavior(stop)
#     DONE = Behavior(stop)
#     LOOK_RIGHT = Behavior(look_right)
#     LOOK_LEFT = Behavior(look_left)
#
#     add_transition!(FORWARD,
#                     Transition((robot, t, state, input) -> input.ultrasound < 0.25,
#                                TURN_RIGHT))
#     add_transition!(TURN_RIGHT,
#                     Transition((robot, t, state, input) -> input.ultrasound > 0.5,
#                                FORWARD))
#     add_transition!(LOOK_RIGHT,
#                     Transition((robot, t, state, input) -> input.head_angle < -pi/4,
#                                LOOK_LEFT))
#     add_transition!(LOOK_LEFT,
#                     Transition((robot, t, state, input) -> input.head_angle > pi/4,
#                                LOOK_RIGHT))
#
#     for behavior in [FORWARD, TURN_RIGHT, LOOK_RIGHT, LOOK_LEFT]
#         add_transition!(behavior,
#                         Transition((robot, t, state, input) -> t > timeout,
#                                    STOP))
#     end
#     add_transition!(STOP,
#                     Transition((robot, t, state, input) -> true,
#                                DONE))
#
#     [FORWARD, TURN_RIGHT, STOP, DONE, LOOK_RIGHT, LOOK_LEFT]
#     BehaviorIterator([FORWARD, TURN_RIGHT, STOP, DONE, LOOK_RIGHT, LOOK_LEFT], [FORWARD, LOOK_RIGHT], DONE, () -> behavior_inputs)
# end
