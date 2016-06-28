function drive_forward(inputs::BehaviorInputs)
    run_continuous(inputs.robot.motors.right, 60)
    run_continuous(inputs.robot.motors.left, 60)
end

function turn_right(inputs::BehaviorInputs)
    run_continuous(inputs.robot.motors.right, -70)
    run_continuous(inputs.robot.motors.left, 30)
end

function halt(inputs::BehaviorInputs)
    stop(inputs.robot.motors.right, "brake")
    stop(inputs.robot.motors.left, "brake")
    stop(inputs.robot.head, "coast")
end

function look_right(inputs::BehaviorInputs)
    servo_absolute(inputs.robot.head, Degrees(3 * 80))
end

function look_left(inputs::BehaviorInputs)
    servo_absolute(inputs.robot.head, Degrees(-3 * 80))
end

const mapping_behaviors = Dict(:forward => drive_forward,
                               :turn_right => turn_right,
                               :look_right => look_right,
                               :look_left => look_left,
                               :halt => halt)

function mapping_state_machines(timeout=Second(30))
    driving_machine = StateMachine(:forward, :halt,
        [(:forward, (inputs) -> inputs.sensor_data.ultrasound < 0.25, :turn_right),
         (:turn_right, (inputs) -> inputs.sensor_data.ultrasound > 0.5, :forward),
         (:forward, (inputs) -> inputs.time > timeout, :halt),
         (:turn_right, (inputs) -> inputs.time > timeout, :halt)])

    head_machine = StateMachine(:look_right, :halt,
        [(:look_right, (inputs) -> inputs.sensor_data.head_angle < -pi/4, :look_left),
         (:look_left, (inputs) -> inputs.sensor_data.head_angle > pi/4, :look_right)])

    (driving_machine, head_machine)
end
