use sp_domain::*;

pub struct UrRobotResource {
    pub path: SPPath,
    pub last_visited_frame: SPPath,
    pub last_visited_with_tcp: SPPath,
    pub acceleration: SPPath,
    trigger: SPPath,  // Command. Trigger a robot motion
    command: SPPath,
    real_velocity: SPPath,
    goal_feature_name: SPPath,
    tcp_name: SPPath,
    pub current_state: SPPath,  // Measured. feedback from robot
    success: SPPath,
    done: SPPath,   // Measured (changed by runner transitions). Motion completed
    error: SPPath,  // Measured. Error from action
    action_state: SPPath,
    pub initial_state: SPState,
}

// I decided to move these to the resource becuase you may
// want more or less details about the action state.
//
// For the UR, the action simplified into the following state machine:
//
// !trigger && !done && !error -> trigger (command)
// trigger && !done && !error -> done  (effect)
// trigger && !done && !error -> error (effect really only needed for model checking. perhaps should be a special type of effect)
// trigger && done && !error -> !trigger (auto) (we finish)
// !trigger && (done || error) -> !done, !error (reset)
//

impl UrRobotResource {
    pub fn new(resource: &mut Resource, mut frame_domain: Vec<SPValue>, mut tool_frame_domain: Vec<SPValue>) -> UrRobotResource {
        let name = resource.path().leaf();
        let trigger = Variable::new_boolean("trigger", VariableType::Command);
        let trigger = resource.add_variable(trigger);

        let command= resource.add_variable(Variable::new(
            "request/command",VariableType::Runner,SPValueType::String,vec!(),
        ));

        let acceleration= resource.add_variable(Variable::new(
            "request/acceleration",VariableType::Runner,SPValueType::Float32,vec!(0.1.to_spvalue()),
        ));
        let real_velocity= resource.add_variable(Variable::new(
            "request/velocity",VariableType::Runner,SPValueType::Float32,vec!(0.1.to_spvalue()),
        ));
        let goal_feature_name= resource.add_variable(Variable::new(
            "request/goal_feature_name",VariableType::Command,SPValueType::String, frame_domain.clone(),
        ));
        let tcp_name= resource.add_variable(Variable::new(
            "request/tcp_name",VariableType::Command,SPValueType::String,tool_frame_domain.clone(),
        ));

        let current_state= resource.add_variable(Variable::new(
            "feedback/current_state", VariableType::Runner, SPValueType::String, vec!(),
        ));

        let success= resource.add_variable(Variable::new_boolean(
            "reply/success", VariableType::Runner
        ));

        let action_state = resource.setup_ros_action(
            "URScriptControl",
            "/ur_script_controller",
            "ur_tools_msgs/action/URScriptControl",
            p!(p: trigger),
            // goal variables
            &[
                MessageVariable::new(&command, "command"),
                MessageVariable::new(&acceleration, "acceleration"),
                MessageVariable::new(&real_velocity, "velocity"),
                MessageVariable::new(&goal_feature_name, "goal_feature_name"),
                MessageVariable::new(&tcp_name, "tcp_name"),
            ],
            &[
                // feedback.
                MessageVariable::new(&current_state, "current_state"),
            ],
            &[
                // result
                MessageVariable::new(&success, "success"),
            ],
        );


        let done = resource.add_variable(Variable::new_boolean(
            "measured/done", VariableType::Measured
        ));
        let error = resource.add_variable(Variable::new_boolean(
            "measured/error", VariableType::Measured
        ));

        // the trigger is set and reset below where we create the motions.

// trigger && !done && !error -> done  (effect)
// trigger && !done && !error -> error (effect really only needed for model checking. perhaps should be a special type of effect)
// !trigger && (done || error) -> !done, !error (reset)

        // sets done to true when we succeed.
        resource.add_transition(
            Transition::new(
                &format!("{}_runner_finish_ok", name),
                p!([!p: done] && [!p: error] && [p: trigger] && [p: action_state == "succeeded"]),
                vec![ a!( p: done)],
                vec![],
                TransitionType::Runner
            )
        );
        // formal representation, request -> done
        resource.add_transition(
            Transition::new(
                &format!("{}_finish_ok", name),
                p!([!p: done] && [!p: error] && [p: trigger]),
                vec![ a!( p: done)],
                vec![],
                TransitionType::Effect
            )
        );

        // sets error to true when we fail.
        resource.add_transition(
            Transition::new(
                &format!("{}_runner_finish_error", name),
                p!([!p: done] && [!p: error] && [p: trigger] &&
                   [[p: action_state == "timeout"] ||
                    [p: action_state == "aborted"] ||
                    [p: action_state == "canceled"]]),
                vec![ a!( p: error)],
                vec![],
                TransitionType::Runner
            )
        );
        // formal representation, request -> error
        // this is not really needed for control, but useful for verification.
        resource.add_transition(
            Transition::new(
                &format!("{}_finish_error", name),
                p!([!p: done] && [!p: error] && [p: trigger]),
                vec![ a!( p: error)],
                vec![],
                TransitionType::Effect
            )
        );

        // resetting the action (going back to "ok") is done
        // automatically when trigger is taken down.
        resource.add_transition(
            Transition::new(
                &format!("{}_runner_reset", name),
                p!([!p: trigger] && [p: action_state == "ok"] && [[p: done] || [p: error]]),
                vec![a!( !p: done), a!( !p: error)],
                vec![],
                TransitionType::Runner
            )
        );
        // formal representation of reset
        resource.add_transition(
            Transition::new(
                &format!("{}_reset", name),
                p!([!p: trigger] && [[p: done] || [p: error]]),
                vec![a!( !p: done), a!( !p: error)],
                vec![],
                TransitionType::Effect
            )
        );

        frame_domain.insert(0, "unknown".to_spvalue());
        tool_frame_domain.insert(0, "unknown".to_spvalue());

        let last_visited_frame = resource.add_variable(Variable::new(
            "estimated/last_visited_frame",VariableType::Estimated,
            SPValueType::String, frame_domain.clone(),
        ));

        let last_visited_with_tcp = resource.add_variable(Variable::new(
            "estimated/last_visited_with_tcp",
            VariableType::Estimated,SPValueType::String, tool_frame_domain.clone(),
        ));

        let initial_state = SPState::new_from_values(
            &[
                (goal_feature_name.clone(), frame_domain[1].clone()),
                (tcp_name.clone(), tool_frame_domain[1].clone()),
                (last_visited_frame.clone(), frame_domain[0].clone()),
                (last_visited_with_tcp.clone(), tool_frame_domain[0].clone()),
                (trigger.clone(), false.to_spvalue()),
                (done.clone(), false.to_spvalue()),
                (error.clone(), false.to_spvalue()),
            ]
        );

        return UrRobotResource {
            path: resource.path().clone(),
            last_visited_frame,
            last_visited_with_tcp,
            acceleration,
            trigger,
            command,
            real_velocity,
            goal_feature_name,
            tcp_name,
            current_state,
            success,
            done,
            error,
            action_state,
            initial_state,
        }
    }

    pub fn run_transition(
        &self,
        model: &mut Model,
        guard: Predicate,
        tcp_frame: &str,
        goal_frame: &str,
        command: &str,
        velocity_scale: f32, // does not work yet
        mut action_when_done: Vec<Action>,
        mut action_when_error: Vec<Action>,
    ) {
        let r = model.get_resource(&self.path);
        let c = &self.command;
        let done = &self.done;
        let error = &self.error;
        let tcp_name = &self.tcp_name;
        let goal_feature_name = &self.goal_feature_name;
        let real_velocity = &self.real_velocity;
        let acceleration = &self.acceleration;
        let trigger = &self.trigger;
        let last_visited_frame = &self.last_visited_frame;
        let last_visited_with_tcp = &self.last_visited_with_tcp;
        let new_guard = p!([!p: trigger] && [!p:done] && [!p:error] && [pp: guard]);
        r.add_transition(Transition::new(
            &format!("{}_{}_to_{}", &r.path().leaf(), tcp_frame, goal_frame),
            new_guard,
            vec![ // formal model cares about these
                a!(p:tcp_name = tcp_frame),
                a!(p:goal_feature_name = goal_frame),
                a!(p:trigger),
            ],
            vec![  // formal model dont care about these
                a!(p:c = command),
                a!(p:real_velocity = 0.1),
                a!(p:acceleration = 0.1),
                // a!(p:real_velocity <- p:self.velocity), // Add multiplication actions before this works. Or send velocity and scaling to driver
            ],
            TransitionType::Controlled));

        let guard_done = p!([p: trigger] && [p:done] &&
                            [p: tcp_name == tcp_frame] &&
                            [p: goal_feature_name == goal_frame]);
        action_when_done.push(a!(!p:trigger)); // reset
        action_when_done.push(a!(p:last_visited_frame = goal_frame)); // save visited
        action_when_done.push(a!(p:last_visited_with_tcp = tcp_frame)); // save visited

        r.add_transition(Transition::new(
            &format!("{}_{}_to_{}_done", &r.path().leaf(), tcp_frame, goal_frame),
            guard_done,
            action_when_done,
            vec![],
            TransitionType::Auto));

        // handle errors. perhaps this transition should be runner by default?
        let guard_error = p!([p: trigger] && [p:error] &&
                             [p: tcp_name == tcp_frame] &&
                             [p: goal_feature_name == goal_frame]);
        action_when_error.push(a!(!p:trigger)); // reset
        action_when_error.push(a!(p:last_visited_frame = "unknown")); // reset visited
        action_when_error.push(a!(p:last_visited_with_tcp = "unknown")); // reset visited

        r.add_transition(Transition::new(
            &format!("{}_{}_to_{}_error", &r.path().leaf(), tcp_frame, goal_frame),
            guard_error,
            action_when_error,
            vec![],
            TransitionType::Controlled));

    }


}
