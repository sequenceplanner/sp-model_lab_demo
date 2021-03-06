use sp_domain::*;

pub struct UrRobotResource {
    pub path: SPPath,
    pub last_visited_frame: SPPath,
    pub last_visited_with_tcp: SPPath,
    pub acceleration: SPPath,
    pub trigger: SPPath,  // Command. Trigger a robot motion
    pub command: SPPath,
    pub velocity: SPPath,
    pub goal_feature_name: SPPath,
    pub tcp_name: SPPath,
    pub current_state: SPPath,  // Measured. feedback from robot
    pub success: SPPath,
    pub done: SPPath,   // Measured (changed by runner transitions). Motion completed
    pub error: SPPath,  // Measured. Error from action
    pub action_state: SPPath,
    pub initial_state: SPState,
    t_index: i32,
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
    pub fn new(model: &mut Model, path: &SPPath, mut frame_domain: Vec<SPValue>, tool_frame_domain: Vec<SPValue>) -> UrRobotResource {
        frame_domain.insert(0, "unknown".to_spvalue());

        // add robot state to high level model
        let last_visited_frame = model.add_product_domain(
            "ur_last_visited_frame", &frame_domain);

        let last_visited_with_tcp = model.add_product_domain(
            "ur_last_visited_with_tcp", &tool_frame_domain);

        let resource = model.get_resource(path);

        let name = resource.path().leaf();
        let trigger = Variable::new_boolean("trigger", VariableType::Command);
        let trigger = resource.add_variable(trigger);

        let command= resource.add_variable(Variable::new(
            "request/command",VariableType::Runner,SPValueType::String,vec!(),
        ));

        let acceleration_scaling= resource.add_variable(Variable::new(
            "request/acceleration_scaling",VariableType::Runner,SPValueType::Float32,vec!(0.1.to_spvalue()),
        ));
        let velocity_scaling= resource.add_variable(Variable::new(
            "request/velocity_scaling",VariableType::Runner,SPValueType::Float32,vec!(0.1.to_spvalue()),
        ));

        let acceleration= resource.add_variable(Variable::new(
            "request/acceleration",VariableType::Runner,SPValueType::Float32,vec!(0.1.to_spvalue()),
        ));
        let velocity= resource.add_variable(Variable::new(
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
            "URControl",
            "/ur_control",
            "ur_controller_msgs/action/URControl",
            p!(trigger),
            // goal variables
            &[
                MessageVariable::new(&command, "command"),
                MessageVariable::new(&acceleration_scaling, "acceleration_scaling"),
                MessageVariable::new(&velocity_scaling, "velocity_scaling"),
                MessageVariable::new(&acceleration, "acceleration"),
                MessageVariable::new(&velocity, "velocity"),
                MessageVariable::new(&goal_feature_name, "goal_feature_id"),
                MessageVariable::new(&tcp_name, "tcp_id"),
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
                p!([!done] && [!error] && [trigger] && [success] && [action_state == "succeeded"]),
                Predicate::TRUE,
                vec![ a!(done)],
                vec![],
                TransitionType::Runner
            )
        );
        // formal representation, request -> done
        resource.add_transition(
            Transition::new(
                &format!("{}_finish_ok", name),
                p!([!done] && [!error] && [trigger]),
                Predicate::TRUE,
                vec![ a!(done)],
                vec![],
                TransitionType::Effect
            )
        );

        // sets error to true when we fail.
        resource.add_transition(
            Transition::new(
                &format!("{}_runner_finish_error", name),
                p!([!done] && [!error] && [trigger] &&
                   [[[action_state == "succeeded"] && [!success]] ||
                    [action_state == "timeout"] ||
                    [action_state == "aborted"] ||
                    [action_state == "canceled"]]),
                Predicate::TRUE,
                vec![ a!(error)],
                vec![],
                TransitionType::Runner
            )
        );
        // formal representation, request -> error
        // this is not really needed for control, but useful for verification.
        // resource.add_transition(
        //     Transition::new(
        //         &format!("{}_finish_error", name),
        //         p!([!p: done] && [!p: error] && [p: trigger]),
        //         Predicate::TRUE,
        //         vec![ a!( p: error)],
        //         vec![],
        //         TransitionType::Effect
        //     )
        // );

        // resetting the action (going back to "ok") is done
        // automatically when trigger is taken down.
        resource.add_transition(
            Transition::new(
                &format!("{}_runner_reset", name),
                p!([!trigger] && [action_state == "ok"] && [[done] || [error]]),
                Predicate::TRUE,
                vec![a!(!done), a!(!error)],
                vec![],
                TransitionType::Runner
            )
        );
        // formal representation of reset
        resource.add_transition(
            Transition::new(
                &format!("{}_reset", name),
                p!([!trigger] && [[done] || [error]]),
                Predicate::TRUE,
                vec![a!(!done), a!(!error)],
                vec![],
                TransitionType::Effect
            )
        );

        let initial_state = SPState::new_from_values(
            &[
                (goal_feature_name.clone(), frame_domain[1].clone()),
                (tcp_name.clone(), tool_frame_domain[1].clone()),
                (last_visited_frame.clone(), frame_domain[0].clone()),
                (last_visited_with_tcp.clone(), tool_frame_domain[0].clone()),
                (velocity_scaling, 0.1.to_spvalue()),
                (velocity.clone(), 0.1.to_spvalue()),
                (acceleration_scaling, 0.1.to_spvalue()),
                (acceleration.clone(), 0.1.to_spvalue()),
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
            velocity,
            goal_feature_name,
            tcp_name,
            current_state,
            success,
            done,
            error,
            action_state,
            initial_state,
            t_index: 0,
        }
    }

    pub fn define_motion(
        &mut self,
        model: &mut Model,
        operation_guard: Predicate,
        transition_guard: Predicate,
        runner_guard: Predicate,
        tcp_frame: &str,
        goal_frame: &str,
        command: &str,
        velocity: f32,
        acceleration: f32,
        mut action_when_done: Vec<Action>,
        mut action_when_error: Vec<Action>,
        // todo: add runner actions.
    ) {
        let goal_state = p!([self.trigger] && [self.done] &&
                            [(self.tcp_name) == tcp_frame] &&
                            [(self.goal_feature_name) == goal_frame]);

        // Add high level operation for the motion
        model.add_op(&format!("move_to_{}_with_{}_{}", goal_frame, tcp_frame, self.t_index),
                     &operation_guard,
                     &vec![a!((self.last_visited_frame) <- goal_frame),
                           a!((self.last_visited_with_tcp) <- tcp_frame),],
                     &goal_state,
                     &vec![a!(!self.trigger)],
                     false,
                     None);

        let r = model.get_resource(&self.path);
        let c = &self.command;
        let done = &self.done;
        let error = &self.error;
        let tcp_name = &self.tcp_name;
        let goal_feature_name = &self.goal_feature_name;
        let velocity_path = &self.velocity;
        let acceleration_path = &self.acceleration;
        let trigger = &self.trigger;
        let last_visited_frame = &self.last_visited_frame;
        let last_visited_with_tcp = &self.last_visited_with_tcp;
        let new_guard = p!([!trigger] && [!done] && [!error] && [p: transition_guard]);
        r.add_transition(Transition::new(
            &format!("{}_{}_to_{}_{}", &r.path().leaf(), tcp_frame, goal_frame, self.t_index),
            new_guard,
            runner_guard,
            vec![ // formal model cares about these
                a!(tcp_name <- tcp_frame),
                a!(goal_feature_name <- goal_frame),
                a!(trigger),
            ],
            vec![  // formal model dont care about these
                a!(c <- command),
                a!(velocity_path <- velocity),
                a!(acceleration_path <- acceleration),
            ],
            TransitionType::Controlled));

        // handle errors. perhaps this transition should be runner by default?
        // remove this to let the operation fail instead
        let guard_error = p!([trigger] && [error] &&
                             [tcp_name == tcp_frame] &&
                             [goal_feature_name == goal_frame]);
        action_when_error.push(a!(!trigger)); // reset
        action_when_error.push(a!(last_visited_frame <- "unknown")); // reset visited
        action_when_error.push(a!(last_visited_with_tcp <- "unknown")); // reset visited

        r.add_transition(Transition::new(
            &format!("{}_{}_to_{}_{}_error", &r.path().leaf(), tcp_frame, goal_frame, self.t_index),
            guard_error,
            Predicate::TRUE,
            action_when_error,
            vec![],
            TransitionType::Controlled));

        self.t_index += 1;

    }


}
