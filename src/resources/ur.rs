use sp_domain::*;



struct UrRobotResource{
    trigger: SPPath,
    goal_feature_name: SPPath,
    tcp_name: SPPath,
    command: SPPath,
    acceleration: SPPath,
    velocity: SPPath,

    
}

/// Make a UR robot resource that talks with the UR action
/// The frame_domain defines the possible frames to move to defined in TF. The tcps are the 
/// possible tcp frames of the robot that should end up at the frame. if it is not possible to 
/// move a specific tcp to a frame, it must be restricted by invariants
pub fn make_ur(resource: &mut Resource, frame_domain: Vec<SPValue>, tcps: Vec<SPValue>) { 
    let command= resource.add_variable(Variable::new(
        "request/command",VariableType::Command,SPValueType::String,vec!(),
    ));

    let acceleration= resource.add_variable(Variable::new(
        "request/acceleration",VariableType::Runner,SPValueType::Float32,vec!(),
    ));
    let velocity= resource.add_variable(Variable::new(
        "request/velocity",VariableType::Runner,SPValueType::Float32,vec!(),
    ));
    let goal_feature_name= resource.add_variable(Variable::new(
        "request/goal_feature_name",VariableType::Command,SPValueType::String, frame_domain.clone(),
    ));
    let tcp_name= resource.add_variable(Variable::new(
        "request/tcp_name",VariableType::Command,SPValueType::String,tcps,
    ));

    let current_state= resource.add_variable(Variable::new(
        "feedback/current_state", VariableType::Runner, SPValueType::String, vec!(),
    ));

    let success= resource.add_variable(Variable::new_boolean(
        "reply/success", VariableType::Measured
    ));

    let trigger= Variable::new_boolean("trigger", VariableType::Command);
    let trigger= resource.add_variable(trigger);

    let _action_state = resource.setup_ros_action(
        "URScriptControl",
        "/ur_script_controller",
        "ur_tools_msgs/action/URScriptControl",
        p!(p: trigger),
        // goal variables
        &[
            MessageVariable::new(&command, "command"),
            MessageVariable::new(&acceleration, "acceleration"),
            MessageVariable::new(&velocity, "velocity"),
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

    let actual_tool_frame = resource.add_variable(Variable::new(
        "request/measured",VariableType::Measured,SPValueType::String, frame_domain,
    ));

    for (p, v) in frames {
        // restrict motions motions using invariants in the scenario model. 
        // Copy this transition and use it 
        resource.add_transition(
            Transition::new(
                &format!("to_{}_start", v.to_string()),
                p!(!p: trigger),
                vec![ a!( p: trigger)],
                TransitionType::Controlled
            )
        );
        resource.add_transition(
            Transition::new(
                &format!("to_{}_effect", v.to_string()),
                p!([p: trigger]),
                vec![ a!(!p: success)],
                TransitionType::Effect
            )
        );
        resource.add_transition(
            Transition::new(
                &format!("to_{}_fin", v.to_string()),
                p!([p: trigger] && [p: success]),
                vec![ a!(!p: trigger), a!(p: actual_tool_frame <- goal_feature_name)],
                TransitionType::Auto
            )
        );

    }

}

