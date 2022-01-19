use sp_domain::*;



struct UrRobotResource{
    run: SPPath,  // Command. Trigger a robot motion
    feedback: SPPath,  // Measured. feedback from robot
    done: SPPath,   // Measured (changed by runner transitions). Motion completed
    error: SPPath,  // Measured. Error from action
    current_tcp_frame: SPPath,  // estimated. the tcp_frame that will be at the goal. Will change when a new motion is triggered.
    current_tcp_frame_pose: SPPath  // estimated. The pose of the tcp_frame. Will be unknown, moving or the goal frame. 
}

impl UrRobotResource {
    pub fn new(resource: &mut Resource, frame_domain: Vec<SPValue>) -> UrRobotResource {
        let command= resource.add_variable(Variable::new(
            "request/command",VariableType::Runner,SPValueType::String,vec!(),
        ));
    
        let acceleration= resource.add_variable(Variable::new(
            "request/acceleration",VariableType::Runner,SPValueType::Float32,vec!(),
        ));
        let velocity= resource.add_variable(Variable::new(
            "request/velocity",VariableType::Runner,SPValueType::Float32,vec!(),
        ));
        let goal_feature_name= resource.add_variable(Variable::new(
            "request/goal_feature_name",VariableType::Runner,SPValueType::String, frame_domain.clone(),
        ));
        let tcp_name= resource.add_variable(Variable::new(
            "request/tcp_name",VariableType::Runner,SPValueType::String,frame_domain.clone(),
        ));
    
        let current_state= resource.add_variable(Variable::new(
            "feedback/current_state", VariableType::Runner, SPValueType::String, vec!(),
        ));
    
        let success= resource.add_variable(Variable::new_boolean(
            "reply/success", VariableType::Runner
        ));
    
        let trigger= Variable::new_boolean("trigger", VariableType::Runner);
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
    
        let run= resource.add_variable(Variable::new_boolean(
            "command/run", VariableType::Command
        ));
        let done= resource.add_variable(Variable::new_boolean(
            "measured/done", VariableType::Measured
        ));
        let error= resource.add_variable(Variable::new_boolean(
            "measured/error", VariableType::Measured
        ));
        // Update the feedback domain
        let feedback_domain: Vec<SPValue> = ["starting", "error", "done"].iter().map(|v| v.to_spvalue()).collect();
        let feedback = resource.add_variable(Variable::new(
            "measured/feedback",VariableType::Measured,SPValueType::String, feedback_domain,
        ));
        let current_tcp_frame = resource.add_variable(Variable::new(
            "estimated/current_tcp_frame",VariableType::Estimated,SPValueType::String, frame_domain.clone(),
        ));
        let current_tcp_frame_pose = resource.add_variable(Variable::new(
            "estimated/current_tcp_frame_pose",VariableType::Estimated,SPValueType::String, frame_domain.clone(),
        ));

        return UrRobotResource{
            run,
            feedback,
            done,
            error,
            current_tcp_frame,
            current_tcp_frame_pose
        }
    }
}




