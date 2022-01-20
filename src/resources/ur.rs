use sp_domain::*;



struct UrRobotResource {
    pub path: SPPath,
    pub velocity: SPPath,
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
}

impl UrRobotResource {
    pub fn new(resource: &mut Resource, frame_domain: Vec<SPValue>) -> UrRobotResource {
        let trigger= Variable::new_boolean("trigger", VariableType::Runner);
        let trigger= resource.add_variable(trigger);
        
        let command= resource.add_variable(Variable::new(
            "request/command",VariableType::Runner,SPValueType::String,vec!(),
        ));
    
        let acceleration= resource.add_variable(Variable::new(
            "request/acceleration",VariableType::Runner,SPValueType::Float32,vec!(),
        ));
        let real_velocity= resource.add_variable(Variable::new(
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
    
        let _action_state = resource.setup_ros_action(
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
    
        let done= resource.add_variable(Variable::new_boolean(
            "measured/done", VariableType::Measured
        ));
        let error= resource.add_variable(Variable::new_boolean(
            "measured/error", VariableType::Measured
        ));

        let velocity = resource.add_variable(Variable::new(
            "estimated/current_tcp_frame",VariableType::Estimated,SPValueType::String, frame_domain.clone(),
        ));


        return UrRobotResource{
            path: resource.path().clone(),
            velocity,
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
        action_when_done: Vec<Action>,
        actions_on_error: Vec<(Predicate, Vec<Action>)>
    ) {
        let mut r = model.get_resource(&self.path);
        let c = &self.command;
        let tcp_name = &self.tcp_name;
        let goal_feature_name = &self.goal_feature_name;
        let real_velocity = &self.real_velocity;
        let trigger = &self.trigger;
        r.add_transition(Transition::new(
            &format!("{}_{}_to_{}", &r.path().leaf(), tcp_frame, goal_frame), 
            guard, 
            vec![
                a!(p:c = command), 
                a!(p:tcp_name = tcp_frame), 
                a!(p:goal_feature_name = goal_frame), 
                a!(p:real_velocity <- p:self.velocity), // Add multiplication actions before this works. Or send velocity and scaling to driver
                a!(p:trigger), 
            ],
            TransitionType::Controlled));

    }

    
}




