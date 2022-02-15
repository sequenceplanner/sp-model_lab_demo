use sp_domain::*;

pub struct RobotiqGripper {
    pub path: SPPath,
    pub command: SPPath, // "open", "close"
    pub measured: SPPath, // "opened", "closed", "gripping", "fault", "unknown"
}

// I decided to move these to the resource becuase you may
// want more or less details about the action state.
//
// The state machine (no error state for now):
//
// command == open && measured != open -> measured = open
// command == close && (measured == open || measured == unknown) -> measured = closed
// command == close && (measured == open || measured == unknown) -> measured = gripping
//
//
impl RobotiqGripper {
    pub fn new(resource: &mut Resource) -> RobotiqGripper {
        let _name = resource.path().leaf();

        let command = resource.add_variable(Variable::new("command", VariableType::Command,
                                                          SPValueType::String,
                                                          vec!["open".to_spvalue(),
                                                               "close".to_spvalue()]));
        let measured = resource.add_variable(Variable::new("measured", VariableType::Measured,
                                                           SPValueType::String,
                                                           vec!["opened".to_spvalue(),
                                                                "gripping".to_spvalue(),
                                                                "closed".to_spvalue()]));

        resource.setup_ros_outgoing("command", "/robotiq_2f_command",
                                    MessageType::Ros("robotiq_2f_msgs/msg/CommandState".into()),
            &[
                MessageVariable::new(&command, "command"),
            ]);

        resource.setup_ros_incoming("measured", "/robotiq_2f_measured",
                                    MessageType::Ros("robotiq_2f_msgs/msg/MeasuredState".into()),
            &[
                MessageVariable::new(&measured, "measured"),
            ]);

        return RobotiqGripper {
            path: resource.path().clone(),
            command,
            measured,
        }
    }

}
