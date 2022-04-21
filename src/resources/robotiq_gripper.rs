use sp_domain::*;

pub struct RobotiqGripper {
    pub path: SPPath,
    pub measured: SPPath, // "opened", "closed", "gripping"
    pub is_closing: SPPath, // predicates
    pub is_opening: SPPath,
    pub initial_state: SPState,
}

// I decided to move these to the resource becuase you may
// want more or less details about the action state.
//
// The state machine (no error state for now):
//
// command == open && measured != open -> measured = open
// command == close && (measured == open) -> measured = closed
// command == close && (measured == open) -> measured = gripping
//
//
impl RobotiqGripper {
    pub fn new(r: &mut Resource) -> RobotiqGripper {
        let _name = r.path().leaf();

        let open_trigger = r.add_variable(Variable::new_boolean("open_trigger", VariableType::Command));
        let close_trigger = r.add_variable(Variable::new_boolean("close_trigger", VariableType::Command));

        let measured = r.add_variable(Variable::new("measured", VariableType::Measured,
                                                           SPValueType::String,
                                                           vec!["unknown".to_spvalue(), // initiall and when moving
                                                                "opened".to_spvalue(),
                                                                "gripping".to_spvalue(),
                                                                "closed".to_spvalue()]));

        // setup the service
        let open_service = r.setup_ros_service(
            "robotiq_2f_open",
            "/robotiq_2f_open",
            "std_srvs/srv/Trigger",
            p!(open_trigger), &[], &[]);

        let close_service = r.setup_ros_service(
            "robotiq_2f_close",
            "/robotiq_2f_close",
            "std_srvs/srv/Trigger",
            p!(close_trigger), &[], &[]);

        r.setup_ros_incoming("measured", "/robotiq_2f_measured",
                                    MessageType::Ros("robotiq_2f_msgs/msg/MeasuredState".into()),
            &[
                MessageVariable::new(&measured, "measured"),
            ]);

        r.add_transition(Transition::new(
            &format!("{}_open", &r.path().leaf()),
            p!([open_service == "ok"] && [!open_trigger] && [measured != "opened"]),
            Predicate::TRUE,
            vec![a!(open_trigger)],
            vec![],
            TransitionType::Controlled));

        r.add_transition(Transition::new(
            &format!("{}_open_done", &r.path().leaf()),
            p!([open_trigger] && [measured != "opened"]),
            Predicate::TRUE,
            vec![a!(measured <- "opened")],
            vec![],
            TransitionType::Effect));

        r.add_transition(Transition::new(
            &format!("{}_open_reset", &r.path().leaf()),
            p!([open_trigger] && [measured == "opened"]),
            Predicate::TRUE,
            vec![a!(!open_trigger)],
            vec![],
            TransitionType::Auto));

        r.add_transition(Transition::new(
            &format!("{}_close", &r.path().leaf()),
            p!([close_service == "ok"] && [!close_trigger] && [measured == "opened"]),
            Predicate::TRUE,
            vec![a!(close_trigger)],
            vec![],
            TransitionType::Controlled));

        r.add_transition(Transition::new(
            &format!("{}_close_done", &r.path().leaf()),
            p!([close_trigger] && [measured != "gripping"] && [measured != "closed"]),
            Predicate::TRUE,
            vec![a!(measured <- "gripping")],
            vec![],
            TransitionType::Effect));

        r.add_transition(Transition::new(
            &format!("{}_close_reset", &r.path().leaf()),
            p!([close_trigger] && [[measured == "gripping"] || [measured == "closed"]]),
            Predicate::TRUE,
            vec![a!(!close_trigger)],
            vec![],
            TransitionType::Auto));


        let is_closing = Variable::new_predicate("is_closing", p!([close_trigger] &&
                                                                  [measured != "closed"] &&
                                                                  [measured != "gripping"]));
        let is_closing = r.add_variable(is_closing);

        let is_opening = Variable::new_predicate("is_opening", p!([open_trigger] &&
                                                                  [measured != "opened"]));
        let is_opening = r.add_variable(is_opening);

        let initial_state = SPState::new_from_values(
            &[
                (open_trigger.clone(), false.to_spvalue()),
                (close_trigger.clone(), false.to_spvalue()),
            ]);

        return RobotiqGripper {
            path: r.path().clone(),
            measured,
            is_closing,
            is_opening,
            initial_state,
        }
    }

}
