use sp_domain::*;

pub struct FrameLocker {
    pub path: SPPath,
    pub frame_exists: SPPath, // boolean
    pub frame_locked: SPPath, // boolean
    pub is_locking: SPPath,
    pub initial_state: SPState,
}

impl FrameLocker {
    pub fn new(r: &mut Resource) -> FrameLocker {
        let _name = r.path().leaf();

        let lock_trigger = r.add_variable(Variable::new_boolean("lock_trigger", VariableType::Command));

        let frame_exists = r.add_variable(
            Variable::new_boolean("frame_exists", VariableType::Measured));
        let frame_locked = r.add_variable(
            Variable::new_boolean("frame_locked", VariableType::Measured));

        // setup the service
        let lock_service = r.setup_ros_service(
            "lock_frame",
            "/lock_frames",
            "std_srvs/srv/Trigger",
            p!(lock_trigger), &[], &[]);

        r.setup_ros_incoming("frame_exists", "/frame_exists",
                                    MessageType::Ros("std_msgs/msg/Bool".into()),
            &[
                MessageVariable::new(&frame_exists, "data"),
            ]);
        r.setup_ros_incoming("frame_exists", "/frame_locked",
                                    MessageType::Ros("std_msgs/msg/Bool".into()),
            &[
                MessageVariable::new(&frame_locked, "data"),
            ]);

        r.add_transition(Transition::new(
            &format!("{}_lock", &r.path().leaf()),
            p!([lock_service == "ok"] && [!lock_trigger] && [frame_exists] && [!frame_locked]),
            Predicate::TRUE,
            vec![a!(lock_trigger)],
            vec![],
            TransitionType::Controlled));

        r.add_transition(Transition::new(
            &format!("{}_lock_done", &r.path().leaf()),
            p!([lock_trigger] && [!frame_locked]),
            Predicate::TRUE,
            vec![a!(frame_locked)],
            vec![],
            TransitionType::Effect));

        r.add_transition(Transition::new(
            &format!("{}_lock_reset", &r.path().leaf()),
            p!([lock_trigger] && [frame_locked]),
            Predicate::TRUE,
            vec![a!(!lock_trigger)],
            vec![],
            TransitionType::Auto));

        let initial_state = SPState::new_from_values(
            &[
                (lock_trigger.clone(), false.to_spvalue()),
            ]);

        let is_locking = Variable::new_predicate("is_locking", p!([lock_trigger] &&
                                                                  [!frame_locked]));
        let is_locking = r.add_variable(is_locking);


        return FrameLocker {
            path: r.path().clone(),
            frame_exists,
            frame_locked,
            is_locking,
            initial_state,
        }
    }

}
