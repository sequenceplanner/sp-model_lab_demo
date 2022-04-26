use sp_domain::*;
use sp_model::resources::plc::PLCResource;
use sp_model::resources::robotiq_gripper::RobotiqGripper;
use sp_model::resources::frame_locker::FrameLocker;
use sp_model::resources::ur::UrRobotResource;
use sp_runner::*;

// for convenience we just launch within this binary.
#[tokio::main]
async fn main() {
    let (model, initial_state) = make_model();
    launch_model(model, initial_state).await.unwrap();
}

// buffers p1-p4
struct Buffer {
    above_frame_name: String,
    at_frame_name: String,
    variable: SPPath
}

pub fn make_model() -> (Model, SPState) {
    let mut m = Model::new("lab_scenario_1");

    let mut buffers = Vec::new();
    buffers.push(Buffer { above_frame_name: "p1_above".into(), at_frame_name: "p1_down".into(),
                          variable: m.add_product_bool("buffer_1") });
    buffers.push(Buffer { above_frame_name: "p2_above".into(), at_frame_name: "p2_down".into(),
                          variable: m.add_product_bool("buffer_2") });
    buffers.push(Buffer { above_frame_name: "p3_above".into(), at_frame_name: "p3_down".into(),
                          variable: m.add_product_bool("buffer_3") });
    buffers.push(Buffer { above_frame_name: "p4_above".into(), at_frame_name: "p4_down".into(),
                          variable: m.add_product_bool("buffer_4") });

    // cylinder by end of conveyor
    let cylinder_by_sensor = m.add_product_bool("cylinder_by_sensor");

    // cylinder in gripper
    let cylinder_in_gripper = m.add_product_bool("cylinder_in_gripper");

    let ur = m.add_resource("ur");
    let mut frames: Vec<SPValue> = ["home_pose",
                                    "pickup",
                                    "pickdown",
                                    "placedown",
                                    "find_aruco_1",
                                    "find_aruco_2",]
        .iter()
        .map(|f| f.to_spvalue())
        .collect();
    frames.extend(buffers
        .iter()
        .map(|f| f.at_frame_name.to_spvalue())
        .collect::<Vec<_>>());

    let tool_frames: Vec<SPValue> = ["robotiq_2f_tcp", "tool0"]
        .iter()
        .map(|f| f.to_spvalue())
        .collect();
    let mut ur = UrRobotResource::new(m.get_resource(&ur), frames, tool_frames);

    let gripper = m.add_resource("gripper");
    let gripper = RobotiqGripper::new(m.get_resource(&gripper));

    let frame_locker = m.add_resource("frame_locker");
    let frame_locker = FrameLocker::new(m.get_resource(&frame_locker));

    let plc_path = m.add_resource("plc");
    let d = vec![0.to_spvalue(), 1.to_spvalue(), 2.to_spvalue()];
    let domain = [d.clone(), d.clone(), d.clone(), d.clone(), d.clone()];
    let plc = PLCResource::new(m.get_resource(&plc_path), domain.clone(), domain.clone());

    let est_pos = ur.last_visited_frame.clone();

    // ur move from unknown to home. may be dangerous.
    ur.define_motion(&mut m, p!(est_pos == "unknown"), Predicate::TRUE,
                     "robotiq_2f_tcp", "home_pose", "move_j", 0.4, 0.3, vec![], vec![]);

    // ur move home to pickup
    ur.define_motion(&mut m, p!(est_pos == "home_pose"), Predicate::TRUE,
                         "robotiq_2f_tcp", "pickup", "move_j", 0.4, 0.3, vec![], vec![]);

    // ur move pickup to home
    ur.define_motion(&mut m, p!(est_pos == "pickup"), Predicate::TRUE,
                     "robotiq_2f_tcp", "home_pose", "move_j", 0.4, 0.3, vec![], vec![]);

    // ur move pickup to pickdown
    ur.define_motion(&mut m, p!(est_pos == "pickup"), Predicate::TRUE,
                     "robotiq_2f_tcp", "pickdown", "move_j", 0.4, 0.3, vec![], vec![]);

    // ur move from pickdown back to pickup
    ur.define_motion(&mut m, p!(est_pos == "pickdown"), Predicate::TRUE,
                     "robotiq_2f_tcp", "pickup", "move_j", 0.4, 0.3, vec![], vec![]);

    // ur move from pickdown back to home_pose
    ur.define_motion(&mut m, p!(est_pos == "pickdown"), Predicate::TRUE,
                     "robotiq_2f_tcp", "home_pose", "move_j", 0.4, 0.3, vec![], vec![]);

    // ur move from home to place
    ur.define_motion(&mut m, p!(est_pos == "home_pose"), Predicate::TRUE,
                     "robotiq_2f_tcp", "placedown", "move_j", 0.4, 0.3, vec![], vec![]);

    // ur move from place back to home_pose
    ur.define_motion(&mut m, p!(est_pos == "placedown"), Predicate::TRUE,
                     "robotiq_2f_tcp", "home_pose", "move_j", 0.4, 0.3, vec![], vec![]);


    for bf in &buffers {
        ur.define_motion(&mut m, p!(est_pos == "home_pose"), Predicate::TRUE,
                         "robotiq_2f_tcp", &bf.at_frame_name, "move_j", 0.4, 0.3, vec![], vec![]);
        ur.define_motion(&mut m, p!(est_pos == bf.at_frame_name), Predicate::TRUE,
                         "robotiq_2f_tcp", "home_pose", "move_j", 0.4, 0.3, vec![], vec![]);
    }

    // can only grip in certain positions.
    m.add_invar(
        "grip_at_the_right_pos",
        &p!([gripper.is_closing] => [[est_pos == "pickdown"] ||
                                     [est_pos == "p1_down"] ||
                                     [est_pos == "p2_down"] ||
                                     [est_pos == "p3_down"] ||
                                     [est_pos == "p4_down"]]),
    );
    m.add_invar(
        "release_at_the_right_pos",
        // todo: add all positions.
        &p!([gripper.is_opening] => [[est_pos == "pickup"] ||
                                     [est_pos == "placedown"] ||
                                     [est_pos == "p1_down"] || [est_pos == "p2_down"] ||
                                     [est_pos == "p3_down"] || [est_pos == "p4_down"]]),
    );

    // m.add_invar("dont_open_gripper_after_failure",
    //             &p!([[gripper.is_opening] =>  [p:ap == t2] || [p:ap == t3] || [p:ap == leave]]] => [p:r1_gripper_part])




    // PLC operations.
    m.add_transition(Transition::new("start_load", p!(
        [!plc.bool_to_plc_1] && [est_pos != "pickdown"]), Predicate::TRUE,
                                vec![ a!(plc.bool_to_plc_1)], vec![], TransitionType::Controlled));
    m.add_transition(Transition::new("finish_load",
                                     p!([!plc.bool_from_plc_1] && [plc.bool_to_plc_1]), Predicate::TRUE,
                                     vec![a!(plc.bool_from_plc_1)], vec![], TransitionType::Effect));
    m.add_op(
        "cylinder_to_sensor",
        // operation model guard.
        &p!(!cylinder_by_sensor),
        // operation model effects.
        &[a!(cylinder_by_sensor)],
        // low level goal
        &p!(plc.bool_from_plc_1),
        // low level actions
        &[a!(!plc.bool_to_plc_1)],
        // auto
        false,
        None,
    );

    m.add_transition(Transition::new("start_unload", p!(!plc.bool_to_plc_2), Predicate::TRUE,
                                vec![ a!(plc.bool_to_plc_2)], vec![], TransitionType::Controlled));
    m.add_transition(Transition::new("finish_unload",
                                     p!([!plc.bool_from_plc_2] && [plc.bool_to_plc_2]), Predicate::TRUE,
                                     vec![a!(plc.bool_from_plc_2)], vec![], TransitionType::Effect));
    m.add_op(
        "cylinder_from_sensor",
        // operation model guard.
        &p!(cylinder_by_sensor),
        // operation model effects.
        &[a!(!cylinder_by_sensor)],
        // low level goal
        &p!(plc.bool_from_plc_2),
        // low level actions
        &[a!(!plc.bool_to_plc_2)],
        // auto
        false,
        None,
    );

    let _pick_at_conv_op = m.add_op(
        "pick_at_conv",
        &p!([!cylinder_in_gripper] && [cylinder_by_sensor]),
        &[a!(cylinder_in_gripper), a!(!cylinder_by_sensor)],
        &p!([est_pos == "pickdown"] && [(gripper.measured) == "gripping"]),
        &[],
        true,
        None,
    );

    let _place_at_conv_op = m.add_op(
        "place_at_conv",
        &p!([cylinder_in_gripper] && [!cylinder_by_sensor]),
        &[a!(!cylinder_in_gripper), a!(cylinder_by_sensor)],
        &p!([est_pos == "placedown"] && [(gripper.measured) == "opened"]),
        &[],
        true,
        None,
    );

    // variable for alternating between find aruco poses.
    let looked_at_1 = m.add_estimated_bool("looked_at_1");

    // effect that says when we move towards an aruco finding pose, the aruco will show up
    m.add_transition(Transition::new(
        &format!("find_aruco_at_1"),
        p!([!frame_locker.frame_exists] &&
           [(ur.trigger)] && [!(ur.done)] &&
           [[(ur.goal_feature_name) == "find_aruco_1"] || [(ur.goal_feature_name) == "find_aruco_2"]]),
        Predicate::TRUE,
        vec![a!(frame_locker.frame_exists)],
        vec![],
        TransitionType::Effect));

    // ur move home to find aruco 1
    ur.define_motion(&mut m, p!([est_pos == "home_pose"] && [!looked_at_1]), Predicate::TRUE,
                     "robotiq_2f_tcp", "find_aruco_1", "move_j", 0.4, 0.3, vec![a!(looked_at_1)], vec![]);

    // ur move find aruco 1 to home
    ur.define_motion(&mut m, p!(est_pos == "find_aruco_1"), Predicate::TRUE,
                     "robotiq_2f_tcp", "home_pose", "move_j", 0.4, 0.3, vec![], vec![]);

    // ur move home to find aruco 2
    ur.define_motion(&mut m, p!([est_pos == "home_pose"] && [looked_at_1]), Predicate::TRUE,
                     "robotiq_2f_tcp", "find_aruco_2", "move_j", 0.4, 0.3, vec![a!(!looked_at_1)], vec![]);

    // ur move find aruco 2 to home
    ur.define_motion(&mut m, p!(est_pos == "find_aruco_2"), Predicate::TRUE,
                     "robotiq_2f_tcp", "home_pose", "move_j", 0.4, 0.3, vec![], vec![]);

    // can only lock in the lock positions.
    m.add_invar(
        "lock_at_the_right_pos",
        // todo: add all positions.
        &p!([frame_locker.is_locking] => [[est_pos == "find_aruco_1"] || [est_pos == "find_aruco_2"]])
    );

    let aruco_locked = m.add_product_bool("aruco_locked");
    let _lock_aruco_op = m.add_op(
        "lock_aruco",
        &p!(!aruco_locked),
        &[a!(aruco_locked)],
        &p!([frame_locker.frame_locked]),
        &[],
        true,
        None,
    );

    // convenience reset of product variable
    m.add_transition(Transition::new(
        &format!("reset_locked_aruco"),
        p!([aruco_locked] && [!frame_locker.frame_locked]),
        Predicate::TRUE,
        vec![a!(!aruco_locked)],
        vec![],
        TransitionType::Runner));

    for b in &buffers {
        let _place_op = m.add_op(
            &format!("place_at_{}", b.variable.leaf()),
            &p!([cylinder_in_gripper] && [aruco_locked] && [!b.variable]),
            &[a!(!cylinder_in_gripper), a!(b.variable)],
            &p!([est_pos == b.at_frame_name] && [(gripper.measured) == "opened"]),
            &[],
            false,
            None,
        );

        let _pick_op = m.add_op(
            &format!("pick_at_{}", b.variable.leaf()),
            &p!([!cylinder_in_gripper] && [aruco_locked] && [b.variable]),
            &[a!(cylinder_in_gripper), a!(!b.variable)],
            &p!([est_pos == b.at_frame_name] && [(gripper.measured) == "gripping"]),
            &[],
            false,
            None,
        );
    }

    // This intention is updated by the GUI.
    m.add_intention(
        "test_intention",
        false,
        &Predicate::FALSE,
        &Predicate::FALSE,
        &[],
    );

    m.add_intention(
        "clear_scene",
        false,
        &Predicate::FALSE,
        &p!([!cylinder_in_gripper] && [!cylinder_by_sensor] &&
            [! buffers[0].variable] && [! buffers[1].variable] &&
            [! buffers[2].variable] && [! buffers[3].variable]),
        &[],
    );

    let mut initial_state = ur.initial_state.clone();
    initial_state.extend(plc.initial_state);
    initial_state.extend(gripper.initial_state);
    initial_state.extend(frame_locker.initial_state);

    let buffer_states: Vec<_> =
        buffers.iter().map(|b| (b.variable.clone(), false.to_spvalue())).collect();
    initial_state.extend(SPState::new_from_values(&buffer_states));

    initial_state.extend(SPState::new_from_values(
        &[
            (looked_at_1, false.to_spvalue()),
            (est_pos, "unknown".to_spvalue()),
            (cylinder_by_sensor, false.to_spvalue()),
            (cylinder_in_gripper, false.to_spvalue()),
            (aruco_locked, false.to_spvalue()),
        ]
    ));

    // operations start in init
    let op_state = m
        .operations
        .iter()
        .map(|o| (o.path().clone(), "i".to_spvalue()))
        .collect::<Vec<_>>();
    initial_state.extend(SPState::new_from_values(op_state.as_slice()));

    // intentions are initially "paused"
    let intention_state = m
        .intentions
        .iter()
        .map(|i| (i.path().clone(), "i".to_spvalue()))
        .collect::<Vec<_>>();
    initial_state.extend(SPState::new_from_values(intention_state.as_slice()));

    return (m, initial_state);
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn modelcheck() {
        let (m, _s) = make_model();

        // println!("{:#?}", m);

        sp_formal::generate_mc_problems(&m);
    }

    #[test]
    fn plan() {
        let (m, mut s) = make_model();
        let ts_model = TransitionSystemModel::from(&m);

        // make a goal
        let op_done = SPPath::from_string("lab_scenario_1/product_state/op_done");
        let goal = [(p!(op_done), None)];

        // set initial state
        let gripper_state = SPPath::from_string("lab_scenario_1/gripper/measured");
        s.add_variable(gripper_state, "opened".to_spvalue());

        let plan = sp_formal::planning::plan(&ts_model, &goal, &s, 100);
        match plan {
            Err(e) => {
                println!("{}", e);
                assert!(false);
            }
            Ok(p) => {
                assert!(p.plan_found);
                println!("THE PLAN");
                for pf in &p.trace {
                    println!("{}", pf.transition);
                }
            }
        };
    }
}
