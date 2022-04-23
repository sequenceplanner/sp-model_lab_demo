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

pub fn make_model() -> (Model, SPState) {
    let mut m = Model::new("lab_scenario_1");

    let ur = m.add_resource("ur");
    let mut frames: Vec<SPValue> = ["home_pose", "above_conv", "at_conv", "above_locked_aruco"]
        .iter()
        .map(|f| f.to_spvalue())
        .collect();
    let buffer_frames = ["above_buffer_1", "above_buffer_2", "above_buffer_3", "above_buffer_4"];
    frames.extend(buffer_frames
        .iter()
        .map(|f| f.to_spvalue())
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

    // ur move home to above_conv
    ur.define_motion(&mut m, p!(est_pos == "home_pose"), Predicate::TRUE,
                         "robotiq_2f_tcp", "above_conv", "move_j", 0.1, 0.3, vec![], vec![]);

    // ur move above_conv to home
    ur.define_motion(&mut m, p!(est_pos == "above_conv"), Predicate::TRUE,
                     "robotiq_2f_tcp", "home_pose", "move_j", 0.1, 0.3, vec![], vec![]);

    // ur move above_conv to at_conv
    ur.define_motion(&mut m, p!(est_pos == "above_conv"),
                     Predicate::TRUE,
                     "robotiq_2f_tcp", "at_conv", "move_j", 0.1, 0.3, vec![], vec![]);

    // ur move home to above_locked_aruco
    ur.define_motion(&mut m, p!(est_pos == "home_pose"), Predicate::TRUE,
                     "robotiq_2f_tcp", "above_locked_aruco", "move_j", 0.1, 0.3, vec![], vec![]);

    // ur move above_locked_aruco to home
    ur.define_motion(&mut m, p!(est_pos == "above_locked_aruco"), Predicate::TRUE,
                     "robotiq_2f_tcp", "home_pose", "move_j", 0.1, 0.3, vec![], vec![]);

    for bf in &buffer_frames {
        ur.define_motion(&mut m, p!(est_pos == "home_pose"), Predicate::TRUE,
                         "robotiq_2f_tcp", bf, "move_j", 0.1, 0.3, vec![], vec![]);
        ur.define_motion(&mut m, p!(est_pos == bf), Predicate::TRUE,
                         "robotiq_2f_tcp", "home_pose", "move_j", 0.1, 0.3, vec![], vec![]);
    }

    // can only grip in certain positions.
    m.add_invar(
        "grip_at_the_right_pos",
        &p!([gripper.is_closing] => [est_pos == "at_conv"]),
    );
    m.add_invar(
        "release_at_the_right_pos",
        // todo: add all positions.
        &p!([gripper.is_opening] => [[est_pos == "above_buffer_1"] || [est_pos == "above_buffer_2"]]),
    );

    // add high level ("product") state

    // buffer locations on paper
    let buffers: Vec<SPPath> = (1..=4).map(|i: i32| {
        m.add_product_bool(&format!("buffer_{}",i))
    }).collect();

    // cylinder by end of conveyor
    let cylinder_by_sensor = m.add_product_bool("cylinder_by_sensor");

    // cylinder in gripper
    let cylinder_in_gripper = m.add_product_bool("cylinder_in_gripper");

    plc.command_transition(&mut m, "start_load", p!(!plc.bool_to_plc_1), Predicate::TRUE,
                           vec![ a!(plc.bool_to_plc_1)], vec![],
                           p!([!plc.bool_from_plc_2] && [plc.bool_to_plc_1]),
                           vec![a!(plc.bool_from_plc_2)]);
    m.add_op(
        "cylinder_to_sensor",
        // operation model guard.
        &p!(!cylinder_by_sensor),
        // operation model effects.
        &[a!(cylinder_by_sensor)],
        // low level goal
        &p!(plc.bool_from_plc_2),
        // low level actions (usually not be needed)
        &[a!(!plc.bool_to_plc_1)],
        // auto
        false,
        None,
    );

    let _pick_op = m.add_op(
        "pick",
        &p!(!cylinder_in_gripper),
        &[a!(cylinder_in_gripper)],
        &p!([est_pos == "at_conv"] && [(gripper.measured) == "gripping"]),
        &[],
        true,
        None,
    );

    for b in &buffers {
        let op_name = format!("place_at_{}", b.leaf());
        let frame_name = format!("above_{}", b.leaf());
        let _place_op = m.add_op(
            &op_name,
            &p!(cylinder_in_gripper),
            &[a!(!cylinder_in_gripper), a!(b)],
            &p!([est_pos == frame_name] && [(gripper.measured) == "opened"]),
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

    let mut initial_state = ur.initial_state.clone();
    initial_state.extend(plc.initial_state);
    initial_state.extend(gripper.initial_state);
    initial_state.extend(frame_locker.initial_state);

    let buffer_states: Vec<_> =
        buffers.iter().map(|b| (b.clone(), false.to_spvalue())).collect();
    initial_state.extend(SPState::new_from_values(&buffer_states));

    initial_state.extend(SPState::new_from_values(
        &[
            // (est_pos, "pose_1".to_spvalue()),
            (est_pos, "unknown".to_spvalue()),
            (cylinder_by_sensor, false.to_spvalue()),
            (cylinder_in_gripper, false.to_spvalue()),
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
