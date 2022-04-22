use sp_domain::*;
use sp_runner::*;
use sp_model::resources::ur::UrRobotResource;
use sp_model::resources::plc::PLCResource;
use sp_model::resources::robotiq_gripper::RobotiqGripper;

// for convenience we just launch within this binary.
#[tokio::main]
async fn main() {
    let (model, initial_state) = make_model();
    launch_model(model, initial_state).await.unwrap();
}


pub fn make_model() -> (Model, SPState) {
    let mut m = Model::new("lab_scenario_1");

    let ur = m.add_resource("ur");
    let frames: Vec<SPValue> = ["at_conv", "above_conv",
                                "pose_1", "pose_2"].iter().map(|f|f.to_spvalue()).collect();
    let tool_frames: Vec<SPValue> = ["robotiq_2f_tcp", "tool1"].iter().map(|f|f.to_spvalue()).collect();
    let mut ur = UrRobotResource::new(m.get_resource(&ur), frames, tool_frames);

    let gripper = m.add_resource("gripper");
    let gripper = RobotiqGripper::new(m.get_resource(&gripper));

    let plc_path = m.add_resource("plc");
    let d = vec!(0.to_spvalue(), 1.to_spvalue(), 2.to_spvalue());
    let domain = [d.clone(), d.clone(), d.clone(), d.clone(), d.clone()];
    let plc = PLCResource::new(m.get_resource(&plc_path), domain.clone(), domain.clone());

    let est_pos = ur.last_visited_frame.clone();
    let guard1 = p!([est_pos == "pose_2"] || [est_pos == "unknown"]);
    let guard2 = p!(est_pos == "pose_1");
    // let runner_guard = p!(plc.bool_from_plc_1);
    let runner_guard = Predicate::TRUE;
    ur.create_transition(&mut m, guard1, runner_guard.clone(),
                      "robotiq_2f_tcp", "pose_1", "move_j", 0.1, 0.3, vec![], vec![]);
    ur.create_transition(&mut m, guard2, runner_guard.clone(),
                      "robotiq_2f_tcp", "pose_2", "move_j", 0.1, 0.3, vec![], vec![]);


    ur.create_transition(&mut m,
                      p!(est_pos == "pose_1"),
                      Predicate::TRUE,
                      "robotiq_2f_tcp", "above_conv", "move_j", 0.1, 0.3, vec![], vec![]);

    ur.create_transition(&mut m,
                      p!(est_pos == "above_conv"),
                      Predicate::TRUE,
                      "robotiq_2f_tcp", "pose_1", "move_j", 0.1, 0.3, vec![], vec![]);

    ur.create_transition(&mut m,
                      p!([est_pos == "above_conv"] &&
                         [[(gripper.measured) == "opened"] || [(gripper.measured) == "gripping"]]),
                      Predicate::TRUE,
                      "robotiq_2f_tcp", "at_conv", "move_j", 0.1, 0.3, vec![], vec![]);

    ur.create_transition(&mut m,
                      p!([est_pos == "at_conv"]),
                      Predicate::TRUE,
                      "robotiq_2f_tcp", "above_conv", "move_j", 0.1, 0.3, vec![], vec![]);

    // can only grip in certain positions.
    m.add_invar(
        "grip_at_the_right_pos",
        &p!([gripper.is_closing] => [est_pos == "at_conv"]),
    );
    m.add_invar(
        "release_at_the_right_pos",
        &p!([gripper.is_opening] => [est_pos == "at_conv"]),
    );

    // add high level ("product") state
    let op_done = m.add_product_bool("op_done");

    let buffer1 = m.add_product_bool("buffer1");
    let buffer2 = m.add_product_bool("buffer2");
    let buffer3 = m.add_product_bool("buffer3");
    let buffer4 = m.add_product_bool("buffer4");
    let buffer5 = m.add_product_bool("buffer5");
    let buffer6 = m.add_product_bool("buffer6");
    let buffer7 = m.add_product_bool("buffer7");
    let buffer8 = m.add_product_bool("buffer8");
    let buffer9 = m.add_product_bool("buffer9");

    // plc.command_transition(&mut m, "start_load", p!(!plc.bool_to_plc_1), runner_guard,
    //                        vec![ a!(plc.bool_to_plc_1)], vec![],
    //                        p!([!plc.bool_from_plc_2] && [plc.bool_to_plc_1]), vec![a!(plc.bool_from_plc_2)]);
    // let cylinder_by_sensor = m.add_product_bool("cylinder_by_sensor");
    // m.add_op(
    //     "cylinder_to_sensor",
    //     // operation model guard.
    //     &p!(!cylinder_by_sensor),
    //     // operation model effects.
    //     &[a!(cylinder_by_sensor)],
    //     // low level goal
    //     &p!(plc.bool_from_plc_2),
    //     // low level actions (should not be needed)
    //     &[a!(!plc.bool_to_plc_1)],
    //     // not auto
    //     true,
    //     None,
    // );


    let _op_1_state = m.add_op(
        "do_operation_1",
        // operation model guard.
        &p!(!op_done),
        // operation model effects.
        &[a!(op_done)],
        // low level goal
        &p!([est_pos == "pose_2"] && [(gripper.measured) == "gripping"]),
        // low level actions (should not be needed)
        &[],
        // auto
        true,
        None,
    );

    let _op_2_state = m.add_op(
        "do_operation_2",
        // operation model guard.
        &p!(op_done),
        // operation model effects.
        &[a!(!op_done)],
        // low level goal
        &p!([est_pos == "pose_2"] && [(gripper.measured) == "opened"]),
        // low level actions (should not be needed)
        &[],
        // auto
        true,
        None,
    );

    m.add_intention(
        "back",
        true,
        // &p!(!op_done),
        &Predicate::FALSE,
        &p!(op_done),
        &[],
    );

    m.add_intention(
        "forth",
        true,
        &p!(op_done),
        &p!(!op_done),
        &[],
    );

    // this is updated by the gui.
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
    initial_state.extend(SPState::new_from_values(
        &[
            // (est_pos, "pose_1".to_spvalue()),
            (est_pos, "unknown".to_spvalue()),
            (op_done, false.to_spvalue()),
            (buffer1, false.to_spvalue()),
            (buffer2, false.to_spvalue()),
            (buffer3, false.to_spvalue()),
            (buffer4, false.to_spvalue()),
            (buffer5, false.to_spvalue()),
            (buffer6, false.to_spvalue()),
            (buffer7, false.to_spvalue()),
            (buffer8, false.to_spvalue()),
            (buffer9, false.to_spvalue()),
            //(cylinder_by_sensor, false.to_spvalue()),
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
                println!("{}",e);
                assert!(false);
            },
            Ok(p) => {
                assert!(p.plan_found);
                println!("THE PLAN");
                for pf in &p.trace {
                    println!("{}", pf.transition);
                }
            },
        };
    }
}
