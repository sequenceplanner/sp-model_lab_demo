use sp_domain::*;
use sp_runner::*;
use sp_model::resources::ur::UrRobotResource;
use sp_model::resources::plc::PLCResource;

// for convenience we just launch within this binary.
#[tokio::main]
async fn main() {
    let (model, initial_state) = make_model();
    launch_model(model, initial_state).await.unwrap();
} 


pub fn make_model() -> (Model, SPState) {
    let mut m = Model::new("lab_scenario_1");

    let ur = m.add_resource("ur");
    let frames: Vec<SPValue> = ["pose_1", "pose_2"].iter().map(|f|f.to_spvalue()).collect();
    let tool_frames: Vec<SPValue> = ["tool0", "tool1"].iter().map(|f|f.to_spvalue()).collect();
    let ur = UrRobotResource::new(m.get_resource(&ur), frames, tool_frames);

    let x = m.add_runner_bool("x");
    let est_pos = ur.last_visited_frame.clone();
    let guard1 = p!([p:est_pos == "pose_2"] || [p:est_pos == "unknown"]);
    let guard2 = p!(p:est_pos == "pose_1");
    let runner_guard = p!(p: x);
    ur.run_transition(&mut m, guard1, runner_guard.clone(), "tool0", "pose_1", "move_j", 0.8, 0.5, vec![], vec![]);
    ur.run_transition(&mut m, guard2, runner_guard.clone(), "tool0", "pose_2", "move_j", 0.4, 0.3, vec![], vec![]);


    let plc_path = m.add_resource("plc");
    let d = vec!(0.to_spvalue(), 1.to_spvalue(), 2.to_spvalue());
    let domain = [d.clone(), d.clone(), d.clone(), d.clone(), d.clone()];
    let plc = PLCResource::new(m.get_resource(&plc_path), domain.clone(), domain.clone());


    // add high level ("product") state
    let op_done = m.add_product_bool("op_done");

    let _op_1_state = m.add_op(
        "do_operation_1",
        // operation model guard.
        &p!(!p: op_done),
        // operation model effects.
        &[a!(p: op_done)],
        // low level goal
        &p!(p: est_pos == "pose_2"),
        // low level actions (should not be needed)
        &[],
        // not auto
        false,
        None,
    );

    let _op_2_state = m.add_op(
        "do_operation_2",
        // operation model guard.
        &p!(p: op_done),
        // operation model effects.
        &[a!(!p: op_done)],
        // low level goal
        &p!(p: est_pos == "pose_1"),
        // low level actions (should not be needed)
        &[],
        // not auto
        false,
        None,
    );

    m.add_intention(
        "back",
        true,
        &p!(!p: op_done),
        &p!(p: op_done),
        &[],
    );

    m.add_intention(
        "forth",
        true,
        &p!(p: op_done),
        &p!(!p: op_done),
        &[],
    );

    let mut initial_state = ur.initial_state.clone();
    initial_state.extend(SPState::new_from_values(
        &[
            (est_pos, "pose_1".to_spvalue()),
            (op_done, false.to_spvalue()),
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
}
