use sp_domain::*;
use sp_runner::*;
use sp_model::resources;

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
    let est_pos = m.add_estimated_domain("ur/last_pos", &frames);
    let ur = resources::ur::UrRobotResource::new(m.get_resource(&ur), frames);

    let guard1 = p!(p:est_pos == "pose_2");
    let guard2 = p!(p:est_pos == "pose_1");
    let action1 = a!(p: est_pos = "pose_1");
    let action2 = a!(p: est_pos = "pose_2");
    ur.run_transition(&mut m, guard1, "tool0", "pose_1", "move_j", 0.1, vec![action1], vec![]);
    ur.run_transition(&mut m, guard2, "tool0", "pose_2", "move_j", 0.1, vec![action2], vec![]);

    let initial_state = SPState::new_from_values(
        &[
            (est_pos, "pose_1".to_spvalue())
        ]
    );

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
