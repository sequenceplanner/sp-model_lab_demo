use sp_domain::*;
use sp_model::*;

#[tokio::main]
async fn main() {
    let (model, initial_state) = make_model();
    let compiled_model = sp_formal::CompiledModel::from(model.clone());
    update_sp(&compiled_model, &initial_state).await;
}

pub fn make_model() -> (Model, SPState) {
    let mut m = Model::new("lab_scenario_1");
    let initial_state =SPState::new();

    
    return (m, s);
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
