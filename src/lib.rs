mod resources;

use sp_domain::*;

/// send model + state to sp-launch
pub async fn update_sp(compiled_model: &sp_formal::CompiledModel, state: &SPState) {
    let state_json = SPStateJson::from_state_flat(&state)
        .to_json()
        .to_string();
    let cm_json = serde_json::to_string(compiled_model).unwrap();

    let state_req = r2r::sp_msgs::srv::Json::Request { json: state_json };
    let model_req = r2r::sp_msgs::srv::Json::Request { json: cm_json };

    let ctx = r2r::Context::create().map_err(SPError::from_any).unwrap();
    let mut node = r2r::Node::create(ctx, "model_maker", "")
        .map_err(SPError::from_any)
        .unwrap();

    let client = node
        .create_client::<r2r::sp_msgs::srv::Json::Service>("/sp/set_model")
        .unwrap();
    let client_state = node
        .create_client::<r2r::sp_msgs::srv::Json::Service>("/sp/set_state")
        .unwrap();

    // let c_alive = node.is_available(&client).unwrap();
    // let c_state_alive = node.is_available(&client_state).unwrap();

    let kill = std::sync::Arc::new(std::sync::Mutex::new(false));

    let k = kill.clone();
    let spin_handle = tokio::task::spawn_blocking(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
        if *k.lock().unwrap() {
            println!("Killing");
            break;
        }
    });

    let d = std::time::Duration::from_secs(5);
    print!("Sending state request... ");
    let result = tokio::time::timeout(d, client_state.request(&state_req).unwrap()).await;
    println!("{:?}", result);
    print!("Sending model change request... ");
    let result = tokio::time::timeout(d, client.request(&model_req).unwrap()).await;
    println!("{:?}", result);

    *kill.lock().unwrap() = true;
    spin_handle.await.unwrap();
}

/// fix for indention problem...
pub fn assign(p1: &SPPath, p2: &SPPath) -> Action {
    a!(p: p1 <- p: p2)
}

