use sp_domain::*;

pub struct PLCResource {
    pub path: SPPath,
    pub bool_from_plc_1: SPPath,
    pub bool_from_plc_2: SPPath,
    pub bool_from_plc_3: SPPath,
    pub bool_from_plc_4: SPPath,
    pub bool_from_plc_5: SPPath,
    pub int_from_plc_1: SPPath,
    pub int_from_plc_2: SPPath,
    pub int_from_plc_3: SPPath,
    pub int_from_plc_4: SPPath,
    pub int_from_plc_5: SPPath,
    pub bool_to_plc_1: SPPath,
    pub bool_to_plc_2: SPPath,
    pub bool_to_plc_3: SPPath,
    pub bool_to_plc_4: SPPath,
    pub bool_to_plc_5: SPPath,
    pub int_to_plc_1: SPPath,
    pub int_to_plc_2: SPPath,
    pub int_to_plc_3: SPPath,
    pub int_to_plc_4: SPPath,
    pub int_to_plc_5: SPPath,
    pub initial_state: SPState,
}



impl PLCResource {
    pub fn new(resource: &mut Resource, int_from_domain: [Vec<SPValue>; 5], int_to_domain: [Vec<SPValue>; 5]) -> PLCResource {
        let _name = resource.path().leaf();
        let bool_to_plc_1= resource.add_variable(Variable::new_boolean("command/bool_to_plc_1", VariableType::Command));
        let bool_to_plc_2= resource.add_variable(Variable::new_boolean("command/bool_to_plc_2", VariableType::Command));
        let bool_to_plc_3= resource.add_variable(Variable::new_boolean("command/bool_to_plc_3", VariableType::Command));
        let bool_to_plc_4= resource.add_variable(Variable::new_boolean("command/bool_to_plc_4", VariableType::Command));
        let bool_to_plc_5= resource.add_variable(Variable::new_boolean("command/bool_to_plc_5", VariableType::Command));
        let int_to_plc_1= resource.add_variable(Variable::new("command/int_to_plc_1", VariableType::Command, SPValueType::Int32, int_to_domain[0].clone()));
        let int_to_plc_2= resource.add_variable(Variable::new("command/int_to_plc_2", VariableType::Command, SPValueType::Int32, int_to_domain[1].clone()));
        let int_to_plc_3= resource.add_variable(Variable::new("command/int_to_plc_3", VariableType::Command, SPValueType::Int32, int_to_domain[2].clone()));
        let int_to_plc_4= resource.add_variable(Variable::new("command/int_to_plc_4", VariableType::Command, SPValueType::Int32, int_to_domain[3].clone()));
        let int_to_plc_5= resource.add_variable(Variable::new("command/int_to_plc_5", VariableType::Command, SPValueType::Int32, int_to_domain[4].clone()));
        let bool_from_plc_1= resource.add_variable(Variable::new_boolean("measured/bool_from_plc_1", VariableType::Measured));
        let bool_from_plc_2= resource.add_variable(Variable::new_boolean("measured/bool_from_plc_2", VariableType::Measured));
        let bool_from_plc_3= resource.add_variable(Variable::new_boolean("measured/bool_from_plc_3", VariableType::Measured));
        let bool_from_plc_4= resource.add_variable(Variable::new_boolean("measured/bool_from_plc_4", VariableType::Measured));
        let bool_from_plc_5= resource.add_variable(Variable::new_boolean("measured/bool_from_plc_5", VariableType::Measured));
        let int_from_plc_1= resource.add_variable(Variable::new("measured/int_from_plc_1", VariableType::Measured, SPValueType::Int32, int_from_domain[0].clone()));
        let int_from_plc_2= resource.add_variable(Variable::new("measured/int_from_plc_2", VariableType::Measured, SPValueType::Int32, int_from_domain[1].clone()));
        let int_from_plc_3= resource.add_variable(Variable::new("measured/int_from_plc_3", VariableType::Measured, SPValueType::Int32, int_from_domain[2].clone()));
        let int_from_plc_4= resource.add_variable(Variable::new("measured/int_from_plc_4", VariableType::Measured, SPValueType::Int32, int_from_domain[3].clone()));
        let int_from_plc_5= resource.add_variable(Variable::new("measured/int_from_plc_5", VariableType::Measured, SPValueType::Int32, int_from_domain[4].clone()));

        resource.setup_ros_outgoing("command", "/opc_command", MessageType::Json,
            &[
                MessageVariable::new(&bool_to_plc_1, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_to_plc_1"),
                MessageVariable::new(&bool_to_plc_2, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_to_plc_2"),
                MessageVariable::new(&bool_to_plc_3, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_to_plc_3"),
                MessageVariable::new(&bool_to_plc_4, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_to_plc_4"),
                MessageVariable::new(&bool_to_plc_5, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_to_plc_5"),
                MessageVariable::new(&int_to_plc_1, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_to_plc_1"),
                MessageVariable::new(&int_to_plc_2, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_to_plc_2"),
                MessageVariable::new(&int_to_plc_3, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_to_plc_3"),
                MessageVariable::new(&int_to_plc_4, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_to_plc_4"),
                MessageVariable::new(&int_to_plc_5, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_to_plc_5"),
            ]
        );
        resource.setup_ros_incoming("measured", "/opc_measured", MessageType::Json,
            &[
                MessageVariable::new(&bool_from_plc_1, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_from_plc_1"),
                MessageVariable::new(&bool_from_plc_2, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_from_plc_2"),
                MessageVariable::new(&bool_from_plc_3, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_from_plc_3"),
                MessageVariable::new(&bool_from_plc_4, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_from_plc_4"),
                MessageVariable::new(&bool_from_plc_5, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_from_plc_5"),
                MessageVariable::new(&int_from_plc_1, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_from_plc_1"),
                MessageVariable::new(&int_from_plc_2, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_from_plc_2"),
                MessageVariable::new(&int_from_plc_3, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_from_plc_3"),
                MessageVariable::new(&int_from_plc_4, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_from_plc_4"),
                MessageVariable::new(&int_from_plc_5, "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_from_plc_5"),
            ]
        );

        let initial_state =  SPState::new_from_values(&[
            (bool_from_plc_1.clone(), SPValue::Bool(false)),
            (bool_from_plc_2.clone(), SPValue::Bool(false)),
            (bool_from_plc_3.clone(), SPValue::Bool(false)),
            (bool_from_plc_4.clone(), SPValue::Bool(false)),
            (bool_from_plc_5.clone(), SPValue::Bool(false)),
            (int_from_plc_1.clone(), int_from_domain[0][0].clone()),
            (int_from_plc_2.clone(), int_from_domain[1][0].clone()),
            (int_from_plc_3.clone(), int_from_domain[2][0].clone()),
            (int_from_plc_4.clone(), int_from_domain[3][0].clone()),
            (int_from_plc_5.clone(), int_from_domain[4][0].clone()),
            (bool_to_plc_1.clone(), SPValue::Bool(false)),
            (bool_to_plc_2.clone(), SPValue::Bool(false)),
            (bool_to_plc_3.clone(), SPValue::Bool(false)),
            (bool_to_plc_4.clone(), SPValue::Bool(false)),
            (bool_to_plc_5.clone(), SPValue::Bool(false)),
            (int_to_plc_1.clone(), int_to_domain[0][0].clone()),
            (int_to_plc_2.clone(), int_to_domain[1][0].clone()),
            (int_to_plc_3.clone(), int_to_domain[2][0].clone()),
            (int_to_plc_4.clone(), int_to_domain[3][0].clone()),
            (int_to_plc_5.clone(), int_to_domain[4][0].clone()),
        ]);

        PLCResource {
            path: resource.path().clone(),
            bool_from_plc_1,
            bool_from_plc_2,
            bool_from_plc_3,
            bool_from_plc_4,
            bool_from_plc_5,
            int_from_plc_1,
            int_from_plc_2,
            int_from_plc_3,
            int_from_plc_4,
            int_from_plc_5,
            bool_to_plc_1,
            bool_to_plc_2,
            bool_to_plc_3,
            bool_to_plc_4,
            bool_to_plc_5,
            int_to_plc_1,
            int_to_plc_2,
            int_to_plc_3,
            int_to_plc_4,
            int_to_plc_5,
            initial_state,
        }

    }

    // To be implemented
    pub fn command_transition (
        &self,
        model: &mut Model,
    ) {
        let r = model.get_resource(&self.path);


    }


}
