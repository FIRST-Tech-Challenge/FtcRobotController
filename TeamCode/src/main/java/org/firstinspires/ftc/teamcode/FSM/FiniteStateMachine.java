package org.firstinspires.ftc.teamcode.FSM;

import java.util.ArrayList;

public class FiniteStateMachine {
    private State current_state, last_state;

    public static void change_state(State target_state){
        target_state.do_action();
    }
}
