package org.firstinspires.ftc.teamcode.FSM;

public abstract class State {
    public State(){

    }

    public abstract void state_action();
    public void do_action(){
        state_action();
        return;
    }


}
