package org.firstinspires.ftc.teamcode.FSM;

public class ControllerSM extends FiniteStateMachine{
    public void startNewSequence(final State state_to_operate){
        Runnable sequence = new Runnable() {
            @Override
            public void run() {
                change_state(state_to_operate);
            }
        };
        Thread sequence_thread = new Thread(sequence);
        sequence_thread.start();
    }
}
