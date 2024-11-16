package org.firstinspires.ftc.teamcode;

public class StateMachine {
    String[] states;
    int currentState;
    public StateMachine(String[] states, int stateNum)
    {
        this.states = states;
        currentState = stateNum;
    }
    public void nextState() {
        currentState++;
    }
    public void setState(int n) {
        currentState = n;
    }
    public String getState() {
        return states[currentState];
    }
}
