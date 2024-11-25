package org.firstinspires.ftc.teamcode.robot.Hobbes.helpers;

public class LinkedState {
    public String nextState;
    public int timeout;
    public LinkedState(String nextStateMacroName, int nextStateTimeoutMS) {
        nextState = nextStateMacroName;
        timeout = nextStateTimeoutMS;
    }
}
