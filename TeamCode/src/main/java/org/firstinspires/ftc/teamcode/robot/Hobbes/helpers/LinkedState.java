package org.firstinspires.ftc.teamcode.robot.Hobbes.helpers;

public class LinkedState {
    public HobbesState nextState;
    public int timeout;
    public LinkedState(HobbesState nextStateMacroName, int nextStateTimeoutMS) {
        nextState = nextStateMacroName;
        timeout = nextStateTimeoutMS;
    }
}
