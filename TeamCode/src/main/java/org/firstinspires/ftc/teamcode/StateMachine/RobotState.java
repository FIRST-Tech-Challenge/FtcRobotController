package org.firstinspires.ftc.teamcode.StateMachine;


import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.R;


public class RobotState {
    public Constants.States getState() {
        return state;
    }

    public void setState(Constants.States state) {
        this.state = state;
    }

    private Constants.States state = Constants.States.SAMPLE;
    private static RobotState instance;
    public static RobotState getInstance(){
        if(instance == null) instance = new RobotState();
        return instance;
    }

}
