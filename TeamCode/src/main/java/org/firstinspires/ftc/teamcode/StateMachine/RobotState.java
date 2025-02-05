package org.firstinspires.ftc.teamcode.StateMachine;


import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.R;

import lombok.Getter;
import lombok.Setter;

public class RobotState {
    @Getter
    @Setter
    private Constants.States state = Constants.States.SAMPLE;
    private static RobotState instance;
    public static RobotState getInstance(){
        if(instance == null) instance = new RobotState();
        return instance;
    }

}
