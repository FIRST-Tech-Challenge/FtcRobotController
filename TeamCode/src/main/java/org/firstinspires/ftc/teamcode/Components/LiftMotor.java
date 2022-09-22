package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LiftMotor {
    private DcMotorEx liftMotor= null;
    public LiftMotor(){
        // hardware map
    }
    public enum liftStates{
        LIFT_HIGH_JUNCTION(0, "LIFT_HIGH_JUNCTION"),
        LIFT_MED_JUNCTION(0, "LIFT_MED_JUNCTION"),
        LIFT_LOW_JUNCTION(0, "LIFT_LOW_JUNCTION"),
        LIFT_GROUND_JUNCTION(0,"LIFT_GROUND_JUNCTION");

        int goal;
        String name;

        liftStates(int value, String name) {
            this.goal = value;
        }

        public void setGoal(int goal) {
            this.goal = goal;
        }
    }
    public void lift(liftStates lifter){
        //use rfmotor setPosition function to lift in accordance with the enum
    }
}
