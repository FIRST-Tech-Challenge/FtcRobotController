package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

public class Lift {
    //TODO: RFMotor
    private RFMotor liftMotor = null;
    public Lift(){ //constructor
        // hardware map
    }
    public enum liftStates{
//        LIFT_HIGH_JUNCTION(0, "LIFT_HIGH_JUNCTION"),
//        LIFT_MED_JUNCTION(0, "LIFT_MED_JUNCTION"),
//        LIFT_LOW_JUNCTION(0, "LIFT_LOW_JUNCTION"),
//        LIFT_GROUND_JUNCTION(0,"LIFT_GROUND_JUNCTION");
//
//        int goal;
//        String name;
//
//        liftStates(int value, String name) {
//            this.goal = value;
//        }
//
//        public void setGoal(int goal) {
//            this.goal = goal;
//        }
        //make enum for all the tick counts for ground low med high junctions, can set with setGoal(int goal);
    }
    public void liftToJunction(liftStates junctions){//TODO: make sure this is async
        //use rfmotor setPosition function to lift in accordance with the enum
        // no conditions
        // log when movement starts & when reach target position
        //async, no use sleep/wait with time, can use multiple processes
    }
}
