package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.Robots.PwPRobot;

import java.util.ArrayList;

public class Lift {
    //TODO: RFMotor
    private RFMotor liftMotor;
    public Lift(){ //constructor
        // hardware map
        logger.createFile("LiftLog", "Time Junction Ticks");
        double maxTick = 0.0; //idk values yet - RFMotor params
        double minTick = 0.0; //idk values yet - RFMotor params
        ArrayList<Double> coeffs = new ArrayList<>(); // RFMotor params
        liftMotor = new RFMotor("liftMotor", DcMotor.RunMode.RUN_USING_ENCODER, PwPRobot.isTeleop, coeffs, maxTick, minTick);
    }
    public enum liftConstants{
        LIFT_HIGH_JUNCTION(0.0),
        LIFT_MED_JUNCTION(0.0),
        LIFT_LOW_JUNCTION(0.0),
        LIFT_GROUND_JUNCTION(0.0);

        double value;

        boolean status;
        String name;

        liftConstants(boolean value, String name) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }

        liftConstants(double num_of_ticks) {
            value = num_of_ticks;
        }
        //make enum for all the tick counts for ground low med high junctions, can set with setGoal(int goal);
    }
    public void liftToPosition(liftConstants targetHeight){//TODO: make sure this is async
        //use rfmotor setPosition function to lift in accordance with the enum
        liftMotor.setPosition(targetHeight.value);
        // no conditions
        // log when movement starts & when reach target position
        logger.log("LiftLog", "Claw lift to " + targetHeight.name() + ", ticks: " + targetHeight.value);
        //async, no use sleep/wait with time, can use multiple processes
    }
    public void liftToPosition(double targetTickCount){
        liftMotor.setPosition(targetTickCount);
        logger.log("LiftLog", "Claw lift to " + targetTickCount + " ticks");
    }
}
