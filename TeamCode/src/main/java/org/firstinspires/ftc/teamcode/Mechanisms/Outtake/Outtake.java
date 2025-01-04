package org.firstinspires.ftc.teamcode.Mechanisms.Outtake;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Actuators.ServoAdvanced;

public class Outtake {
    HardwareMap hardwareMap;
    ServoAdvanced shoulderLeft;
    ServoAdvanced shoulderRight;
    ServoAdvanced wristPitch;
    ServoAdvanced wristRoll;
    ServoAdvanced linkageLeft;
    ServoAdvanced linkageRight;
    public static double shoulderUp = 0;
    public static double shoulderDown = 0;
    public static double shoulderFront = 0;
    public static double shoulderBack = 0;
    public static double wristPitchUp = 0;
    public static double wristPitchDown = 0;
    public static double wristPitchMid = 0;
    public static double wristRollNormal = 0;
    public static double wristRollReverse = 0;
    public static double linkageFront = 0;
    public static double linkageBack = 0;
    public static double linkageMid = 0;
    public Outtake(HardwareMap hardwareMap){
        //will do later
    }

    public Action sampleCollect(){
        return new ParallelAction(

        );
    }
    public Action sampleDeposit(){
        return new ParallelAction(

        );
    }
    public Action specimenCollect(){
        return new ParallelAction(

        );
    }
    public Action specimenDeposit(){
        return new ParallelAction(

        );
    }
}
