package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
public class LiftPIDController {

    DcMotorEx mainSlideMotor, slideMotor2, slideMotor3;
    PIDController liftController;
    int target =0;
    public static double multiplier = 0.5;
    public static double multiplierZero = 0.09;
    public static double p=0.04, i=0, d=0.0001;
    public static double f=0.06;

    public LiftPIDController (DcMotorEx mainSlideMotor, DcMotorEx slideMotor2, DcMotorEx slideMotor3){
        this.mainSlideMotor = mainSlideMotor;
        this.slideMotor2 = slideMotor2;
        this.slideMotor3 = slideMotor3;
        liftController = new PIDController(p, i,d);
    }

    public void setTarget(int target){
        this.target = target;
    }

    public double calculatePower(){

        liftController.setPID(p, i, d);
        int liftPos = mainSlideMotor.getCurrentPosition();
        double pid = liftController.calculate(liftPos, target);

        double power = pid +f;

       if (target<150){
           power = power* multiplierZero;
       } else{
           power = power*multiplier;
       }

        return power;
    }

    public double calculatePower(DcMotorEx motor){

        liftController.setPID(p, i, d);
        int liftPos = motor.getCurrentPosition();
        double pid = liftController.calculate(liftPos, target);

        double power = pid +f;

        if (target<150){
            power = power* multiplierZero;
        } else{
            power = power*multiplier;
        }

        return power;
    }


}
