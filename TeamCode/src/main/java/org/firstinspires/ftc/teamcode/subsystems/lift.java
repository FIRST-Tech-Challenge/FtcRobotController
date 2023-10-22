package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDFController;
public class lift {
    public PIDFController pidf;
    public static int pos;
    public DcMotor lift1;
    public DcMotor lift2;
    public lift(HardwareMap hwmap){
        lift1=hwmap.get(DcMotor.class,"lift1");
        //lift2=hwmap.get(DcMotor.class,"lift2");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     // lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pidf.setTolerance(5);
        pidf.setPIDF(0.004,0,0.005,0.00000001);
        pos = lift1.getCurrentPosition();
    }
    public void setHeight(int ticks){

    }
    public void setTilt(int ticks){

    }
    public void run(){

    }

}
