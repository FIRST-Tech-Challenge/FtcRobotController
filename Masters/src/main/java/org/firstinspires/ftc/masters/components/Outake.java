package org.firstinspires.ftc.masters.components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.annotation.Target;

@Config
public class Outake implements Component{

    private PIDController controller;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double p = 0.00057, i = 0, d = 0.000000015;
    public static double f = 0.002;

    public int target = 0;

    private final Servo claw;
    private final Servo wrist;
//    private final DcMotor extension1;
//    private final DcMotor extension2;


    Telemetry telemetry;
    Init init;

    public Outake(Init init, Telemetry telemetry){

        this.init=init;
        this.telemetry=telemetry;

        claw= init.getClaw();
        wrist= init.getWrist();
//        this.extension1=init.getExtension1();
//        this.extension2=init.getExtension2();
//        this.elbow1=init.getElbow1();
//        this.elbow2=init.getElbow2();
//        this.fingers=init.getFingers();
        initializeHardware();

    }

    public void initializeHardware() {

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        target = 0;

        claw.setPosition(ITDCons.close);

    }

    public void setPID(double p){
        controller.setP(p);
    }

//    public void slidePower(double power) {
//        extension1.setPower(power);
//        extension2.setPower(power);
//    }


    public void openClaw() {
        claw.setPosition(ITDCons.open);
    }

    public void closeClaw() {
        claw.setPosition(ITDCons.close);
    }

    public void setWristFront(){
        wrist.setPosition(ITDCons.wristFront);
    }

    public void setWristBack(){
        wrist.setPosition(ITDCons.wristBack);
    }

//    public void moveSlide(int target) {
//
//        this.target=target;
//
//        int rotatePos = extension2.getCurrentPosition();
//        double pid = controller.calculate(rotatePos, target);
//        double lift = pid + f;
//
//        extension1.setPower(lift);
//        extension2.setPower(lift);
//
//    }
//
//    public int getTarget(){
//        return target;
//    }
//
//    public int getExtensionPos(){
//        return extension2.getCurrentPosition();
//    }

    public void init(){
//        elbow1.setPosition(ITDCons.diffyInit);
//        elbow2.setPosition(ITDCons.diffyInit);

    }


}
