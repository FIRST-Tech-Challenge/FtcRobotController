package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class Shooter {
    private final RobotHardwareMap robotHardwareMap;
    private final OpMode opMode;
    //    RobotHardwareMap robotHardwareMap;
    DcMotorEx intake;
    private DcMotorEx launcherPollen;

    public DcMotorEx getMotor() {
        return motor;
    }

    public void setMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    private DcMotorEx motor;
//    private double speed;


    public enum Power {
        LOW,
        MEDIUM,
        HIGH
    }
    double previousSpeed = 0;
    private boolean running = true;
    private Power powerLevel = Power.LOW;
    public Shooter(RobotHardwareMap robotHardwareMap, OpMode opMode) {
        this.robotHardwareMap = robotHardwareMap;
        this.opMode = opMode;
    }

    public void init(){
//        launcherPollen = robotHardwareMap.launcherPollen;
//        this.motor = robotHardwareMap.baseHardwareMap.tryGet(DcMotorEx.class,"launch_pollen");
        this.motor = robotHardwareMap.launcherPollen;
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void setMotorSpeed(double speed){
        this.motor.setPower(speed);
    }
    public void setVelocity(double angularSpeed){
        this.motor.setVelocity(angularSpeed);
    }
    public void stop(){
        this.motor.setPower(0);
    }

    public void setPowerLevel(Power power){
//        if (power == Power.HIGH){
//            this.setMotorSpeed(1);
//        } else if (power == Power.MEDIUM){
//            this.setMotorSpeed(.75);
//
//        } else if (power == Power.LOW) {
//            this.setMotorSpeed(.5);
//        } else {
//            this.setMotorSpeed(0);
//        }
    }
    public void toggleShooter(double motorSpeed) {
////        if (launcherPollen.getPower() > 0){
////            running = true;
////        }
//        running= !running;
//        if (running){
//            setMotorSpeed(motorSpeed);
//        } else {
//            setMotorSpeed(0);
//        }
    }

    public boolean isRunning() {
        return running;
    }
}
