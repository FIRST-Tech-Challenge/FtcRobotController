package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.helper;

public class Shooter {
    private final RobotHardwareMap robotHardwareMap;
    private final OpMode opMode;
    //    RobotHardwareMap robotHardwareMap;
    DcMotorEx intake;
    private DcMotorEx launcherPollen;
    double target;
    private static final double ratio = 1;


    public DcMotorEx getMotor() {
        return motor;
    }

    public void setMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    DcMotorEx motor;
//    private double speed;
    private CRServo servo;
    private double rpm = 0;

    boolean hasServo = false;


    public boolean isAtSpeed() {
        rpm = helper.getRPM(this.motor.getVelocity(),1);
        return rpm >= target;
    }


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
        // Once we talk about making init private
//        this.init();
    }


    public boolean isHasServo() {
        return hasServo;
    }

    public void init(){
//        launcherPollen = robotHardwareMap.launcherPollen;
//        this.motor = robotHardwareMap.baseHardwareMap.tryGet(DcMotorEx.class,"launch_pollen");
        this.motor = robotHardwareMap.launcherPollen;
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.servo = robotHardwareMap.getPollenServo();
        if (servo != null){
            hasServo = true;
            servo.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        rpm = this.motor.getVelocity()*60/19.2/ratio;
        target = 4700;
    }
    private double motorSpeed;
    public void setMotorSpeed(double speed){
        this.motorSpeed = speed;
    }
    public void setVelocity(double angularSpeed){
        this.motor.setVelocity(angularSpeed);
    }
    public void stop(){
        this.motor.setPower(0);
    }

    public Power getPowerLevel() {
        return powerLevel;
    }
    public void increasePower(){
        if (Power.LOW == powerLevel){
            powerLevel = Power.MEDIUM;
        }

        if (Power.MEDIUM == powerLevel){
            powerLevel = Power.HIGH;
        }

        setPowerLevel(powerLevel);
//        return powerLevel;
    }
    public void decreasePower(){
        if (Power.HIGH == powerLevel){
            powerLevel = Power.MEDIUM;
        }

        if (Power.MEDIUM == powerLevel){
            powerLevel = Power.LOW;
        }
        setPowerLevel(powerLevel);
//        return powerLevel;
    }

    public void setPowerLevel(Power power){
        powerLevel = power;
        if (power == Power.HIGH){
            this.setMotorSpeed(1);
        } else if (power == Power.MEDIUM){
            this.setMotorSpeed(.75);

        } else if (power == Power.LOW) {
            this.setMotorSpeed(.5);
        } else {
            this.setMotorSpeed(0);
        }
    }
    public void toggleShooter() {
        running = !running;
        if (running){
            this.getMotor().setPower(motorSpeed);
        } else {
            setMotorSpeed(0);
        }
    }
    public void shoot(){
        if (isHasServo()) {
            servo.setPower(.75);
        }
    }
    public void stopShooting(){
        if (isHasServo()) {
            servo.setPower(0);
        }
    }

    public boolean isRunning() {
        return running;
    }
}
