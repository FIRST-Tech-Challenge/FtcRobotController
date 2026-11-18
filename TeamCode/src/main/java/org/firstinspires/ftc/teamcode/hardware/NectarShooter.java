package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class NectarShooter  extends Shooter{
    RobotHardwareMap robotHardwareMap;
    DcMotorEx intake;
    DcMotorEx launcherPollen;
//    private DcMotorEx shooter;
    private double speed;
    private CRServo servo;


//    public enum Power {
//        LOW,
//        MEDIUM,
//        HIGH
//    }
    double previousSpeed = 0;
    private boolean running = true;
    private Power powerLevel = Power.LOW;
    public NectarShooter (RobotHardwareMap robotHardwareMap, OpMode opMode) {
        super(robotHardwareMap,opMode);
//        .robotHardwareMap = robotHardwareMap;
//        this.init();



    }
    @Override
    public void init(){
//        launcherPollen = robotHardwareMap.launcherPollen;
        this.motor = robotHardwareMap.launcherNectar;
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.servo = robotHardwareMap.getNectarServo();
        if (servo != null){
            hasServo = true;
        }
        target = 4700;
    }
//    public void setMotorSpeed(double speed){
//        this.motor.setPower(speed);
//    }
//    public void setPowerLevel(Power power){
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
//    }
//    public void toggleShooter(double motorSpeed) {
//        if (launcherPollen.getPower() > 0){
//            running = true;
//        }
//        running= !running;
//        if (running){
//            setMotorSpeed(motorSpeed);
//        } else {
//            setMotorSpeed(0);
//        }
//    }
//    public void shoot(){
//        if (robotHardwareMap.hasNectarServo()) {
//            servo.setPower(.75);
//        }
//    }
//    public void stopShooting(){
//        if (robotHardwareMap.hasNectarServo()) {
//        servo.setPower(0);
//        }
//    }

//    public boolean isRunning() {
//        return running;
//    }
}
