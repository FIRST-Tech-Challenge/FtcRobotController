package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class NectarShooter {
    RobotHardwareMap robotHardwareMap;
    DcMotorEx intake;
    DcMotorEx launcherPollen;
    private DcMotorEx shooter;
    private double speed;


    public enum Power {
        LOW,
        MEDIUM,
        HIGH
    }
    double previousSpeed = 0;
    private boolean running = true;
    private Power powerLevel = Power.LOW;
    public NectarShooter(RobotHardwareMap robotHardwareMap) {
        this.robotHardwareMap = robotHardwareMap;



    }

    public void init(HardwareMap haMap,String name){
//        launcherPollen = robotHardwareMap.launcherPollen;
        this.shooter = robotHardwareMap.launcherNectar;
        this.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setMotorSpeed(double speed){
        this.shooter.setPower(speed);
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
//        if (launcherPollen.getPower() > 0){
//            running = true;
//        }
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
