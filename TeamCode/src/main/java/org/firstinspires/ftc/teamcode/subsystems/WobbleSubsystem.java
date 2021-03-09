package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Subsystem;

public class WobbleSubsystem extends Subsystem {

    private UpliftRobot robot;
    public Servo wobbleTop;
    public Servo wobbleBottom;
    public Servo clamp;

    public WobbleSubsystem(UpliftRobot robot) {
        super(robot);
        this.robot = robot;
        this.wobbleTop = robot.wobbleTop;
        this.wobbleBottom = robot.wobbleBottom;
        this.clamp = robot.clamp;
    }

    @Override
    public void enable() {

    }

    @Override
    public void disable() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void safeDisable() {
        // ADD SAFE POSITION HERE
    }
    public void pickUp(){
        closeWobble();
        liftWobble();
    }
    public void pickUpTeleop(){
        closeWobble();
        robot.safeSleep(500);
        highWobble();
    }
    public void dropOff(){
        dropWobble();
        openWobble();
    }

    public void highWobble() {
        wobbleTop.setPosition(1 - 0.85);
        wobbleBottom.setPosition(0.85);
        robot.safeSleep(500);
    }

    public void liftWobble() {
        wobbleTop.setPosition(1 - 0.2);
        wobbleBottom.setPosition(0.2);
        robot.safeSleep(500);
    }

    public void dropWobble() {
        wobbleTop.setPosition(1 - 0);
        wobbleBottom.setPosition(0);
        robot.safeSleep(500);
    }

    public void endgameWobble() {
        wobbleTop.setPosition(1 - 0.5);
        wobbleBottom.setPosition(0.5);
        robot.safeSleep(500);
    }

    public void closeWobble() {
        clamp.setPosition(0.8);
        clamp.setPosition(0.3);
        robot.safeSleep(1000);
    }

    public void openWobble() {
        clamp.setPosition(0.8);
        robot.safeSleep(500);
    }





}
