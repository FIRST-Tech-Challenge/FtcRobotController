package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Subsystem;

public class WobbleSubsystem extends Subsystem {

    private UpliftRobot robot;
    public Servo wobbleLeft;
    public Servo wobbleRight;
    public Servo clamp;

    public WobbleSubsystem(UpliftRobot robot) {
        super(robot);
        this.robot = robot;
        this.wobbleLeft = robot.wobbleLeft;
        this.wobbleRight = robot.wobbleRight;
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

    public void setWobblePosition(double pos) {
        wobbleLeft.setPosition(1 - pos);
        wobbleRight.setPosition(pos);
    }

    public void pickUp() {
        closeWobble();
        liftWobble();
    }
    public void pickUpTeleop() {
        closeWobble();
        robot.safeSleep(500);
        highWobble();
    }
    public void dropOff() {
        dropWobble();
        openWobble();
    }

    public void highWobble() {
        setWobblePosition(0.85);
        robot.safeSleep(500);
    }

    public void liftWobble() {
        setWobblePosition(0.2);
        robot.safeSleep(300);
    }

    public void dropWobble() {
        setWobblePosition(0);
        robot.safeSleep(200);
    }

    public void endgameWobble() {
        setWobblePosition(0.5);
        robot.safeSleep(500);
    }

    public void closeWobble() {
        clamp.setPosition(0.03);
        robot.safeSleep(500);
    }

    public void openWobble() {
        clamp.setPosition(0.4);
        robot.safeSleep(500);
    }





}
