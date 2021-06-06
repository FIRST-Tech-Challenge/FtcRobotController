package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.UpliftRobot;

public class WobbleFunctions {

    UpliftRobot robot;
    Servo wobbleLeft;
    Servo wobbleRight;
    Servo clamp;

    public WobbleFunctions(UpliftRobot robot) {
        this.robot = robot;
        this.wobbleLeft = robot.wobbleLeft;
        this.wobbleRight = robot.wobbleRight;
        this.clamp = robot.clamp;
    }

    public void setWobblePosition(double pos) {
        wobbleLeft.setPosition(1 - pos);
        wobbleRight.setPosition(pos);
    }

    public void pickUp() {
        closeWobble();
        liftWobble();
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
