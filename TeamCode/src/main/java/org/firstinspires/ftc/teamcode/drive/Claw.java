package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private static final double MIN_ROT = 0.0;
    private static final double MAX_ROT = 1.0;
    private static final double MIN_CLAW = 0.5;
    private static final double MAX_CLAW = 1.0;
    public final Servo leftRot, rightRot;
    public final Servo leftClaw, rightClaw;

    public Claw(Servo leftRot, Servo rightRot, Servo leftClaw, Servo rightClaw) {
        this.leftRot = leftRot;
        this.rightRot = rightRot;
        this.leftClaw = leftClaw;
        this.rightClaw = rightClaw;
        this.leftRot.setDirection(Servo.Direction.FORWARD);
        this.rightRot.setDirection(Servo.Direction.REVERSE);
        this.leftClaw.setDirection(Servo.Direction.FORWARD);
        this.rightClaw.setDirection(Servo.Direction.REVERSE);
    }

    public void closeAll() {
        this.closeLeft();
        this.closeRight();
    }

    public void closeLeft() {
        this.leftClaw.setPosition(MIN_CLAW);;
    }

    public void closeRight() {
        this.rightClaw.setPosition(MIN_CLAW);;
    }

    public void openLeft() {
        this.leftClaw.setPosition(MAX_CLAW);;
    }

    public void openRight() {
        this.rightClaw.setPosition(MAX_CLAW);;
    }

    public void setRotate(double rotate) {
        rotate = Math.min(Math.max(rotate, MIN_ROT), MAX_ROT);
        this.leftRot.setPosition(rotate);
        this.rightRot.setPosition(rotate);
    }

    public void rotate(double rotate) {
        this.setRotate(this.leftRot.getPosition() + rotate);
    }
}
