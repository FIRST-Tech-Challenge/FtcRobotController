package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LiftArm {
    public DcMotor slide, hand;
    public Servo trapdoor;
    public Servo intakeHolder;

    public Servo lifterL;
    public Servo lifterR;


    public HandPosition handPosition;
    public double intakePower = 0;

    // Used to set pole height.
    public enum Distance {
        DEFAULT,
        AUTO,
        HALF,
        FULL
    }

    public enum HandPosition{
        IN,
        OUT
    }


    public enum TrapdoorPositions{
        OPEN,
        CLOSE
    }

    public enum lifterPositions{
        OPEN,
        CLOSE
    }

    public enum holderPositions{
        OPEN,
        CLOSE
    }

    // Used to identify left and right slide motors
    public enum Sides {
        RIGHT,
        LEFT
    }


    //Constructor
    public LiftArm(HardwareMap hardwareMap) {
        //Initialize motors and servos
        hand = hardwareMap.get(DcMotor.class, "hand");
        slide = hardwareMap.get(DcMotor.class, "slide");
        trapdoor = hardwareMap.get(Servo.class, "trapdoor");
        intakeHolder = hardwareMap.get(Servo.class, "intakeHolder");

        lifterL = hardwareMap.get(Servo.class, "lifterL");
        lifterR = hardwareMap.get(Servo.class, "lifterR");

        intakeHolder.setDirection(Servo.Direction.REVERSE);

        secureHook();

        trapdoor.setPosition(.2);

        //Set motor directions
        hand.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.REVERSE);

        //Reset encoders
        hand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setHandPosition(HandPosition.IN);
    }



    public void setArmDistance(Distance targetDistance) {
        switch (targetDistance) {
            case DEFAULT:
                slide.setTargetPosition(0);
                break;
            case AUTO:
                slide.setTargetPosition(2000);
                break;
            case HALF:
                slide.setTargetPosition(5000);
                break;
            case FULL:
                slide.setTargetPosition(8000);
                break;
        }
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
    }



    public void setHandPosition(HandPosition targetHandPosition) {
        handPosition = targetHandPosition;
        switch (targetHandPosition) {
            case IN:
                hand.setTargetPosition(0);
                break;
            case OUT:
                hand.setTargetPosition(-140);
                break;
        }
        hand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hand.setPower(1);

    }



    public void openTrapdoor() {
        trapdoor.setPosition(0.7);
    }
    public void closeTrapdoor() {
        trapdoor.setPosition(0.2);
    }

    public void releaseHook() {
        lifterR.setPosition(Servo.MIN_POSITION);
        lifterL.setPosition(Servo.MIN_POSITION);
    }
    public void secureHook() {
        lifterR.setPosition(Servo.MAX_POSITION);
        lifterL.setPosition(Servo.MAX_POSITION);
    }

    public void extendAndDrop(Distance targetDistance) {
        setArmDistance(targetDistance);
        setHandPosition(HandPosition.OUT);
    }

    public void retract() {
        closeTrapdoor();
        setHandPosition(HandPosition.IN);
        setArmDistance(Distance.DEFAULT);
    }

    public void holdArm() {
        if (!slide.isBusy()) {
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide.setPower(0.05);
        }
    }


    public double getSlidePosition() {
        return slide.getCurrentPosition();
    }

    public double getHandPosition() {
        return hand.getCurrentPosition();
    }
}