package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;

public class LiftArm {
    public DcMotor slide, hand;
    public Servo trapdoor;
    public Servo purplePixel;


    public Servo plane;

    public HandPosition handPosition;
    public double intakePower = 0;

    // Used to set pole height.
    public enum Distance {
        DEFAULT,
        AUTO,
        HALF,
        FULL,
        ENDGAMESTART,
        ENDGAMEHOLD
    }

    public enum HandPosition{
        IN,
        OUT
    }


    //Constructor
    public LiftArm(HardwareMap hardwareMap) {
        //Initialize motors and servos
        hand = hardwareMap.get(DcMotor.class, "hand");
        slide = hardwareMap.get(DcMotor.class, "slide");
        trapdoor = hardwareMap.get(Servo.class, "trapdoor");
        plane = hardwareMap.get(Servo.class, "plane");
        purplePixel = hardwareMap.get(Servo.class, "purple");

        closeTrapdoor();
        reloadPlane();
        purplePixel.setPosition(Servo.MAX_POSITION);

        //Set motor directions
        hand.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);

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
                slide.setTargetPosition(1800);
                break;
            case HALF:
                slide.setTargetPosition(4400);
                break;
            case FULL:
                slide.setTargetPosition(8000);
                break;
            case ENDGAMESTART:
                slide.setTargetPosition(7500);
                break;
            case ENDGAMEHOLD:
                slide.setTargetPosition(1000);
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
                hand.setTargetPosition(150);
                break;
        }
        hand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        float endPos = hand.getTargetPosition();
        if (endPos == 150) {
            hand.setPower(0.25);
        } else if (endPos == 0) {
            hand.setPower(0.25);
        }
    }



    public void openTrapdoor() {
        trapdoor.setPosition(0.2);
    }
    public void closeTrapdoor() {
        trapdoor.setPosition(0.7);
    }

    public void reloadPlane() {
        plane.setPosition(-0.2);
    }
    public void launchPlane() {
        plane.setPosition(.75);
    }


    public void dropPurplePixel() { purplePixel.setPosition(Servo.MIN_POSITION);
    }


    public void extend(Distance targetDistance) {
        setHandPosition(HandPosition.OUT);
        setArmDistance(targetDistance);
    }

    public void retract() {
        closeTrapdoor();
        setArmDistance(Distance.DEFAULT);
        setHandPosition(HandPosition.IN);
    }

    public void holdArm() {
        if (!slide.isBusy()) {
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide.setPower(0.05);
        }
    }

    public void holdHang() {
        if (!slide.isBusy()) {
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide.setPower(0.25);
        }
    }

    public double getSlidePosition() {
        return slide.getCurrentPosition();
    }

    public double getHandPosition() {
        return hand.getCurrentPosition();
    }
}