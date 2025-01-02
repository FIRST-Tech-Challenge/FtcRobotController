package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class VerticalArm {

    private final Servo handLeftServo, handRightServo;
    private final DcMotor armMotor;
    private static final int defaultStartingPosition = 0;
    private static final int defaultPower = 1;
    private static final double diameterOffset = 0;
    private static final double diameter = 1.5 + diameterOffset;
    private static final double circumference = Math.PI * diameter;
    private static final double minimumHeight = 1.5;
    private static final double maximumHeight = 30;
    private static final double countsPerRevolution = 537.689839572;
    private static final double openHandLeftPosition = 0;
    private static final double closeHandLeftPosition = 0.2;
    private static final double openHandRightPosition = 1;
    private static final double closeHandRightPosition = 0.8;
    private static final long defaultSleep = 100;

    public VerticalArm(HardwareMap hardwareMap) {
        handLeftServo = hardwareMap.get(Servo.class, "handLeft");
        handRightServo = hardwareMap.get(Servo.class, "handRight");

        armMotor = hardwareMap.get(DcMotor.class, "armExtend");
        initializeArmMotor();
    }

    public void moveToHeight(double desiredHeight, boolean shouldWait){
        // guard inputs
        if (desiredHeight < minimumHeight) { desiredHeight = minimumHeight; }
        if (desiredHeight > maximumHeight) { desiredHeight = maximumHeight; }

        double desiredDistance = desiredHeight - minimumHeight;
        double desiredRevolutions = desiredDistance / circumference;
        double desiredPosition = desiredRevolutions * countsPerRevolution;

        armMotor.setTargetPosition((int)Math.round(desiredPosition));

        if (shouldWait)
            while (armMotor.isBusy()) { sleep(defaultSleep); }
    }

    public void openHand() {
        handLeftServo.setPosition(openHandLeftPosition);
        handRightServo.setPosition(openHandRightPosition);
    }

    public void closeHand() {
        handLeftServo.setPosition(closeHandLeftPosition);
        handRightServo.setPosition(closeHandRightPosition);
    }

    private void initializeArmMotor(){
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setTargetPosition(defaultStartingPosition);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(defaultPower);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
