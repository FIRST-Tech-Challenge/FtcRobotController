package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class HorizontalArm {

    private final Servo handLeftServo, handRightServo, handRotateServo, armExtendServo;
    private static final double openHandLeftPosition = 0;
    private static final double closeHandLeftPosition = 0.3;
    private static final double openHandRightPosition = 1;
    private static final double closeHandRightPosition = 0.7;
    private static final double minimumExtension = 0.7;
    private static final double maximumExtension = 0.3;

    public HorizontalArm(HardwareMap hardwareMap) {
        handLeftServo = hardwareMap.get(Servo.class, "Horiz_Left");
        handRightServo = hardwareMap.get(Servo.class, "Horiz_Right");
        handRotateServo = hardwareMap.get(Servo.class, "Horiz_Rotate");
        armExtendServo = hardwareMap.get(Servo.class, "Horiz_Extend");

        armExtendServo.setDirection(Servo.Direction.REVERSE);
    }

    public void openHand() {
        handLeftServo.setPosition(openHandLeftPosition);
        handRightServo.setPosition(openHandRightPosition);
    }

    public void closeHand() {
        handLeftServo.setPosition(closeHandLeftPosition);
        handRightServo.setPosition(closeHandRightPosition);
    }

    public void armExtensionDistance(double desiredExtension) {
        double desiredPosition = 0;
        if (desiredPosition == 0) {desiredPosition = 0;
        }
        else {desiredPosition = 0.7 - (0.4/(desiredExtension/14));
        }

        if (desiredPosition < minimumExtension) { desiredPosition = minimumExtension; }
        if (desiredPosition > maximumExtension) { desiredPosition = maximumExtension; }

        armExtendServo.setPosition(desiredPosition);
    }
    
    public void rotateHandUp() {
        handRotateServo.setPosition(0.3);
    }

    public void rotateHandDown() {
        handRotateServo.setPosition(0);
    }



}
