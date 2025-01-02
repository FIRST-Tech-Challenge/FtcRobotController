package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class HorizontalArm {

    private final Servo handLeftServo, handRightServo, handRotateServo, armExtendServo;
    private static final double openHandLeftPosition = 0;
    private static final double closeHandLeftPosition = 0.3;
    private static final double openHandRightPosition = 1;
    private static final double closeHandRightPosition = 0.7;
    private static final double minimumPosition = 0.7;
    private static final double maximumPosition = 0.3;
    private static final double extensionRange = 18;
    private static final double positionRange = 0.4;

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

    public void moveToExtensionDistance(double desiredExtension) {
        double desiredPosition = maximumPosition + (positionRange * (desiredExtension / extensionRange));

        if (desiredPosition > minimumPosition) { desiredPosition = minimumPosition; }
        if (desiredPosition < maximumPosition) { desiredPosition = maximumPosition; }

        armExtendServo.setPosition(desiredPosition);
    }
    
    public void rotateHandUp() {
        handRotateServo.setPosition(0.3);
    }

    public void rotateHandDown() {
        handRotateServo.setPosition(0);
    }
}
