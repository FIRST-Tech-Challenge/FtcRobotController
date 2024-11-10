package com.kalipsorobotics.code2023;

import static java.lang.Math.toRadians;

import com.kuriosityrobotics.shuttle.hardware.ServoControl;
import com.qualcomm.robotcore.hardware.Servo;

import com.kalipsorobotics.utilities.OpModeUtilities;

class OuttakeArmPivot extends ServoControl {
    private static final double ARM_SERVO_SPEED = toRadians(240); // per second
    private static final double ARM_SERVO_RANGE = toRadians(300);
    public static final boolean FLIP_DIRECTION = true;
    public static final double ZERO_POSITION = toRadians(90);

    private OuttakeArmPivot(Servo servo) {
        super(servo, ARM_SERVO_SPEED, ARM_SERVO_RANGE, FLIP_DIRECTION, ZERO_POSITION);
    }
    public OuttakeArmPivot(OpModeUtilities opModeUtilities) {
        this(opModeUtilities.getHardwareMap().servo.get("arm"));
    }

    public void goToOuttakePos() throws InterruptedException {
        goToAngle(toRadians(90));
    }
    public void goToZeroPos() throws InterruptedException {
        goToAngle(toRadians(0));
    }
}
