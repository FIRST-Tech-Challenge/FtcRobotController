package com.kalipsorobotics.fresh.mechanisms;

import static java.lang.Math.toRadians;

import com.kalipsorobotics.fresh.OpModeUtilities;
import com.kuriosityrobotics.shuttle.hardware.ServoControl;
import com.qualcomm.robotcore.hardware.Servo;

public class LinearLocker extends ServoControl {
    private static final double ARM_SERVO_SPEED = toRadians(240); // per second
    private static final double ARM_SERVO_RANGE = toRadians(300);
    public static final boolean FLIP_DIRECTION = true;
    public static final double ZERO_POSITION = toRadians(111);

    private LinearLocker(Servo servo) {
        super(servo, ARM_SERVO_SPEED, ARM_SERVO_RANGE, FLIP_DIRECTION, ZERO_POSITION);
    }

    public LinearLocker(OpModeUtilities opModeUtilities) {
        this(opModeUtilities.getHardwareMap().servo.get("linearLocker"));
    }

    public void close() throws InterruptedException {
        goToAngle(toRadians(30));
    }
    public void open() throws InterruptedException {
        goToAngle(toRadians(0));
    }
}
