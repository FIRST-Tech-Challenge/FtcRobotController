package com.kalipsorobotics.fresh.mechanisms;

import static java.lang.Math.toRadians;

import com.kalipsorobotics.fresh.OpModeUtilities;
import com.kuriosityrobotics.shuttle.hardware.ServoControl;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncherServo extends ServoControl {
    private static final double ARM_SERVO_SPEED = toRadians(240); // per second
    private static final double ARM_SERVO_RANGE = toRadians(300);
    public static final boolean FLIP_DIRECTION = true;
    public static final double ZERO_POSITION = toRadians(219);

    private DroneLauncherServo(Servo servo) {
        super(servo, ARM_SERVO_SPEED, ARM_SERVO_RANGE, FLIP_DIRECTION, ZERO_POSITION);
    }
    public DroneLauncherServo(OpModeUtilities opModeUtilities) {
        this(opModeUtilities.getHardwareMap().servo.get("planeLauncherServo"));
    }

    public void disEngage() throws InterruptedException {
        goToAngle(toRadians(0));
    }
    public void engage() throws InterruptedException {
        goToAngle(toRadians(84));
    }
}
