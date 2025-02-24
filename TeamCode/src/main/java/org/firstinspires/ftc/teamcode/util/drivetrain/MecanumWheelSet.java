package org.firstinspires.ftc.teamcode.util.drivetrain;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.HashMap;

public class MecanumWheelSet {
    private final HashMap<MecanumWheel, MotorEx> motors;

    public MecanumWheelSet(
            MotorEx frontLeft, MotorEx frontRight,
            MotorEx backLeft, MotorEx backRight
    ) {
        this.motors = new HashMap<MecanumWheel, MotorEx>(){{
            put(MecanumWheel.FRONT_LEFT, frontLeft);
            put(MecanumWheel.FRONT_RIGHT, frontRight);
            put(MecanumWheel.BACK_LEFT, backLeft);
            put(MecanumWheel.BACK_RIGHT, backRight);
        }};
    }

    private MotorEx getMotor(MecanumWheel wheel) {
        MotorEx motor = this.motors.get(wheel);
        if(motor == null) throw new RuntimeException("Mecanum motor '" + wheel.name() + "' is not initialized.");
        return motor;
    }

    public void setPower(double power) {
        this.setPower(MecanumWheel.FRONT_LEFT, power);
        this.setPower(MecanumWheel.FRONT_RIGHT, power);
        this.setPower(MecanumWheel.BACK_LEFT, power);
        this.setPower(MecanumWheel.BACK_RIGHT, power);
    }

    public void setSidePower(double left, double right) {
        this.setPower(MecanumWheel.FRONT_LEFT, left);
        this.setPower(MecanumWheel.FRONT_RIGHT, right);
        this.setPower(MecanumWheel.BACK_LEFT, left);
        this.setPower(MecanumWheel.BACK_RIGHT, right);
    }

    public void setPower(MecanumWheel wheel, double power) {
        this.getMotor(wheel).set(power);
    }

    public void setPower(MecanumChassisUtils.MecanumWheelSpeeds wheelSpeeds) {
        this.setPower(MecanumWheel.FRONT_LEFT, wheelSpeeds.getFrontLeft());
        this.setPower(MecanumWheel.FRONT_RIGHT, wheelSpeeds.getFrontRight());
        this.setPower(MecanumWheel.BACK_LEFT, wheelSpeeds.getBackLeft());
        this.setPower(MecanumWheel.BACK_RIGHT, wheelSpeeds.getBackRight());
    }

    public void setInverted(MecanumWheel wheel, boolean inverted) {
        this.getMotor(wheel).setInverted(inverted);
    }

    public void setZeroPowerBehavior(Motor.ZeroPowerBehavior zeroPowerBehavior) {
        this.setZeroPowerBehavior(MecanumWheel.FRONT_LEFT, zeroPowerBehavior);
        this.setZeroPowerBehavior(MecanumWheel.FRONT_RIGHT, zeroPowerBehavior);
        this.setZeroPowerBehavior(MecanumWheel.BACK_LEFT, zeroPowerBehavior);
        this.setZeroPowerBehavior(MecanumWheel.BACK_RIGHT, zeroPowerBehavior);
    }

    public void setZeroPowerBehavior(MecanumWheel wheel, Motor.ZeroPowerBehavior zeroPowerBehavior) {
        this.getMotor(wheel).setZeroPowerBehavior(zeroPowerBehavior);
    }

    public enum MecanumWheel {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }
}
