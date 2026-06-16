package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private static final String LEFT_FRONT_MOTOR_NAME = "LeftFrontMotor";
    private static final String RIGHT_FRONT_MOTOR_NAME = "RightFrontMotor";
    private static final String BACK_MOTOR_NAME = "BackMotor";

    private final DcMotor leftFrontMotor;
    private final DcMotor rightFrontMotor;
    private final DcMotor backMotor;

    public Drivetrain(HardwareMap hardwareMap) {
        leftFrontMotor = hardwareMap.get(DcMotor.class, LEFT_FRONT_MOTOR_NAME);
        rightFrontMotor = hardwareMap.get(DcMotor.class, RIGHT_FRONT_MOTOR_NAME);
        backMotor = hardwareMap.get(DcMotor.class, BACK_MOTOR_NAME);

        setDirections();
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(double drive, double turn) {
        double leftFrontPower = drive + turn;
        double rightFrontPower = drive - turn;

        double maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
        }

        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        backMotor.setPower(clip(drive));
    }

    public void stop() {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        backMotor.setPower(0);
    }

    private void setDirections() {
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftFrontMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rightFrontMotor.setZeroPowerBehavior(zeroPowerBehavior);
        backMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    private void setRunMode(DcMotor.RunMode runMode) {
        leftFrontMotor.setMode(runMode);
        rightFrontMotor.setMode(runMode);
        backMotor.setMode(runMode);
    }

    private double clip(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}
