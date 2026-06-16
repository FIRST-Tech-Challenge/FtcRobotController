package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private static final String FRONT_MOTOR_NAME = "FrontMotor";
    private static final String LEFT_BACK_MOTOR_NAME = "LeftBackMotor";
    private static final String RIGHT_BACK_MOTOR_NAME = "RightBackMotor";

    private final DcMotor frontMotor;
    private final DcMotor leftBackMotor;
    private final DcMotor rightBackMotor;

    public Drivetrain(HardwareMap hardwareMap) {
        frontMotor = hardwareMap.get(DcMotor.class, FRONT_MOTOR_NAME);
        leftBackMotor = hardwareMap.get(DcMotor.class, LEFT_BACK_MOTOR_NAME);
        rightBackMotor = hardwareMap.get(DcMotor.class, RIGHT_BACK_MOTOR_NAME);

        setDirections();
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(double drive, double turn) {
        double leftBackPower = drive + turn;
        double rightBackPower = drive - turn;

        double maxPower = Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower));
        if (maxPower > 1.0) {
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }

        frontMotor.setPower(clip(drive));
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
    }

    public void stop() {
        frontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    private void setDirections() {
        frontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        frontMotor.setZeroPowerBehavior(zeroPowerBehavior);
        leftBackMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rightBackMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    private void setRunMode(DcMotor.RunMode runMode) {
        frontMotor.setMode(runMode);
        leftBackMotor.setMode(runMode);
        rightBackMotor.setMode(runMode);
    }

    private double clip(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}
