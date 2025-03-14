package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveControl {
    private DcMotor RF, RB, LF, LB;

    public DriveControl(HardwareMap hardwareMap) {
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        LB = hardwareMap.get(DcMotor.class, "LB");
    }

    // ✅ Set individual motor powers
    public void setDriveMotorPowers(double rightFrontPower, double leftFrontPower, double rightBackPower, double leftBackPower) {
        RF.setPower(rightFrontPower);
        RB.setPower(rightBackPower);
        LF.setPower(leftFrontPower);
        LB.setPower(leftBackPower);
    }

    // ✅ NEW: POV Drive Mode (Uses joystick inputs)
    public void driveWithGamepad(Gamepad gamepad1) {
        double axial = -gamepad1.left_stick_y;  // Forward/Backward
        double lateral = gamepad1.left_stick_x;  // Left/Right strafing
        double yaw = gamepad1.right_stick_x;  // Rotation

        // Calculate motor powers
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize motor values to prevent exceeding range (-1.0 to 1.0)
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
        ));

        leftFrontPower /= max;
        rightFrontPower /= max;
        leftBackPower /= max;
        rightBackPower /= max;

        // Set the normalized motor powers
        setDriveMotorPowers(rightFrontPower, leftFrontPower, rightBackPower, leftBackPower);
    }
}