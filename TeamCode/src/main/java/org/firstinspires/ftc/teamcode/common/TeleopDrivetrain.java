package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeleopDrivetrain extends Drivetrain {


    public TeleopDrivetrain(HardwareMap hardwareMap, Telemetry telemetry, boolean loggingOn) {
        super(hardwareMap,telemetry, loggingOn);
    }

    public void creepDirection(double axial, double strafe, double yaw) {
        setToCreepPower();
        moveDirection(axial, strafe, yaw);
        setToNormalPower();
    }
    public void moveDirection(double axial, double strafe, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = axial - strafe - yaw;
        double rightFrontPower = axial + strafe + yaw;
        double leftBackPower = axial + strafe - yaw;
        double rightBackPower = axial - strafe + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower * maxPower);
        rightFrontDrive.setPower(rightFrontPower * maxPower);
        leftBackDrive.setPower(leftBackPower * maxPower);
        rightBackDrive.setPower(rightBackPower * maxPower);
    }

    public void stop() {
        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
    }
}