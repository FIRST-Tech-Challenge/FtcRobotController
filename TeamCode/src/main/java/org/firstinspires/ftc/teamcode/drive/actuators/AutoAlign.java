package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//@TeleOp(name = "IMU Auto Align", group = "Linear Opmode")
public class AutoAlign extends LinearOpMode {

    private DcMotor lf, lb, rf, rb;
    private IMU imu;

    private double targetHeading = 0; // radians
    private static final double ALIGN_TOL = Math.toRadians(2); // stop threshold (~2°)
    private static final double kP = 1.2;

    @Override
    public void runOpMode() {

        // Motor mapping
        lf = hardwareMap.get(DcMotor.class, "odol");
        lb = hardwareMap.get(DcMotor.class, "odor");
        rf = hardwareMap.get(DcMotor.class, "FR");
        rb = hardwareMap.get(DcMotor.class, "odom");

        // Reverse left side
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        // IMU initialization
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        ));

        waitForStart();

        while (opModeIsActive()) {
            double lx = -gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y;
            double rx;

            // Update target heading on RB press
            if (gamepad1.right_bumper) {
                targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            // Get current heading
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double error = angleWrap(targetHeading - currentHeading);

            if (gamepad1.left_bumper) {
                // Auto-align mode
                if (Math.abs(error) > ALIGN_TOL) {
                    rx = error * kP;
                } else {
                    rx = 0;
                }
            } else {
                rx = -gamepad1.right_stick_x;
            }

            // Field-centric transformation
            double adjustedLx = ly * Math.sin(currentHeading) + lx * Math.cos(currentHeading);
            double adjustedLy = ly * Math.cos(currentHeading) - lx * Math.sin(currentHeading);

            double max = Math.max(Math.abs(adjustedLy) + Math.abs(adjustedLx) + Math.abs(rx), 1.0);

            lf.setPower((adjustedLy + adjustedLx + rx) / max);
            lb.setPower((adjustedLy - adjustedLx + rx) / max);
            rf.setPower((adjustedLy - adjustedLx - rx) / max);
            rb.setPower((adjustedLy + adjustedLx - rx) / max);

            telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
            telemetry.addData("Current Heading (deg)", Math.toDegrees(currentHeading));
            telemetry.addData("Heading Error (deg)", Math.toDegrees(error));
            telemetry.addData("Auto Align Active", gamepad1.left_bumper);
            telemetry.update();
        }
    }

    // Normalizes angle between -PI to PI
    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
