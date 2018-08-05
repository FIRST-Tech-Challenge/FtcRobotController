package com.acmerobotics.splinelib.drive;

import com.acmerobotics.splinelib.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class TrackWidthCalibrationOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = initDrive();
        BNO055IMU imu = initIMU();

        telemetry.log().add("Press play to begin the track width calibration routine");
        telemetry.log().add("Make sure your robot has enough clearance to turn smoothly");
        telemetry.log().add("Additionally, set the drive's track width to 1");
        telemetry.update();

        waitForStart();

        telemetry.log().clear();
        telemetry.log().add("Running...");
        telemetry.update();

        drive.setVelocity(new Pose2d(0.0, 0.0, 0.2));
        drive.resetPoseEstimate(new Pose2d(0.0, 0.0, 0.0));
        while (opModeIsActive()) {
            double angle = imu.getAngularOrientation().firstAngle;
            if (imu.getParameters().angleUnit == BNO055IMU.AngleUnit.DEGREES) {
                angle = Math.toRadians(angle);
            }
            if (angle >= Math.PI) {
                drive.setVelocity(new Pose2d(0.0, 0.0, 0.0));
                break;
            }
            drive.updatePoseEstimate(System.nanoTime() / 1e9);
        }
        double effectiveTrackWidth = drive.getPoseEstimate().heading() / Math.PI;

        telemetry.log().clear();
        telemetry.log().add("Calibration complete");
        telemetry.log().add(String.format("effective track width = %.2f", effectiveTrackWidth));
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }

    protected abstract Drive initDrive();
    protected abstract BNO055IMU initIMU();
}
