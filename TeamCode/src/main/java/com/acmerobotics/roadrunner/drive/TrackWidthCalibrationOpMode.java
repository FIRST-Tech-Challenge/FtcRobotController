package com.acmerobotics.roadrunner.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Op mode for measuring the empirical track width of a robot drive.
 */
public abstract class TrackWidthCalibrationOpMode extends LinearOpMode {
    private int totalRevolutions;
    private double power;

    /**
     * @param totalRevolutions number of revolutions
     * @param power angular power
     */
    public TrackWidthCalibrationOpMode(int totalRevolutions, double power) {
        this.totalRevolutions = totalRevolutions;
        this.power = power;
    }

    public TrackWidthCalibrationOpMode() {
        this(4, 0.3);
    }

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

        int revolutions = 0;
        boolean startedMoving = false;
        double lastHeading = 0;

        drive.setPoseEstimate(new Pose2d());
        drive.setVelocity(new Pose2d(0.0, 0.0,  power));
        while (opModeIsActive() && (!startedMoving || revolutions <= totalRevolutions)) {
            double heading = imu.getAngularOrientation().firstAngle;
            if (imu.getParameters().angleUnit == BNO055IMU.AngleUnit.DEGREES) {
                heading = Math.toRadians(heading);
            }
            if (heading >= Math.PI / 2.0) {
                startedMoving = true;
            }
            if (startedMoving && (lastHeading < 0.0 && heading >= 0.0)) {
                revolutions++;
            }
            drive.updatePoseEstimate();
            lastHeading = heading;
        }
        drive.setVelocity(new Pose2d(0.0, 0.0, 0.0));
        double effectiveTrackWidth = drive.getPoseEstimate().getHeading() / (2.0 * Math.PI * totalRevolutions);

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
