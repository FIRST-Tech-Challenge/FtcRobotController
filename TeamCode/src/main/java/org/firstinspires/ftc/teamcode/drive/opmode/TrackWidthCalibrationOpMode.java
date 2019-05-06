package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveREV;

/*
 * This routine measures the effective track width of the drive (i.e., the distance between a
 * pair of wheels on opposite sides of the robot). This is required for the robot turn properly
 * during open-loop control.
 *
 * Note: this routine is *deprecated*; NewTrackWidthCalibrationOpMode is recommended instead.
 */
@Config
@Deprecated
@Disabled
@Autonomous
public class TrackWidthCalibrationOpMode extends LinearOpMode {
    public static int TOTAL_REVOLUTIONS = 10;
    public static double POWER = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        // it's important that the IMU/gyro/heading sensor is not part of the localization
        drive.setLocalizer(new MecanumDrive.MecanumLocalizer(drive, false));

        telemetry.log().add("Press play to begin the track width calibration routine");
        telemetry.log().add("Make sure your robot has enough clearance to turn smoothly");
        telemetry.log().add("Additionally, set the drive's track width to 1");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.log().clear();
        telemetry.log().add("Running...");
        telemetry.update();

        int revolutions = 0;
        boolean startedMoving = false;
        double lastHeading = 0;

        drive.setPoseEstimate(new Pose2d());
        drive.setDrivePower(new Pose2d(0.0, 0.0,  POWER));
        while (!isStopRequested() && (!startedMoving || revolutions <= TOTAL_REVOLUTIONS)) {
            double heading = drive.getExternalHeading();
            if (heading >= Math.PI / 2.0) {
                startedMoving = true;
            }
            if (startedMoving && (lastHeading < 0.0 && heading >= 0.0)) {
                revolutions++;
            }
            drive.updatePoseEstimate();
            lastHeading = heading;
        }
        drive.setDriveSignal(new DriveSignal());
        double effectiveTrackWidth = drive.getPoseEstimate().getHeading() / (2.0 * Math.PI * TOTAL_REVOLUTIONS);

        telemetry.log().clear();
        telemetry.log().add("Calibration complete");
        telemetry.log().add(Misc.formatInvariant("Effective track width = %.2f", effectiveTrackWidth));
        telemetry.update();

        while (!isStopRequested()) {
            idle();
        }
    }
}
