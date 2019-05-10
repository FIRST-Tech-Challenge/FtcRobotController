package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.Angle;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveREV;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.BASE_CONSTRAINTS;
/*
 * Similar to the deprecated OldTrackWidthTuner, this routine attempts to automagically
 * determine the drive track width. The basic idea is to use a motion profile to rotate the robot
 * a certain circumferential distance and compare it to the angle swept out (as measured by the
 * IMU). For robustness, this procedure is repeated a few times, and the final track width is
 * averaged over those runs.
 */
@Config
@Autonomous(group = "drive")
public class NewTrackWidthTuner extends LinearOpMode {
    public static int CIRCUMFERENTIAL_DISTANCE = 500;
    public static int NUM_TRIALS = 5;

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

        MovingStatistics trackWidthStats = new MovingStatistics(NUM_TRIALS);
        NanoClock clock = NanoClock.system();
        for (int i = 0; i < NUM_TRIALS; i++) {
            drive.setPoseEstimate(new Pose2d());

            double headingAccumulator = 0;
            double lastHeading = drive.getExternalHeading();
            MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                    new MotionState(0, 0, 0, 0),
                    new MotionState(CIRCUMFERENTIAL_DISTANCE, 0, 0, 0),
                    BASE_CONSTRAINTS.maxVel,
                    BASE_CONSTRAINTS.maxAccel
            );

            double startTime = clock.seconds();
            while (!isStopRequested()) {
                double elapsedTime = clock.seconds() - startTime;
                if (elapsedTime > profile.duration()) {
                    drive.setDriveSignal(new DriveSignal());
                    break;
                }

                double heading = drive.getExternalHeading();
                // accumulator is an unwrapped version of the heading
                headingAccumulator += Angle.norm(heading - lastHeading);
                lastHeading = heading;

                MotionState state = profile.get(elapsedTime);
                drive.setDriveSignal(new DriveSignal(
                        new Pose2d(0, 0, state.getV()),
                        new Pose2d(0, 0, state.getA())
                ));

                drive.updatePoseEstimate();
            }

            double trackWidth = drive.getPoseEstimate().getHeading() / headingAccumulator;
            trackWidthStats.add(trackWidth);

            sleep(1000);
        }

        telemetry.log().clear();
        telemetry.log().add("Calibration complete");
        telemetry.log().add(Misc.formatInvariant("Effective track width = %.2f (SE = %.3f)",
                trackWidthStats.getMean(), trackWidthStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS)));
        telemetry.update();

        while (!isStopRequested()) {
            idle();
        }
    }
}
