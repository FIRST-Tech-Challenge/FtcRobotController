package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.drive.Kinematics;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.Angle;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveSimple;

@Config
@Autonomous
public class NewTrackWidthCalibrationOpMode extends LinearOpMode {
    public static int CIRCUMFERENTIAL_DISTANCE = 500;
    public static int NUM_TRIALS = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveSimple drive = new SampleMecanumDriveSimple(hardwareMap);
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
                    0.5 * DriveConstants.BASE_CONSTRAINTS.maximumVelocity,
                    0.5 * DriveConstants.BASE_CONSTRAINTS.maximumAcceleration
            );

            double startTime = clock.seconds();
            while (!this.isStopRequested() && this.isStarted()) {
                double elapsedTime = clock.seconds() - startTime;
                if (time > profile.duration()) {
                    drive.setVelocity(new Pose2d(0, 0, 0));
                    break;
                }

                double heading = drive.getExternalHeading();
                // accumulator is an unwrapped version of the heading
                headingAccumulator += Angle.norm(heading - lastHeading);

                MotionState state = profile.get(elapsedTime);
                drive.setVelocity(new Pose2d(0, 0,
                        Kinematics.calculateMotorFeedforward(
                                2 * state.getV(),
                                0.0,
                                DriveConstants.kV,
                                DriveConstants.kA,
                                DriveConstants.kStatic
                        )
                ));

                drive.updatePoseEstimate();
            }

            double trackWidth = drive.getPoseEstimate().getHeading() / headingAccumulator;
            trackWidthStats.add(trackWidth);

            sleep(1000);
        }

        telemetry.log().clear();
        telemetry.log().add("Calibration complete");
        telemetry.log().add(String.format("Effective track width = %.2f (SE = %.3f)",
                trackWidthStats.getMean(), trackWidthStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS)));
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
