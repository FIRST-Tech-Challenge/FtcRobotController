package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.commands.drive.roadrunner.RunCommand;
import org.firstinspires.ftc.teamcode.commands.drive.roadrunner.TurnCommand;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drive.roadrunner.MecanumDriveSubsystem;

/**
 * This routine determines the effective track width. The procedure works by executing a point turn
 * with a given angle and measuring the difference between that angle and the actual angle (as
 * indicated by an external IMU/gyro, track wheels, or some other localizer). The quotient
 * given angle / actual angle gives a multiplicative adjustment to the estimated track width
 * (effective track width = estimated track width * given angle / actual angle). The routine repeats
 * this procedure a few times and averages the values for additional accuracy. Note: a relatively
 * accurate track width estimate is important or else the angular constraints will be thrown off.
 *
 * NOTE: this has been refactored to use FTCLib's command-based
 */
@Config
@Autonomous(group = "drive")
public class TrackWidthTuner extends CommandOpMode {

    public static double ANGLE = 180; // deg
    public static int NUM_TRIALS = 5;
    public static int DELAY = 1000; // ms

    private MecanumDriveSubsystem drive;
    private MovingStatistics trackWidthStats;
    private TurnCommand turnCommand;
    private double headingAccumulator, lastHeading;
    private int trial;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        // TODO: if you haven't already, set the localizer to something that doesn't depend on
        // drive encoders for computing the heading

        turnCommand = new TurnCommand(drive, Math.toRadians(ANGLE));

        telemetry.addLine("Press play to begin the track width tuner routine");
        telemetry.addLine("Make sure your robot has enough clearance to turn smoothly");
        telemetry.update();

        SequentialCommandGroup setupCommand = new SequentialCommandGroup(
                new WaitUntilCommand(this::isStarted),
                new InstantCommand(() -> {
                    telemetry.clearAll();
                    telemetry.addLine("Running...");
                    telemetry.update();

                    trackWidthStats = new MovingStatistics(NUM_TRIALS);
                    trial = 0;
                }
        ));

        InstantCommand finishCommand = new InstantCommand(() -> {
            telemetry.clearAll();
            telemetry.addLine("Tuning complete");
            telemetry.addLine(Misc.formatInvariant("Effective track width = %.2f (SE = %.3f)",
                    trackWidthStats.getMean(),
                    trackWidthStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS)));
            telemetry.update();
        });

        RunCommand tuneCommand = new RunCommand(() -> {
            if (trial < NUM_TRIALS && (turnCommand == null || !turnCommand.isScheduled())) {
                if (headingAccumulator != 0) {
                    double trackWidth = DriveConstants.TRACK_WIDTH * Math.toRadians(ANGLE) / headingAccumulator;
                    trackWidthStats.add(trackWidth);
                }

                sleep(DELAY);

                trial++;

                drive.setPoseEstimate(new Pose2d());

                // it is important to handle heading wraparounds
                headingAccumulator = 0;
                lastHeading = 0;

                turnCommand = new TurnCommand(drive, Math.toRadians(ANGLE));
                turnCommand.schedule();
            } else {
                double heading = drive.getPoseEstimate().getHeading();
                headingAccumulator += Angle.norm(heading - lastHeading);
                lastHeading = heading;
            }
        });

        schedule(setupCommand.andThen(
                new WaitUntilCommand(() -> trial == NUM_TRIALS)
                    .deadlineWith(tuneCommand)
                    .whenFinished(turnCommand::cancel),
                finishCommand
        ));
    }

}
