package org.firstinspires.ftc.teamcode.samples;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/** Reference autonomous structure. Disabled until Constants can build a follower. */
@Disabled
@Autonomous
public class PedroAutoSample extends CommandOpMode {

    private Follower follower;
    public TelemetryData telemetryData = new TelemetryData(telemetry);

    // Poses
    private final Pose startPose = new Pose(9, 111, Math.toRadians(-90));
    private final Pose scorePose = new Pose(16, 128, Math.toRadians(-45));
    private final Pose pickup1Pose = new Pose(30, 121, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(30, 131, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(45, 128, Math.toRadians(90));
    private final Pose parkPose = new Pose(68, 96, Math.toRadians(-90));

    // Paths
    private Path scorePreload;
    private Path grabPickup1;
    private Path grabPickup2;
    private Path grabPickup3;
    private Path scorePickup1;
    private Path scorePickup2;
    private Path scorePickup3;
    private Path park;

    public void buildPaths() {
        scorePreload = Paths.line(startPose, scorePose)
                .linear(startPose.heading(), scorePose.heading());

        grabPickup1 = Paths.line(scorePose, pickup1Pose)
                .linear(scorePose.heading(), pickup1Pose.heading());

        scorePickup1 = Paths.line(pickup1Pose, scorePose)
                .linear(pickup1Pose.heading(), scorePose.heading());

        grabPickup2 = Paths.line(scorePose, pickup2Pose)
                .linear(scorePose.heading(), pickup2Pose.heading());

        scorePickup2 = Paths.line(pickup2Pose, scorePose)
                .linear(pickup2Pose.heading(), scorePose.heading());

        grabPickup3 = Paths.line(scorePose, pickup3Pose)
                .linear(scorePose.heading(), pickup3Pose.heading());

        scorePickup3 = Paths.line(pickup3Pose, scorePose)
                .linear(pickup3Pose.heading(), scorePose.heading());

        park = Paths.curve(
                scorePose,
                new Pose(68, 110), // Control point
                parkPose
        ).linear(scorePose.heading(), parkPose.heading());
    }

    // Mechanism commands - replace these with your actual subsystem commands
    private InstantCommand openOuttakeClaw() {
        return new InstantCommand(() -> {
            // Example: outtakeSubsystem.openClaw();
        });
    }

    private InstantCommand grabSample() {
        return new InstantCommand(() -> {
            // Example: intakeSubsystem.grabSample();
        });
    }

    private InstantCommand scoreSample() {
        return new InstantCommand(() -> {
            // Example: outtakeSubsystem.scoreSample();
        });
    }

    private InstantCommand level1Ascent() {
        return new InstantCommand(() -> {
            // Example: hangSubsystem.level1Ascent();
        });
    }

    @Override
    public void initialize() {
        super.reset();

        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        buildPaths();

        // Schedule the autonomous sequence
        schedule(
                // Score preload
                new FollowPathCommand(follower, scorePreload),
                openOuttakeClaw(),
                new WaitCommand(1000), // Wait 1 second

                // First pickup cycle
                // Sets globalMaxPower to 50% for all future paths
                // (unless a custom maxPower is given)
                new FollowPathCommand(follower, grabPickup1).setGlobalMaxPower(0.5),
                grabSample(),
                new FollowPathCommand(follower, scorePickup1),
                scoreSample(),

                // Second pickup cycle
                new FollowPathCommand(follower, grabPickup2),
                grabSample(),
                new FollowPathCommand(follower, scorePickup2, 1.0), // Overrides maxPower to 100% for this path only
                scoreSample(),

                // Third pickup cycle
                new FollowPathCommand(follower, grabPickup3),
                grabSample(),
                new FollowPathCommand(follower, scorePickup3),
                scoreSample(),

                // Park
                new FollowPathCommand(follower, park, false), // park with holdEnd false
                level1Ascent()
        );
    }

    @Override
    public void run() {
        super.run();
        follower.update();

        telemetryData.addData("X", follower.pose().x());
        telemetryData.addData("Y", follower.pose().y());
        telemetryData.addData("Heading", follower.pose().heading());
        telemetryData.update();
    }
}
