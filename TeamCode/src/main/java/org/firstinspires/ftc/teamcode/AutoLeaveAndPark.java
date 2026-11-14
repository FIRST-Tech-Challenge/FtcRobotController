package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Leave and Park - Mark 1
 * The simplest scoring Auto (rung A-1 in TeamCode/docs/biobuzz-auto-strategy.md).
 * The robot drives off the wall and ends partly inside our LOADING ZONE:
 *     LEAVE = 3 points, PARK = 5 points, so 8 points. No shooter, no camera, no turret.
 *
 *   1. DRIVE_TO_WAYPOINT : slide to the right of the hive (so we do not clip it).
 *   2. DRIVE_TO_PARK     : drive down the field into the LOADING ZONE.
 *   3. DONE              : the robot holds its spot.
 *
 * This OpMode owns NO motors itself. Pedro's Follower drives the four drive motors and reads the
 * Pinpoint. So do NOT use DriveSubsystem or PinpointSubsystem in this OpMode.
 *
 * WHERE THE ROBOT STARTS (BLUE, the default)   (all the numbers are in pedro/FieldConstants.java)
 *   Tile D6, in the middle of the tile, with the robot's BACK against the TOP wall, facing DOWN
 *   the field toward the hive. Measure from the LEFT wall to the middle of the robot: about 82.5 inches.
 *   The robot must not touch a flower, and no part of it may be inside the LOADING ZONE.
 *   RED (press B in INIT) is the same plan turned 180 degrees: tile C1, back against the BOTTOM
 *   wall, facing UP. Red has not been driven yet, so check it slowly the first time.
 *
 * BEFORE YOU RUN IT (once per robot)
 *   - Do the Pedro tuning in TeamCode/docs/pedro-setup-and-auto-guide.md, paste the numbers into
 *     pedro/Constants.java, and set FORESIGHT_TUNED to true. Until then this OpMode REFUSES to
 *     drive and says so on the screen.
 *   - Make sure PedroForwardBackAlpha has passed on this robot first.
 *   - Clear the path: from the start, over to the right of the hive, and down to the LOADING ZONE.
 *     (Use the real field if you can. Otherwise tape the hive's footprint on the floor.)
 *
 * HOW TO RUN IT
 *   1. Set the robot on its start spot and leave it STILL. (INIT recalibrates the Pinpoint's gyro,
 *      so a robot that moves during INIT will be off.)
 *   2. Select "Leave and Park - Mark 1" and press INIT. Wait for "Pinpoint ready".
 *   3. Check the ALLIANCE line. gamepad1 X = BLUE, gamepad1 B = RED. BLUE is the default.
 *   4. Press START. (The 30 second timer should be ON, like in a match.)
 *
 * WHAT TO WATCH FOR
 *   - The robot should end with part of it inside the LOADING ZONE and NOT touching the wall.
 *     Put tape on the floor at the zone line and at the wall, and note how far off it is.
 *   - "Pose" on the screen is where Pedro THINKS the robot is. Compare it with where the robot
 *     really is. A big gap means the start spot was wrong or the Pinpoint settings are wrong.
 *   - Run it 5 times in a row before you trust it (rung rule). Write each run in
 *     TeamCode/docs/biobuzz-test-and-match-log.md.
 *   - It is in group "Auto" (below), with the other match Autos.
 */
@Autonomous(name = "Leave and Park - Mark 1", group = "Auto",
        preselectTeleOp = "TeleOp - Alpha 14")
public class AutoLeaveAndPark extends OpMode {

    // ------------------------------------------------------------------
    // Settings
    // ------------------------------------------------------------------
    // The most time one drive step is allowed to take, in seconds. Normally a step ends when
    // Pedro says the robot has arrived and settled. If it never settles (stuck, or a very tight
    // finish setting), this moves us on so the robot still gets to the next step in 30 seconds.
    private static final double LEG_TIMEOUT_SECONDS = 6.0;

    // ------------------------------------------------------------------
    // The steps of this Auto. The robot is always in exactly one of these.
    // ------------------------------------------------------------------
    private enum AutoState {
        DRIVE_TO_WAYPOINT,
        DRIVE_TO_PARK,
        DONE
    }

    // ------------------------------------------------------------------
    // Things this OpMode uses
    // ------------------------------------------------------------------
    private Follower follower;

    // Which hive is ours. (We only use the BLUE / RED choice here. This Auto has no camera.)
    private VisionSubsystem.Alliance alliance = VisionSubsystem.Alliance.BLUE;

    private Pose startPose;
    private Pose waypointPose;
    private Pose parkPose;

    // The two paths are built when the alliance is chosen (INIT), never inside loop().
    private Path waypointPath;
    private Path parkPath;

    private AutoState autoState = AutoState.DONE;

    // Times one drive step, so a step that never settles cannot use up the whole Auto.
    private final ElapsedTime legTimer = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Create the Follower. This recalibrates the Pinpoint, so keep the robot still.
        follower = Constants.create(hardwareMap);

        // BLUE is the default. This builds the poses and paths and tells Pedro where we start.
        applyAlliance(VisionSubsystem.Alliance.BLUE);

        telemetry.addLine("Pinpoint ready. Keep the robot still, then press START.");
        addStatusTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // The Follower is idle here, so this only reads the Pinpoint and keeps the wheels stopped.
        follower.update();

        // gamepad1 X picks BLUE, gamepad1 B picks RED. Nothing happens if it is already chosen.
        if (gamepad1.x && alliance != VisionSubsystem.Alliance.BLUE) {
            applyAlliance(VisionSubsystem.Alliance.BLUE);
        } else if (gamepad1.b && alliance != VisionSubsystem.Alliance.RED) {
            applyAlliance(VisionSubsystem.Alliance.RED);
        }

        addStatusTelemetry();
        addPoseTelemetry();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        if (!Constants.FORESIGHT_TUNED) {
            // Do not drive on another robot's braking numbers. Stay in DONE and show why.
            autoState = AutoState.DONE;
            return;
        }

        // The rules say the robot has not moved since INIT, so it is on its start spot. Tell
        // Pedro that again, in case the robot was nudged while the drive team was setting up.
        follower.setPose(startPose);
        follower.update();

        autoState = AutoState.DRIVE_TO_WAYPOINT;
        legTimer.reset();
        follower.follow(waypointPath);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. Update Pedro. This reads the Pinpoint and sets the wheel powers. It must run
        //    every loop, or the robot does not move (or stops steering).
        follower.update();

        // 2. Decide what to do next.
        switch (autoState) {
            case DRIVE_TO_WAYPOINT:
                // isBusy() stays true until the robot has arrived and settled.
                if (!follower.isBusy() || legTimer.seconds() >= LEG_TIMEOUT_SECONDS) {
                    legTimer.reset();
                    follower.follow(parkPath);
                    autoState = AutoState.DRIVE_TO_PARK;
                }
                break;

            case DRIVE_TO_PARK:
                if (!follower.isBusy() || legTimer.seconds() >= LEG_TIMEOUT_SECONDS) {
                    autoState = AutoState.DONE;
                }
                break;

            case DONE:
                // Nothing left to do. The Follower keeps holding the last pose.
                break;
        }

        // 3. Telemetry
        addStatusTelemetry();
        telemetry.addData("State", autoState);
        addPoseTelemetry();
        telemetry.addData("Distance to end of path (in)", "%.2f", follower.distanceToEndpoint());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Put the Follower in IDLE and tell the drivetrain to stop right now. (follower.stop()
        // only changes the mode. The wheels are zeroed by the next update(), and there is none.)
        follower.stop();
        follower.drivetrain.stop();
    }

    // ------------------------------------------------------------------
    // Alliance
    // ------------------------------------------------------------------
    /*
     * Picks the alliance: builds the three poses and the two paths for it, and tells Pedro where
     * the robot is standing. Only call it in INIT, never in loop().
     */
    private void applyAlliance(VisionSubsystem.Alliance newAlliance) {
        alliance = newAlliance;

        startPose = FieldConstants.startPose(alliance);
        waypointPose = FieldConstants.waypointPose(alliance);
        parkPose = FieldConstants.parkPose(alliance);

        // A path is a line between poses. ".constant(startPose)" means: keep facing the heading
        // of startPose the whole way, so the robot never turns. (It slides sideways instead.)
        waypointPath = Paths.line(startPose, waypointPose).constant(startPose);
        parkPath = Paths.line(waypointPose, parkPose).constant(startPose);

        // Tell Pedro where the robot is standing right now.
        follower.setPose(startPose);
    }

    // ------------------------------------------------------------------
    // Telemetry helpers
    // ------------------------------------------------------------------
    private void addStatusTelemetry() {
        telemetry.addData("ALLIANCE (gamepad1 X = BLUE, B = RED)", alliance);
        if (alliance == VisionSubsystem.Alliance.BLUE) {
            telemetry.addLine("Start: tile D6, back to the TOP wall, facing DOWN");
        } else {
            telemetry.addLine("Start: tile C1, back to the BOTTOM wall, facing UP");
        }

        if (Constants.FORESIGHT_TUNED) {
            telemetry.addData("Foresight", "tuned, OK to drive");
        } else {
            telemetry.addLine("!!! NOT TUNED: this OpMode will NOT drive. !!!");
            telemetry.addLine("Tune Foresight (see the Pedro guide), paste the numbers into");
            telemetry.addLine("pedro/Constants.java, then set FORESIGHT_TUNED = true.");
        }
    }

    private void addPoseTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("Pose X (in)", "%.2f", pose.x());
        telemetry.addData("Pose Y (in)", "%.2f", pose.y());
        telemetry.addData("Pose heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
    }
}
