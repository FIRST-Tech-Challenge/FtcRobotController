package org.firstinspires.ftc.teamcode;

import com.bylazar.field.FieldManager;
import com.bylazar.field.CanvasRotation;
import com.bylazar.field.FieldPresetParams;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Checks that Pedro Pathing is wired up correctly on Lox before any path following is attempted.
 * This deliberately does not follow a path - it only drives the robot and reports the pose the
 * Pinpoint reports, so it does not depend on any of the untuned Foresight constants.
 *
 * AutoTune has a Tests -> Localization procedure (http://192.168.43.1:10158) that also drives the
 * robot and reports the pose, but it prints the pose as text to the Driver Station and nothing
 * more. The field view below - the robot drawn where it thinks it is - only exists here.
 *
 * Telemetry goes to the Driver Station and to Panels, and the robot is drawn on the Panels field
 * view. Connect to the Robot Controller's WiFi and open http://192.168.43.1:8001 to see it.
 *
 * The pose starts at (0, 0) heading 0 and is measured in inches / radians, with x forward,
 * y to the LEFT, and heading counterclockwise-positive.
 *
 * What to check, in order:
 *   1. Drive forward  -> x increases, y stays near 0.
 *   2. Strafe left    -> y increases, x stays near 0.
 *   3. Turn counterclockwise -> heading increases.
 *   4. Push the robot a measured distance by hand (say 24 in) -> the reported number matches.
 *
 * If a value moves the wrong way, fix it in pedroPathing/Constants: a backwards drive axis is a
 * motor direction, a backwards odometry axis is an xPodDirection / yPodDirection.
 *
 * PATH FOLLOWING
 *
 * Hold A to follow a straight line PATH_DISTANCE inches ahead, or B for a curve the same distance
 * ahead and to the left. The path is built from wherever the robot is standing when the button
 * goes down, so it can be run from anywhere on the field, and it is drawn on the field view in red
 * next to the robot in blue - the gap between the two is the following error.
 *
 * UNLIKE the checks above, this DOES depend on Constants.foresightConfig, which is still Pedro's
 * placeholder numbers from another robot. Expect it to track badly until the Foresight tuner has
 * been run. Three things keep that contained, and all of them are worth keeping until it is tuned:
 * the buttons are dead-man switches (release and the robot stops), holdEnd is off so the follower
 * idles at the end instead of fighting to hold the last pose, and both paths use constant heading
 * so nothing commands a rotation. Swap constant(start) for tangent() once the tuning is real.
 */
@TeleOp(name = "Pedro Test", group = "Pedro")
public class PedroLocalizationTest extends OpMode {
    /** Half the robot's width, in inches - only used to size the marker on the field view. */
    private static final double ROBOT_RADIUS = 9;

    /**
     * Stick inputs are scaled by this so the robot is slow enough to watch. Pedro 3 has no
     * follower-level max power, so the scaling is applied to the powers on the way in.
     */
    private static final double DRIVE_SCALE = 0.5;

    /** How far the test paths travel, in inches. */
    private static final double PATH_DISTANCE = 24.0;

    /** Straight-line segments used to draw a path on the field view. */
    private static final int PATH_SAMPLES = 24;

    /*
     * Canvas transform for the Panels field view. Affects ONLY what Panels draws - the pose
     * numbers are untouched, so never "fix" a drawing problem in Constants.java instead.
     *
     * These are the stock PEDRO_PATHING values, restated here so they are visible rather than
     * hidden in the preset. They are correct: working the renderer's matrix through
     * (getItemTransform in Field/web/src/widgets/renderer.ts, with DEG_90 = +pi/2) gives
     * X' = x and Y' = -y, so Pedro +x draws right and Pedro +y draws up, origin bottom-left.
     *
     * The flags are applied in the BROWSER, not on the robot - setOffsets only attaches the
     * preset to the canvas that gets sent over the wire. So if the field view disagrees with
     * these values, suspect the Panels UI's own field settings before changing anything here.
     */
    private static final double OFFSET_X = 24.0 * -3;
    private static final double OFFSET_Y = 24.0 * -3;
    private static final CanvasRotation ROTATION = CanvasRotation.DEG_90;
    private static final boolean FLIP_X = false;
    private static final boolean FLIP_Y = true;
    private static final boolean REVERSE_XY = true;
    private static final int velocity = 1200;

    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private FieldManager field;

    /** The path being followed, kept after it finishes so it stays on the field view. */
    private Path testPath;

    /** True while a path button is held, so the path is built once on the way down. */
    private boolean pathArmed;

    private DcMotorEx flywheel;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(135, 9, Math.toRadians(90)));

        // Stop at the end of a path rather than actively holding the last pose - holding uses the
        // untuned Foresight constants to fight every nudge, which is not what a test should do.
        follower.holdEnd.set(false);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        field = PanelsField.INSTANCE.getField();

        // Draw using Pedro's coordinate frame rather than Panels' own, so what the field view
        // shows lines up with the numbers Pedro reports. Set explicitly rather than trusting the
        // shipped preset, whose values are wrong in this version of Panels.
        FieldPresetParams preset = PanelsField.INSTANCE.getPresets().getPEDRO_PATHING();
//        preset.setReverseXY(REVERSE_XY);
        field.setOffsets(preset);

        // Printed so the canvas transform actually in effect is visible, rather than guessed at.
        telemetry.addData("field offset", preset.getOffsetX() + ", " + preset.getOffsetY());
        telemetry.addData("field rotation", preset.getRotation());
        telemetry.addData("field flipX/flipY/reverseXY",
                preset.getFlipX() + " / " + preset.getFlipY() + " / " + preset.getReverseXY());
        telemetry.addLine("Drive the robot, then compare the pose below to what it actually did.");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            flywheel.setVelocity(velocity);
        } else {
            flywheel.setVelocity(0);
        }

        if (gamepad1.b) {
            // Rising edge only: the path is anchored to where the robot stood when the button
            // went down, so holding the button does not keep rebuilding it underneath itself.
            if (!pathArmed) {
                pathArmed = true;
                testPath = gamepad1.a ? line(follower.pose()) : curve(follower.pose());
                follower.follow(testPath);
            }
        } else {
            if (pathArmed) {
                pathArmed = false;
                follower.stop();
            }
            // Pedro takes strafe as positive-to-the-left and turn as positive-counterclockwise,
            // so both stick x axes are negated.
            follower.manual(
                    -gamepad1.left_stick_y * DRIVE_SCALE,
                    -gamepad1.left_stick_x * DRIVE_SCALE,
                    -gamepad1.right_stick_x * DRIVE_SCALE);
        }

        follower.update();

        Pose pose = follower.pose();
        panelsTelemetry.addData("x (in)", pose.x());
        panelsTelemetry.addData("y (in)", pose.y());
        panelsTelemetry.addData("heading (deg)", Math.toDegrees(pose.heading()));
        panelsTelemetry.addData("localization NaN",
                Double.isNaN(pose.x()) || Double.isNaN(pose.y()) || Double.isNaN(pose.heading()));
        panelsTelemetry.addData("mode", follower.mode());
        if (testPath != null) {
            panelsTelemetry.addData("path completion", follower.completion());
            panelsTelemetry.addData("distance to end (in)", follower.distanceToEndpoint());
            panelsTelemetry.addData("error at end (in)",
                    follower.following() ? "-" : testPath.endPose().distance(pose));
        }
        // Mirrors the same lines to the Driver Station, so the OpMode is still usable with
        // nothing connected to Panels.
        panelsTelemetry.update(telemetry);

        if (testPath != null) {
            drawPath(testPath);
        }
        drawRobot(pose);
    }

    /** A straight line PATH_DISTANCE inches ahead of where the robot is now. */
    private Path line(Pose start) {
        return Paths.line(start, ahead(start, PATH_DISTANCE, 0)).constant(start);
    }

    /**
     * A quarter-circle-ish Bezier to a point PATH_DISTANCE ahead and PATH_DISTANCE to the left,
     * bending through a control point straight ahead of the robot.
     */
    private Path curve(Pose start) {
        return Paths.curve(
                start,
                ahead(start, PATH_DISTANCE, 0),
                ahead(start, PATH_DISTANCE, PATH_DISTANCE)).constant(start);
    }

    /**
     * The pose `forward` inches ahead of `start` and `left` inches to its left, keeping start's
     * heading. Pedro measures heading counterclockwise from +x, so ahead is (cos, sin) and left
     * is that turned a quarter turn counterclockwise.
     */
    private Pose ahead(Pose start, double forward, double left) {
        double sin = Math.sin(start.heading());
        double cos = Math.cos(start.heading());
        return new Pose(
                start.x() + forward * cos - left * sin,
                start.y() + forward * sin + left * cos,
                start.heading());
    }

    /** Draws the commanded path as a red polyline, so it can be compared to where the robot went. */
    private void drawPath(Path path) {
        field.setStyle(new Style(
                PanelsField.INSTANCE.getTRANSPARENT(),
                PanelsField.INSTANCE.getRED(),
                1.0));
        Pose previous = path.get(0.0);
        for (int i = 1; i <= PATH_SAMPLES; i++) {
            Pose point = path.get((double) i / PATH_SAMPLES);
            field.moveCursor(previous.x(), previous.y());
            field.line(point.x(), point.y());
            previous = point;
        }
    }

    /** Draws the robot as a circle with a line pointing the way it is facing. */
    private void drawRobot(Pose pose) {
        field.setStyle(new Style(
                PanelsField.INSTANCE.getTRANSPARENT(),
                PanelsField.INSTANCE.getBLUE(),
                1.0));
        field.moveCursor(pose.x(), pose.y());
        field.circle(ROBOT_RADIUS);
        field.moveCursor(pose.x(), pose.y());
        field.line(
                pose.x() + ROBOT_RADIUS * Math.sin(pose.heading()),
                pose.y() + ROBOT_RADIUS * Math.cos(pose.heading()));
        field.update();
    }
}