package org.firstinspires.ftc.teamcode.robot.opmode.teleop;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Simple robot-centric mecanum drive for testing wheels and motors.
 * Motor names and directions come from Constants.drivetrainConfig.
 *
 * Driving:
 *   left stick      forward / strafe
 *   right stick X   turn
 *   right bumper    slow mode (40%)
 *   right trigger   heading hold: Pinpoint keeps the robot pointing the same way
 *                   while you drive; using the turn stick re-aims it
 *
 * Single-wheel test (hold, overrides driving):
 *   X  front left     Y  front right
 *   A  back left      B  back right
 *   left bumper       run the tested wheel in reverse
 *
 * Pinpoint (settings from Constants.localizerConfig):
 *   back            reset pose to (0, 0, 0)
 *   left trigger    save the current pose
 *   left bumper     drive back to the saved pose with Pedro Pathing
 *                   (move a stick or press X/Y/A/B to cancel)
 * If no Pinpoint is configured, driving still works without it.
 */
@TeleOp(name = "Drive Test", group = "Test")
@Disabled
public class DriveTest extends LinearOpMode {

    private static final double SLOW_SCALE = 0.4;
    private static final double TEST_POWER = 0.5;

    // Heading hold: turn power per radian of heading error, and the most it may add
    private static final double HEADING_KP = 1.5;
    private static final double HEADING_MAX_TURN = 0.5;
    private static final double TURN_STICK_DEADBAND = 0.05;

    // Any stick past this cancels a return-to-saved-pose
    private static final double CANCEL_STICK = 0.2;
    // Closer than this to the saved pose and there is nothing to drive
    private static final double MIN_RETURN_DISTANCE = 1.0;

    @Override
    public void runOpMode() {
        MecanumConfig c = Constants.drivetrainConfig;
        DcMotorEx fL = initMotor(c.frontLeftName.get(), c.frontLeftDirection.get());
        DcMotorEx fR = initMotor(c.frontRightName.get(), c.frontRightDirection.get());
        DcMotorEx bL = initMotor(c.backLeftName.get(), c.backLeftDirection.get());
        DcMotorEx bR = initMotor(c.backRightName.get(), c.backRightDirection.get());

        Follower follower = null;
        Localizer localizer = null;
        GoBildaPinpointDriver pinpoint = null;
        String pinpointError = null;
        try {
            follower = Constants.create(hardwareMap);
            // Go idle at the end of a path instead of holding, so the sticks take over again
            follower.holdEnd.set(false);
            localizer = follower.localizer;
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        } catch (Exception e) {
            follower = null;
            localizer = null;
            pinpointError = e.getMessage();
        }

        telemetry.addLine("Ready. Sticks drive, X/Y/A/B spin one wheel.");
        if (localizer == null) telemetry.addData("Pinpoint", "NOT FOUND: %s", pinpointError);
        telemetry.update();
        waitForStart();

        Double headingTarget = null;
        Pose savedPose = null;
        boolean returning = false;
        String returnMessage = "";
        boolean lastLeftTrigger = false;
        boolean lastLeftBumper = false;

        while (opModeIsActive()) {
            double flPower, frPower, blPower, brPower;

            boolean wheelTest = gamepad1.x || gamepad1.y || gamepad1.a || gamepad1.b;
            boolean leftTrigger = gamepad1.left_trigger > 0.5;
            boolean leftTriggerPressed = leftTrigger && !lastLeftTrigger;
            boolean leftBumperPressed = gamepad1.left_bumper && !lastLeftBumper;
            lastLeftTrigger = leftTrigger;
            lastLeftBumper = gamepad1.left_bumper;

            if (follower != null) {
                if (gamepad1.back) {
                    if (returning) follower.stop();
                    returning = false;
                    localizer.setPose(Pose.zero());
                    headingTarget = null;
                }

                if (leftTriggerPressed) {
                    savedPose = localizer.pose();
                    returnMessage = "Pose saved";
                }

                if (leftBumperPressed && !wheelTest && !returning) {
                    Pose current = localizer.pose();
                    if (savedPose == null) {
                        returnMessage = "No pose saved (press left trigger first)";
                    } else if (current.distance(savedPose) < MIN_RETURN_DISTANCE) {
                        returnMessage = "Already at saved pose";
                    } else {
                        follower.follow(line(current, savedPose).linear(current, savedPose));
                        returning = true;
                        returnMessage = "Returning...";
                    }
                }

                if (returning) {
                    boolean sticksMoved = Math.abs(gamepad1.left_stick_x) > CANCEL_STICK
                            || Math.abs(gamepad1.left_stick_y) > CANCEL_STICK
                            || Math.abs(gamepad1.right_stick_x) > CANCEL_STICK;
                    if (sticksMoved || wheelTest) {
                        follower.stop();
                        returning = false;
                        returnMessage = "Return cancelled";
                    }
                }

                if (returning) {
                    // The follower updates the localizer and drives the wheels itself
                    follower.update();
                    if (!follower.following()) {
                        returning = false;
                        returnMessage = "Arrived";
                        // The follower may leave the motors floating; restore our settings
                        for (DcMotorEx m : new DcMotorEx[]{fL, fR, bL, bR}) {
                            m.setPower(0);
                            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        }
                    }
                } else {
                    // Only update the localizer: follower.update() would stop the motors while idle
                    localizer.update();
                }
            }

            if (!returning) {
                double testPower = gamepad1.left_bumper ? -TEST_POWER : TEST_POWER;
                boolean headingHold = !wheelTest && localizer != null && gamepad1.right_trigger > 0.5;

                if (wheelTest) {
                    flPower = gamepad1.x ? testPower : 0;
                    frPower = gamepad1.y ? testPower : 0;
                    blPower = gamepad1.a ? testPower : 0;
                    brPower = gamepad1.b ? testPower : 0;
                } else {
                    double forward = -gamepad1.left_stick_y;
                    double strafe = gamepad1.left_stick_x;
                    double turn = gamepad1.right_stick_x;

                    if (!headingHold || Math.abs(turn) > TURN_STICK_DEADBAND) {
                        // Driver is turning (or hold is off): lock onto the new heading when they let go
                        headingTarget = null;
                    } else {
                        double heading = localizer.pose().heading();
                        if (headingTarget == null) headingTarget = heading;
                        double error = angleWrap(headingTarget - heading);
                        // Heading increases turning left, but positive stick turns right
                        turn = Range.clip(-HEADING_KP * error, -HEADING_MAX_TURN, HEADING_MAX_TURN);
                    }

                    flPower = forward + strafe + turn;
                    frPower = forward - strafe - turn;
                    blPower = forward - strafe + turn;
                    brPower = forward + strafe - turn;

                    // Keep ratios between wheels when any power exceeds 1
                    double max = Math.max(1.0, Math.max(
                            Math.max(Math.abs(flPower), Math.abs(frPower)),
                            Math.max(Math.abs(blPower), Math.abs(brPower))));
                    double scale = (gamepad1.right_bumper ? SLOW_SCALE : 1.0) / max;
                    flPower *= scale;
                    frPower *= scale;
                    blPower *= scale;
                    brPower *= scale;
                }

                fL.setPower(flPower);
                fR.setPower(frPower);
                bL.setPower(blPower);
                bR.setPower(brPower);

                telemetry.addData("Mode", wheelTest ? "WHEEL TEST" : (gamepad1.right_bumper ? "Drive (slow)" : "Drive"));
                if (headingHold) {
                    telemetry.addData("Heading hold", headingTarget == null
                            ? "turning (will lock on release)"
                            : String.format("locked at %.1f deg", Math.toDegrees(headingTarget)));
                } else if (localizer == null && gamepad1.right_trigger > 0.5) {
                    telemetry.addData("Heading hold", "unavailable: no Pinpoint");
                }
            } else {
                headingTarget = null;
                telemetry.addData("Mode", "RETURNING (move a stick to cancel)");
                telemetry.addData("Distance left (in)", "%.1f", follower.distanceToEndpoint());
            }

            if (follower == null && (leftTriggerPressed || leftBumperPressed)) {
                returnMessage = "Unavailable: no Pinpoint";
            }
            telemetry.addData("Saved pose", savedPose == null ? "none (left trigger to save)"
                    : String.format("%.1f, %.1f, %.1f deg",
                            savedPose.x(), savedPose.y(), Math.toDegrees(savedPose.heading())));
            if (!returnMessage.isEmpty()) telemetry.addData("Return", returnMessage);

            addMotorTelemetry("Front Left", fL);
            addMotorTelemetry("Front Right", fR);
            addMotorTelemetry("Back Left", bL);
            addMotorTelemetry("Back Right", bR);
            addPinpointTelemetry(localizer, pinpoint, pinpointError);
            telemetry.update();
        }
    }

    /** Wraps an angle in radians to [-pi, pi] so the robot turns the short way round. */
    private static double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    private DcMotorEx initMotor(String name, DcMotor.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    private void addPinpointTelemetry(Localizer localizer, GoBildaPinpointDriver pinpoint, String error) {
        telemetry.addLine();
        if (localizer == null) {
            telemetry.addData("Pinpoint", "NOT FOUND: %s", error);
            return;
        }
        Pose pose = localizer.pose();
        Velocity vel = localizer.velocity();
        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("X / Y (in)", "%.2f / %.2f", pose.x(), pose.y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        telemetry.addData("Vel X / Y (in/s)", "%.1f / %.1f", vel.vx, vel.vy);
        telemetry.addData("Turn rate (deg/s)", "%.1f", Math.toDegrees(vel.omega));
        telemetry.addData("Raw encoders X / Y", "%d / %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());
        telemetry.addLine("Press BACK to reset pose");
    }

    private void addMotorTelemetry(String label, DcMotorEx motor) {
        telemetry.addData(label, "power %.2f  pos %d  vel %.0f  %.2fA",
                motor.getPower(),
                motor.getCurrentPosition(),
                motor.getVelocity(),
                motor.getCurrent(CurrentUnit.AMPS));
    }
}
