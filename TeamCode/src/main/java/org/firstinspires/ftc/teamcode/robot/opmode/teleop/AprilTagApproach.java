package org.firstinspires.ftc.teamcode.robot.opmode.teleop;

import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Finds AprilTags 42 and 43 with the Limelight 3A and drives the robot until the tags are
 * TARGET_DISTANCE_IN straight out from the camera, squared up to face them. With
 * CAMERA_FACES_BACKWARD the camera is on the back, so the robot backs up to the tags.
 *
 * Uses only the camera: no Pinpoint or wheel odometry. Every loop it re-measures where the
 * tags are and drives to shrink the error, so wheel slip doesn't matter.
 *
 *   Both tags visible   aims at the point halfway between them and turns so both are the
 *                       same distance away (robot square to the tags).
 *   One tag visible     aims at that tag and drives to the distance, but can't tell which
 *                       way to square up, so it just faces the tag.
 *   No tags visible     turns in place to look for them, toward where they were last seen.
 *
 * Controls (gamepad 1):
 *   A               start
 *   B or any stick  cancel (the sticks then drive the robot as normal)
 *   left stick      forward / strafe while idle
 *   right stick X   turn while idle
 *
 * Limelight setup: add an AprilTag (fiducial) pipeline in slot APRILTAG_PIPELINE, with the
 * tag family and tag size set to the game's tags, so the camera can measure distance.
 */
@TeleOp(name = "AprilTag Approach", group = "Test")
@Disabled
public class AprilTagApproach extends LinearOpMode {

    private static final String LIMELIGHT_NAME = "limelight";
    private static final int APRILTAG_PIPELINE = 0;
    private static final int TAG_A = 42;
    private static final int TAG_B = 43;

    // true when the camera is on the back of the robot looking backward, so the robot
    // backs up to the tags; false when it looks out the front
    private static final boolean CAMERA_FACES_BACKWARD = true;

    // Where to stop: tags this far from the robot edge the camera looks out of
    private static final double TARGET_DISTANCE_IN = 55.0;
    // Camera mounting: how far the lens sits inside that edge, and how far right of the
    // robot's centerline (left is negative; "right" as seen from behind the robot driving forward)
    private static final double CAMERA_INSIDE_EDGE_IN = 0.0;
    private static final double CAMERA_RIGHT_OF_CENTER_IN = 0.0;

    // Power per inch / degree of error, and the most each may use
    private static final double FORWARD_KP = 0.03;
    private static final double STRAFE_KP = 0.04;
    private static final double TURN_KP = 0.015;
    private static final double MAX_DRIVE = 0.4;
    private static final double MAX_TURN = 0.3;
    // Smallest power that still moves the robot, so it doesn't stall just short of the target
    private static final double MIN_POWER = 0.06;

    // Close enough to stop
    private static final double DISTANCE_TOLERANCE_IN = 1.5;
    private static final double LATERAL_TOLERANCE_IN = 1.5;
    private static final double HEADING_TOLERANCE_DEG = 2.0;
    // Must stay in tolerance this long to count as arrived
    private static final double SETTLE_S = 0.3;

    private static final double SEARCH_TURN = 0.2;
    private static final double SEARCH_TIMEOUT_S = 10.0;
    // Tags out of view this long while approaching: stop and search again
    private static final double LOST_TIMEOUT_S = 0.5;
    private static final long MAX_STALENESS_MS = 100;
    private static final double CANCEL_STICK = 0.2;

    private enum State { IDLE, SEARCHING, APPROACHING, DONE }

    private DcMotorEx fL, fR, bL, bR;

    @Override
    public void runOpMode() {
        MecanumConfig c = Constants.drivetrainConfig;
        fL = initMotor(c.frontLeftName.get(), c.frontLeftDirection.get());
        fR = initMotor(c.frontRightName.get(), c.frontRightDirection.get());
        bL = initMotor(c.backLeftName.get(), c.backLeftDirection.get());
        bR = initMotor(c.backRightName.get(), c.backRightDirection.get());

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addLine("Ready. Press A to find tags " + TAG_A + " and " + TAG_B + ".");
        telemetry.update();
        waitForStart();

        State state = State.IDLE;
        String message = "Press A to start";
        ElapsedTime stateTimer = new ElapsedTime();
        ElapsedTime lastSeen = new ElapsedTime();
        ElapsedTime inTolerance = new ElapsedTime();
        // Search toward where the tags were last seen: +1 right, -1 left
        double searchDirection = 1;

        while (opModeIsActive()) {
            Position tagA = null;
            Position tagB = null;
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()
                    && result.getStaleness() < MAX_STALENESS_MS
                    && result.getPipelineIndex() == APRILTAG_PIPELINE) {
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    // Camera space: x right, y down, z straight out of the lens
                    if (tag.getTargetPoseCameraSpace() == null) continue;
                    Position p = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
                    if (tag.getFiducialId() == TAG_A) tagA = p;
                    if (tag.getFiducialId() == TAG_B) tagB = p;
                }
            }
            boolean seen = tagA != null || tagB != null;

            boolean sticksMoved = Math.abs(gamepad1.left_stick_x) > CANCEL_STICK
                    || Math.abs(gamepad1.left_stick_y) > CANCEL_STICK
                    || Math.abs(gamepad1.right_stick_x) > CANCEL_STICK;
            if (state == State.SEARCHING || state == State.APPROACHING) {
                if (gamepad1.b || sticksMoved) {
                    state = State.IDLE;
                    message = "Cancelled";
                }
            } else if (gamepad1.a) {
                state = seen ? State.APPROACHING : State.SEARCHING;
                stateTimer.reset();
                lastSeen.reset();
                inTolerance.reset();
                message = "";
            }

            // Errors as the camera sees them: positive distance = too far, positive lateral = tags
            // to the camera's right, positive heading = turned left of square (so turn right).
            // Turning is the same either way the camera faces; driving and strafing get flipped
            // below when it faces backward.
            double forwardError = 0, lateralError = 0, headingError = 0;
            boolean squared = false;
            if (seen) {
                double x, z;
                if (tagA != null && tagB != null) {
                    Position left = tagA.x < tagB.x ? tagA : tagB;
                    Position right = left == tagA ? tagB : tagA;
                    x = (left.x + right.x) / 2;
                    z = (left.z + right.z) / 2;
                    // Square to the tags: the robot faces them straight on when both are equally far
                    headingError = Math.toDegrees(Math.atan2(left.z - right.z, right.x - left.x));
                    squared = true;
                } else {
                    Position only = tagA != null ? tagA : tagB;
                    x = only.x;
                    z = only.z;
                }
                // A backward camera's right is the robot's left
                lateralError = x + (CAMERA_FACES_BACKWARD ? -CAMERA_RIGHT_OF_CENTER_IN : CAMERA_RIGHT_OF_CENTER_IN);
                forwardError = z - CAMERA_INSIDE_EDGE_IN - TARGET_DISTANCE_IN;
                if (!squared) {
                    // One tag: no way to square up, so turn to face it instead of strafing
                    headingError = Math.toDegrees(Math.atan2(lateralError, z));
                }
                searchDirection = Math.signum(lateralError) == 0 ? searchDirection : Math.signum(lateralError);
                lastSeen.reset();
            }

            double forward = 0, strafe = 0, turn = 0;
            switch (state) {
                case IDLE:
                case DONE:
                    forward = -gamepad1.left_stick_y;
                    strafe = gamepad1.left_stick_x;
                    turn = gamepad1.right_stick_x;
                    break;

                case SEARCHING:
                    if (seen) {
                        state = State.APPROACHING;
                        inTolerance.reset();
                    } else if (stateTimer.seconds() > SEARCH_TIMEOUT_S) {
                        state = State.IDLE;
                        message = "Tags not found. Point the robot toward them and press A.";
                    } else {
                        turn = SEARCH_TURN * searchDirection;
                    }
                    break;

                case APPROACHING:
                    if (!seen) {
                        // Stand still briefly in case the camera just blinked, then search
                        if (lastSeen.seconds() > LOST_TIMEOUT_S) {
                            state = State.SEARCHING;
                            stateTimer.reset();
                        }
                        inTolerance.reset();
                        break;
                    }
                    boolean distanceOk = Math.abs(forwardError) < DISTANCE_TOLERANCE_IN;
                    boolean lateralOk = !squared || Math.abs(lateralError) < LATERAL_TOLERANCE_IN;
                    boolean headingOk = Math.abs(headingError) < HEADING_TOLERANCE_DEG;
                    if (!(distanceOk && lateralOk && headingOk)) inTolerance.reset();
                    if (inTolerance.seconds() > SETTLE_S) {
                        state = State.DONE;
                        message = squared ? "Arrived" : "Arrived, but only one tag was visible so the robot isn't squared up";
                        break;
                    }
                    // Toward the tags and the camera's right are the robot's backward and left
                    // when the camera faces backward
                    double cameraDirection = CAMERA_FACES_BACKWARD ? -1 : 1;
                    forward = distanceOk ? 0 : cameraDirection * control(FORWARD_KP * forwardError, MAX_DRIVE);
                    strafe = !squared || lateralOk ? 0 : cameraDirection * control(STRAFE_KP * lateralError, MAX_DRIVE);
                    turn = headingOk ? 0 : control(TURN_KP * headingError, MAX_TURN);
                    break;
            }
            drive(forward, strafe, turn);

            telemetry.addData("State", state);
            if (!message.isEmpty()) telemetry.addData("Status", message);
            telemetry.addData("Tag " + TAG_A, describe(tagA));
            telemetry.addData("Tag " + TAG_B, describe(tagB));
            if (seen) {
                telemetry.addData("Distance error (in)", "%.1f", forwardError);
                telemetry.addData("Lateral error (in)", "%.1f", lateralError);
                telemetry.addData(squared ? "Heading error (deg)" : "Aim error (deg)", "%.1f", headingError);
            }
            telemetry.addData("Drive", "fwd %.2f  strafe %.2f  turn %.2f", forward, strafe, turn);
            telemetry.addData("Camera pipeline", "%d", result == null ? -1 : result.getPipelineIndex());
            telemetry.update();
        }

        drive(0, 0, 0);
        limelight.stop();
    }

    /** Proportional output clipped to max, but never weaker than MIN_POWER so the robot keeps moving. */
    private static double control(double output, double max) {
        double clipped = Range.clip(output, -max, max);
        return Math.abs(clipped) < MIN_POWER ? Math.copySign(MIN_POWER, clipped) : clipped;
    }

    private static String describe(Position p) {
        return p == null ? "not seen" : String.format("%.1f in from camera, %.1f in to camera's right", p.z, p.x);
    }

    private void drive(double forward, double strafe, double turn) {
        double flPower = forward + strafe + turn;
        double frPower = forward - strafe - turn;
        double blPower = forward - strafe + turn;
        double brPower = forward + strafe - turn;

        // Keep ratios between wheels when any power exceeds 1
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower))));
        fL.setPower(flPower / max);
        fR.setPower(frPower / max);
        bL.setPower(blPower / max);
        bR.setPower(brPower / max);
    }

    private DcMotorEx initMotor(String name, DcMotor.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }
}
