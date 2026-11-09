package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.RefCounted;

public class Drive {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private GoBildaPinpointDriver pinpoint;
    private Telemetry telemetry;

    private final double DEFAULT_DRIVE_POWER = 0.5;
    private final double DEFAULT_TURN_POWER = 0.5;
    private final double DEFAULT_POSITION_TOLERANCE = 0.5; // in inches
    private final double DEFAULT_HEADING_TOLERANCE = 3; // in degrees
    private final double DEFAULT_TIMEOUT = 5.0; // in seconds

    /**
     * false (default): bang-bang -- full set power until inside tolerance, then stop.
     * true: PID -- power shrinks as the target gets closer and brakes against speed.
     * In both modes drivePower and turnPower are the top speed.
     */
    private boolean usePid = false;

    // PID gains. Starting guesses -- tune on the robot. Public static so they are easy to tweak
    // from a test OpMode (or FTC Dashboard, if you add it).
    public static double KP_DRIVE = 0.04;    // power per inch of distance (0.4 at 10")
    public static double KD_DRIVE = 0.004;   // braking power per in/s of speed toward target
    public static double KS_DRIVE = 0.15;    // extra push to overcome friction (static feedforward)
    public static double KP_TURN = 0.012;    // power per degree of heading error
    public static double KD_TURN = 0.001;    // braking power per deg/s of turning
    public static double KS_TURN = 0.06;     // extra turn push to overcome friction

    // PID "settled" test: inside tolerance AND slower than these for SETTLE_TIME.
    public static double SETTLE_SPEED = 1.0;        // in/s
    public static double SETTLE_TURN_SPEED = 5.0;   // deg/s
    public static double SETTLE_TIME = 0.1;         // s
    public static double VELOCITY_SMOOTHING = 0.5;  // 0 = raw, closer to 1 = smoother but laggier

    public Drive(
            DcMotor frontLeft,
            DcMotor frontRight,
            DcMotor backLeft,
            DcMotor backRight,
            GoBildaPinpointDriver pinpoint,
            Telemetry telemetry
    ) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.pinpoint = pinpoint;
        this.telemetry = telemetry;
    }

    /**
     * Mecanum mixing: turn "go this way and spin this much" into four motor powers.
     *
     * @param forward +1 is straight out the front of the robot, -1 is backwards
     * @param right   +1 is sideways to the robot's right, -1 is left
     * @param turn    +1 is counterclockwise, -1 is clockwise
     */
    public void drive(double forward, double right, double turn) {
        double fl = forward + right - turn;
        double bl = forward - right - turn;
        double fr = forward - right + turn;
        double br = forward + right + turn;

        // Scale all four down together if any exceeds 1, so the direction is kept, just slower.
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }

    /**
     * Go to a spot on the field AND face a given direction. Driving and turning happen together.
     * In bang-bang mode each runs at a fixed power and stops on its own once close enough; in PID
     * mode (setUsePid(true)) each slows as it closes in, and the move ends once the robot has
     * settled inside both tolerances.
     */
    public void driveTo(
            double targetX,
            double targetY,
            double targetHeading,
            double drivePower,
            double turnPower,
            double positionTolerance,
            double headingTolerance,
            double timeout
    ) {

        ElapsedTime timer = new ElapsedTime();
        boolean atPosition = false;                 // bang-bang mode only
        boolean atHeading = false;

        // PID mode only: field velocity estimated from successive positions, and how long the
        // robot has been settled on the target.
        double lastX = Double.NaN, lastY = 0, lastTime = 0;
        double velX = 0, velY = 0;
        double settledSince = -1;

        // Also quit if STOP is pressed: the SDK interrupts the OpMode thread, and without this
        // check the robot would keep driving until the timeout.
        while (timer.seconds() < timeout && !Thread.currentThread().isInterrupted()) {

            pinpoint.update();                      // nothing updates until you call this
            Pose2D pose = pinpoint.getPosition();
            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.DEGREES);
            double now = timer.seconds();

            double errorX = targetX - x;            // how far off we are, in FIELD coordinates
            double errorY = targetY - y;
            double distance = Math.hypot(errorX, errorY);

            // Shortest way round: 170 to -170 is a 20 degree turn, not 340.
            double headingError = AngleUnit.normalizeDegrees(targetHeading - heading);

            double power;
            double turn;

            if (usePid) {
                // --- PID: push in proportion to what is left, brake in proportion to speed ---

                // Field velocity from the change in position, lightly smoothed.
                if (!Double.isNaN(lastX) && now > lastTime) {
                    double dt = now - lastTime;
                    velX = VELOCITY_SMOOTHING * velX + (1 - VELOCITY_SMOOTHING) * (x - lastX) / dt;
                    velY = VELOCITY_SMOOTHING * velY + (1 - VELOCITY_SMOOTHING) * (y - lastY) / dt;
                }
                lastX = x;
                lastY = y;
                lastTime = now;
                double speed = Math.hypot(velX, velY);
                double headingVel = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

                // Done only when inside both tolerances AND nearly stopped, for a short while.
                // Otherwise the robot could coast through the target and still count as there.
                boolean inside = distance < positionTolerance
                        && Math.abs(headingError) < headingTolerance;
                boolean still = speed < SETTLE_SPEED && Math.abs(headingVel) < SETTLE_TURN_SPEED;
                if (inside && still) {
                    if (settledSince < 0) settledSince = now;
                    if (now - settledSince >= SETTLE_TIME) break;
                } else {
                    settledSince = -1;
                }

                // Speed toward the target: the part of the velocity along the error direction.
                double towardSpeed = distance > 0 ? (velX * errorX + velY * errorY) / distance : 0;

                power = KP_DRIVE * distance - KD_DRIVE * towardSpeed;
                if (distance > positionTolerance) power += KS_DRIVE;
                power = Range.clip(power, -drivePower, drivePower); // negative = braking

                turn = KP_TURN * headingError - KD_TURN * headingVel;
                if (Math.abs(headingError) > headingTolerance) {
                    turn += KS_TURN * Math.signum(headingError);
                }
                turn = Range.clip(turn, -turnPower, turnPower);

            } else {
                // --- Bang-bang: fixed power until inside tolerance, then off ---

                // Once we arrive, stay arrived. Without this the robot would chase an overshoot
                // back and forth across the target until the timeout.
                if (distance < positionTolerance) atPosition = true;
                if (Math.abs(headingError) < headingTolerance) atHeading = true;

                if (atPosition && atHeading) break;

                power = atPosition ? 0 : drivePower;
                turn = atHeading ? 0 : turnPower * Math.signum(headingError);
            }

            showPose();
            telemetry.addData("Mode", usePid ? "PID" : "Bang-bang");
            telemetry.addData("Target", "x: %.1f  y: %.1f  heading: %.1f",
                    targetX, targetY, targetHeading);
            telemetry.addData("Robot",  "x: %.1f  y: %.1f  heading: %.1f",
                    x, y, heading);
            telemetry.addData("Error", "%.1f in   %.1f deg", distance, headingError);
            telemetry.addData("Output", "power %.2f   turn %.2f", power, turn);
            telemetry.update();

            driveFieldRelative(errorX, errorY, heading, power, turn);
        }

        stopDriving();
    }

    public void driveTo(
            double targetX,
            double targetY,
            double targetHeading
    ) {
        driveTo(
                targetX,
                targetY,
                targetHeading,
                DEFAULT_DRIVE_POWER,
                DEFAULT_TURN_POWER,
                DEFAULT_POSITION_TOLERANCE,
                DEFAULT_HEADING_TOLERANCE,
                DEFAULT_TIMEOUT
        );
    }

    public void driveTo(
            double targetX,
            double targetY,
            double targetHeading,
            double drivePower
    ) {
        driveTo(
                targetX,
                targetY,
                targetHeading,
                drivePower,
                DEFAULT_TURN_POWER,
                DEFAULT_POSITION_TOLERANCE,
                DEFAULT_HEADING_TOLERANCE,
                DEFAULT_TIMEOUT
        );
    }

    public void driveTo(
            double targetX,
            double targetY,
            double targetHeading,
            double drivePower,
            double turnPower
    ) {
        driveTo(
                targetX,
                targetY,
                targetHeading,
                drivePower,
                turnPower,
                DEFAULT_POSITION_TOLERANCE,
                DEFAULT_HEADING_TOLERANCE,
                DEFAULT_TIMEOUT
        );
    }

    public void driveTo(
            double targetX,
            double targetY,
            double targetHeading,
            double drivePower,
            double turnPower,
            double timeoutInSeconds
    ) {
        driveTo(
                targetX,
                targetY,
                targetHeading,
                drivePower,
                turnPower,
                DEFAULT_POSITION_TOLERANCE,
                DEFAULT_HEADING_TOLERANCE,
                timeoutInSeconds
        );
    }
    /**
     * Drive toward a direction measured on the FIELD, whichever way the robot happens to be pointed.
     *
     * The motors only understand "forward" and "right," and those swing around as the robot turns.
     * Rotating the field vector by the robot's heading converts one into the other -- it is a plain
     * 2-D rotation. Dividing by the length throws away the distance and keeps only the direction;
     * power decides the speed.
     *
     * The turn is NOT rotated -- spinning means the same thing in both frames. It just rides along,
     * which is what lets the robot travel a straight line on the field while it turns.
     *
     * @param fieldX  x part of the direction to travel, in field coordinates
     * @param fieldY  y part of the direction to travel, in field coordinates
     * @param heading robot's current heading, degrees
     * @param power   how hard to push, 0 to 1
     * @param turn    spin power, + is counterclockwise
     */
    public void driveFieldRelative(double fieldX, double fieldY, double heading, double power, double turn) {
        double h = Math.toRadians(heading);
        double length = Math.hypot(fieldX, fieldY);

        double forward = 0;                         // already on the spot? then there is no
        double right = 0;                           // direction to convert -- just turn
        if (length > 0) {
            forward = (fieldX * Math.cos(h) + fieldY * Math.sin(h)) / length;
            right = (fieldX * Math.sin(h) - fieldY * Math.cos(h)) / length;
        }

        drive(forward * power, right * power, turn);
    }

    /** Choose how driveTo() controls power: true for PID, false for bang-bang. */
    public void setUsePid(boolean usePid) {
        this.usePid = usePid;
    }

    public boolean isUsingPid() {
        return usePid;
    }

    /** Stop, robot! Stop! */
    public void stopDriving() {
        drive(0, 0, 0);
    }

    /** Put the robot's current field position on the driver station. */
    public void showPose() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        telemetry.addData("Pose", "x: %.1f  y: %.1f  heading: %.1f",
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES));
    }
}
