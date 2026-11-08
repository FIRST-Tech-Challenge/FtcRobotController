package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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
     * Go to a spot on the field AND face a given direction. Driving and turning happen together,
     * each at a fixed power, and each stops on its own once it is close enough.
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
        boolean atPosition = false;
        boolean atHeading = false;

        while (timer.seconds() < timeout) {

            pinpoint.update();                      // nothing updates until you call this
            Pose2D pose = pinpoint.getPosition();
            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.DEGREES);

            double errorX = targetX - x;            // how far off we are, in FIELD coordinates
            double errorY = targetY - y;
            double distance = Math.hypot(errorX, errorY);

            // Shortest way round: 170 to -170 is a 20 degree turn, not 340.
            double headingError = AngleUnit.normalizeDegrees(targetHeading - heading);

            // Once we arrive, stay arrived. Without this the robot would chase an overshoot back
            // and forth across the target until the timeout.
            if (distance < positionTolerance) atPosition = true;
            if (Math.abs(headingError) < headingTolerance) atHeading = true;

            showPose();
            telemetry.addData("Target", "x: %.1f  y: %.1f  heading: %.1f",
                    targetX, targetY, targetHeading);
            telemetry.addData("Robot",  "x: %.1f  y: %.1f  heading: %.1f",
                    x, y, heading);
            telemetry.addData("Error", "%.1f in   %.1f deg", distance, headingError);
            telemetry.update();

            if (atPosition && atHeading) break;

            double power = atPosition ? 0 : drivePower;
            double turn = atHeading ? 0 : turnPower * Math.signum(headingError);

            driveFieldRelative(errorX, errorY, heading, -power, turn);
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
