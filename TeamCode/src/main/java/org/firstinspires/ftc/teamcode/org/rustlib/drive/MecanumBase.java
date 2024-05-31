package org.firstinspires.ftc.teamcode.org.rustlib.drive;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.org.rustlib.control.PIDController;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Vector2d;

import java.util.function.Supplier;

public class MecanumBase {
    private final DcMotor lf;
    private final DcMotor rf;
    private final DcMotor lb;
    private final DcMotor rb;
    private final double maxEndpointErr;
    private final double trackEndpointHeadingMaxDistance;
    private final double calculateTargetHeadingMinDistance;
    private final double maxFinalVelocityInPerSec;
    private int waypointIndex = 0;
    private Path toFollow;
    private final ElapsedTime timer = new ElapsedTime();
    private Pose2d lastPose = new Pose2d();
    private double lastTimestamp = 0;
    private double followStartTimestamp;
    private Waypoint[][] segments;
    private double lastTargetAngle = 0;

    public enum DriveState {
        IDLE,
        FOLLOWING,
    }

    public DriveState driveState = DriveState.IDLE;
    private final Supplier<Pose2d> poseSupplier;
    public final PIDController driveController = new PIDController();
    public final PIDController rotController = new PIDController();

    private MecanumBase(Builder builder) {
        lf = builder.leftFront;
        rf = builder.rightFront;
        lb = builder.leftBack;
        rb = builder.rightBack;
        poseSupplier = builder.poseSupplier;
        maxEndpointErr = builder.maxEndpointErr;
        trackEndpointHeadingMaxDistance = builder.trackEndpointHeadingMaxDistance;
        calculateTargetHeadingMinDistance = builder.calculateTargetHeadingMinDistance;
        maxFinalVelocityInPerSec = builder.maxFinalVelocityInPerSec;
        driveController.setGains(builder.driveGains);
        rotController.setGains(builder.rotGains);
    }

    public interface LeftFront {
        RightFront defineLeftFront(DcMotor motor, boolean reversed);

        RightFront defineLeftFront(DcMotor motor);
    }

    public interface RightFront {
        LeftBack defineRightFront(DcMotor motor, boolean reversed);

        LeftBack defineRightFront(DcMotor motor);
    }

    public interface LeftBack {
        RightBack defineLeftBack(DcMotor motor, boolean reversed);

        RightBack defineLeftBack(DcMotor motor);
    }

    public interface RightBack {
        Pose defineRightBack(DcMotor motor, boolean reversed);

        Pose defineRightBack(DcMotor motor);
    }

    public interface Pose {
        Builder setPoseSupplier(Supplier<Pose2d> poseSupplier);
    }

    public static class Builder implements LeftFront, RightFront, LeftBack, RightBack, Pose {
        private DcMotor leftFront;
        private DcMotor rightFront;
        private DcMotor leftBack;
        private DcMotor rightBack;
        private Supplier<Pose2d> poseSupplier;
        private double maxEndpointErr = 0.5;
        private double trackEndpointHeadingMaxDistance = 12.0;
        private double calculateTargetHeadingMinDistance = 15.0;
        private double maxFinalVelocityInPerSec = 1.0;
        private PIDController.PIDGains driveGains = new PIDController.PIDGains(0.1, 0.002, 0.00001);
        private PIDController.PIDGains rotGains = new PIDController.PIDGains(1.0, 0, 0);

        private Builder() {

        }

        private void reverseIf(DcMotor motor, boolean reversed) {
            if (reversed) {
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
        }

        @Override
        public RightFront defineLeftFront(DcMotor motor, boolean reversed) {
            leftFront = motor;
            reverseIf(motor, reversed);
            return this;
        }

        @Override
        public RightFront defineLeftFront(DcMotor motor) {
            defineLeftFront(motor, false);
            return this;
        }

        @Override
        public LeftBack defineRightFront(DcMotor motor, boolean reversed) {
            rightFront = motor;
            reverseIf(motor, reversed);
            return this;
        }

        @Override
        public LeftBack defineRightFront(DcMotor motor) {
            defineRightFront(motor, false);
            return this;
        }

        @Override
        public RightBack defineLeftBack(DcMotor motor, boolean reversed) {
            leftBack = motor;
            reverseIf(motor, reversed);
            return this;
        }

        @Override
        public RightBack defineLeftBack(DcMotor motor) {
            defineLeftBack(motor, false);
            return this;
        }

        @Override
        public Pose defineRightBack(DcMotor motor, boolean reversed) {
            rightBack = motor;
            reverseIf(motor, reversed);
            return this;
        }

        @Override
        public Pose defineRightBack(DcMotor motor) {
            defineRightBack(motor, false);
            return this;
        }

        @Override
        public Builder setPoseSupplier(Supplier<Pose2d> supplier) {
            poseSupplier = supplier;
            return this;
        }

        public Builder setMaxEndpointErr(double maxErr) {
            maxEndpointErr = maxErr;
            return this;
        }

        public Builder setUseEndpointHeadingDistance(double distance) {
            trackEndpointHeadingMaxDistance = distance;
            return this;
        }

        public Builder setTargetHeadingCalculationDistance(double distance) {
            calculateTargetHeadingMinDistance = distance;
            return this;
        }

        public Builder setMaxFinalVelocity(double velocity) {
            maxFinalVelocityInPerSec = velocity;
            return this;
        }

        public Builder setDriveGains(PIDController.PIDGains gains) {
            driveGains = gains;
            return this;
        }

        public Builder setRotGains(PIDController.PIDGains gains) {
            rotGains = gains;
            return this;
        }

        public MecanumBase build() {
            return new MecanumBase(this);
        }
    }

    public static LeftFront getBuilder() {
        return new Builder();
    }

    public void enableBraking() {
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void disableBraking() {
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void drive(double drive, double strafe, double turn, double botHeading, boolean squareInputs) {
        if (squareInputs) {
            drive = Math.copySign(Math.pow(drive, 2), drive);
            strafe = Math.copySign(Math.pow(strafe, 2), strafe);
            turn = Math.copySign(Math.pow(turn, 2), turn);
        }
        double originalDrive = drive;
        drive = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);
        strafe = strafe * Math.cos(-botHeading) - originalDrive * Math.sin(-botHeading);
        strafe *= 1.414;

        double[] wheelSpeeds = { // Order: lf, rf, lb, rb
                drive + strafe - turn,
                drive - strafe + turn,
                drive - strafe - turn,
                drive + strafe + turn
        };
        double largest = 1.0;
        for (double wheelSpeed : wheelSpeeds) {
            if (Math.abs(wheelSpeed) > largest) {
                largest = Math.abs(wheelSpeed);
            }
        }
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] /= largest;
        }

        lf.setPower(wheelSpeeds[0]);
        rf.setPower(wheelSpeeds[1]);
        lb.setPower(wheelSpeeds[2]);
        rb.setPower(wheelSpeeds[3]);
    }

    public void drive(double drive, double strafe, double turn, double botHeading) {
        drive(drive, strafe, turn, botHeading, true);
    }

    public void drive(double drive, double strafe, double turn) {
        drive(drive, strafe, turn, 0);
    }

    private static double getTValue(Vector2d point1, Vector2d point2, Vector2d interpolationPoint) {
        if (point1.x == point2.x) {
            return (interpolationPoint.y - point1.y) / (point2.y - point1.y);
        }
        return (interpolationPoint.x - point1.x) / (point2.x - point1.x);
    }

    public void setFollowPath(Path path) {
        waypointIndex = 0;
        driveState = DriveState.IDLE;
        toFollow = path;
        segments = toFollow.generateLineSegments();
        followStartTimestamp = timer.milliseconds();
    }

    public void driveToPosition(Waypoint targetPoint, boolean useEndpointHeading) {
        Pose2d botPose = poseSupplier.get();
        Vector2d relativeTargetVector = (new Vector2d(targetPoint.x - botPose.x, targetPoint.y - botPose.y));
        Vector2d movementSpeed = (new Vector2d(driveController.calculate(0, relativeTargetVector.magnitude), relativeTargetVector.angle, false)).rotate(-botPose.rotation.getAngleRadians());

        double rotSpeed;
        double targetAngle;

        boolean canFlip = false;

        if (useEndpointHeading && targetPoint.targetEndRotation != null && botPose.distanceTo(targetPoint) < trackEndpointHeadingMaxDistance) {
            targetAngle = targetPoint.targetEndRotation.getAngleRadians();
        } else if (targetPoint.targetFollowRotation != null) {
            targetAngle = targetPoint.targetFollowRotation.getAngleRadians();
        } else if (relativeTargetVector.magnitude > calculateTargetHeadingMinDistance) {
            targetAngle = relativeTargetVector.angle - Math.PI / 2;
            canFlip = true;
        } else {
            targetAngle = lastTargetAngle;
            canFlip = true;
        }
        lastTargetAngle = targetAngle;

        double rotError = Rotation2d.getError(targetAngle, botPose.rotation.getAngleRadians());
        if (rotError > Math.PI && canFlip) {
            rotError = Rotation2d.getError(targetAngle + Math.PI, botPose.rotation.getAngleRadians());
        }
        double magnitude = movementSpeed.magnitude / (1.5 * Math.pow(Math.abs(rotError), 2) + 1); // originally 0.9 * rotError ^ 2
        magnitude = Range.clip(magnitude, -targetPoint.maxVelocity, targetPoint.maxVelocity);
        movementSpeed = new Vector2d(magnitude, movementSpeed.angle, false);
        rotSpeed = rotController.calculate(0, rotError);

        drive(movementSpeed.y, movementSpeed.x, rotSpeed);
    }

    public void driveToPosition(Waypoint targetPoint) {
        driveToPosition(targetPoint, true);
    }

    private static Waypoint lineIntersection(Pose2d botPose, Waypoint[] lineSegment) {
        double intersectionX;
        double intersectionY;

        double m0 = (lineSegment[0].y - lineSegment[1].y) / (lineSegment[0].x - lineSegment[1].x);

        if (m0 == 0) { // If the path segment has a slope of zero
            intersectionX = botPose.x;
            intersectionY = lineSegment[0].y;
        } else if (lineSegment[0].x == lineSegment[1].x) { // If the path segment is vertical
            intersectionX = lineSegment[0].x;
            intersectionY = botPose.y;
        } else {
            double b0 = lineSegment[0].y - m0 * lineSegment[0].x;

            double m1 = -1 / m0;
            double b1 = botPose.y - m1 * botPose.x;

            intersectionX = (b0 - b1) / (m0 - m1);
            intersectionY = m0 * intersectionX + b0;


        }
        return new Waypoint(intersectionX, intersectionY, 0, lineSegment[1].targetFollowRotation, lineSegment[1].targetEndRotation, lineSegment[1].maxVelocity);
    }

    private static Waypoint lineCircleIntersection(Pose2d botPose, Waypoint[] lineSegment, double radius) {
        double x1;
        double y1;

        double x2;
        double y2;

        double m = (lineSegment[0].y - lineSegment[1].y) / (lineSegment[0].x - lineSegment[1].x);
        double b = lineSegment[0].y - m * lineSegment[0].x;

        double h = botPose.x;
        double k = botPose.y;

        double commonTerm;

        if (Double.isFinite(m)) {
            commonTerm = Math.sqrt(Math.pow(m, 2) * (Math.pow(radius, 2) - Math.pow(h, 2)) + (2 * m * h) * (k - b) + 2 * b * k + Math.pow(radius, 2) - Math.pow(b, 2) - Math.pow(k, 2));

            x1 = (m * (k - b) + h + commonTerm) / (Math.pow(m, 2) + 1);
            x2 = (m * (k - b) + h - commonTerm) / (Math.pow(m, 2) + 1);

            y1 = m * x1 + b;
            y2 = m * x2 + b;
        } else { // Vertical line
            x1 = lineSegment[0].x;
            commonTerm = Math.sqrt(Math.pow(radius, 2) - Math.pow((x1 - h), 2));
            y1 = botPose.y + commonTerm;
            x2 = x1;
            y2 = botPose.y - commonTerm;
        }

        Waypoint point0 = new Waypoint(x1, y1, 0, lineSegment[1].targetFollowRotation, lineSegment[1].targetEndRotation, lineSegment[1].maxVelocity);
        Waypoint point1 = new Waypoint(x2, y2, 0, lineSegment[1].targetFollowRotation, lineSegment[1].targetEndRotation, lineSegment[1].maxVelocity);

        Pair<Waypoint, Double> bestIntersection;

        if (point0.equals(Vector2d.undefined)) { // If the first intersection isn't a valid point, neither is the second.  This means the robot is too far away from the path for its bounding circle to intersect it
            Waypoint intersection = lineIntersection(botPose, lineSegment);
            bestIntersection = new Pair<>(intersection, getTValue(lineSegment[0], lineSegment[1], intersection));
        } else {
            Pair<Waypoint, Double> intersection0 = new Pair<>(point0, getTValue(lineSegment[0], lineSegment[1], point0));
            Pair<Waypoint, Double> intersection1 = new Pair<>(point1, getTValue(lineSegment[0], lineSegment[1], point1));

            bestIntersection = intersection0.second > intersection1.second ? intersection0 : intersection1;
        }

        if (bestIntersection.second > 1) {
            return null;
        }

        return bestIntersection.first;
    }

    public void followPath() {
        Pose2d botPose = poseSupplier.get();
        Waypoint targetPoint;
        boolean endOfPath = false;

        switch (driveState) {
            case IDLE:
                if (waypointIndex != 0) return;
                followStartTimestamp = timer.milliseconds();
                driveState = DriveState.FOLLOWING;
                disableBraking();
            case FOLLOWING:
                if (timer.milliseconds() > followStartTimestamp + toFollow.timeout) {
                    driveState = DriveState.IDLE;
                }

                targetPoint = lineCircleIntersection(botPose, segments[waypointIndex], segments[waypointIndex][1].followRadius);

                if (targetPoint == null) { // If null is returned, the t value of the target point is greater than 1 or less than 0
                    if (waypointIndex == segments.length - 1) {
                        targetPoint = segments[segments.length - 1][1]; // If the robot is already on the last segment, the target point will be the endpoint of that segment
                        endOfPath = true;
                    } else {
                        waypointIndex++;
                        followPath();
                        return;
                    }
                }

                driveToPosition(targetPoint, endOfPath);
        }
    }

    public boolean finishedFollowing() {
        try {
            if (timer.milliseconds() > followStartTimestamp + toFollow.timeout && timer.milliseconds() > followStartTimestamp + 1) {
                return true;
            }

            boolean atEndpoint;
            boolean atTargetHeading;

            Pose2d botPose = poseSupplier.get();
            double currentTimestamp = timer.milliseconds();
            double speed = botPose.distanceTo(lastPose) / (currentTimestamp - lastTimestamp) * 1000;

            lastTimestamp = currentTimestamp;

            atEndpoint = speed < maxFinalVelocityInPerSec
                    && botPose.distanceTo(segments[segments.length - 1][1]) < maxEndpointErr
                    && waypointIndex == segments.length - 1;
            if (segments[segments.length - 1][1].targetEndRotation == null) {
                atTargetHeading = true;
            } else {
                atTargetHeading = Math.abs(Rotation2d.getError(segments[segments.length - 1][1].targetEndRotation.getAngleRadians(), botPose.rotation.getAngleRadians())) < Math.toRadians(5);
            }

            lastPose = botPose;
            return atEndpoint && atTargetHeading;
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * @return An estimation of the remaining distance the robot needs to travel before completing the path.
     */
    public double remainingDistance() {
        if (driveState == DriveState.IDLE) {
            return 0;
        }
        double distance = poseSupplier.get().distanceTo(segments[waypointIndex][1]);
        for (int i = waypointIndex + 1; i < segments.length; i++) {
            distance += segments[i][0].distanceTo(segments[i][1]);
        }
        return distance;
    }

    public boolean atWaypoint(Waypoint waypoint, double maxLinearErr, double maxRotErr) {
        if (waypoint == null) {
            return false;
        }
        Pose2d botPose = poseSupplier.get();
        double rotErr;
        if (waypoint.targetEndRotation == null) {
            rotErr = 0;
        } else {
            rotErr = Math.abs(Rotation2d.getError(waypoint.targetEndRotation.getAngleRadians(), botPose.rotation.getAngleRadians()));
        }
        return botPose.distanceTo(waypoint) < maxLinearErr && rotErr < maxRotErr;
    }

}
