package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;
import org.firstinspires.ftc.teamcode.playmaker.Localizer.RobotTransform;

import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class LocalizerMoveAction implements Action {

    RobotTransform[] transforms;
    private int currentTransformIndex = 0;
    private double fullSpeed;
    private double preciseSpeed;
    private double rotateSpeed = ROTATE_SPEED;
    private double previousHeadingInDegrees = 0;
    private double previousHeadingTime = 0;
    private FollowPathMethod followPathMethod;

    // Linear Path Following Configuration
    static final double SLOWDOWN_DISTANCE_INCHES = 12;
    static final double SLOWEST_DISTANCE_INCHES = 4;
    static final double ROTATE_SPEED = 0.275;
    static final double SLOW_ROTATE_THRESHOLD_DEGREES = 15;
    static final double MAX_ROTATION_AMOUNT = 0.5;


    // Tolerances
    static final double DISTANCE_TOLERANCE_INCHES = 0.7;
    static final double FAST_DISTANCE_TOLERANCE_INCHES = 3;
    static final double HEADING_TOLERANCE_DEGREES = 1;

    // Speed Up
    static final double SPEED_STUCK_TIMEOUT = 750;
    static final double SPEED_STUCK_BUMP_AMOUNT = 0.15;
    static final double SPEED_STUCK_DIST_THERSHOLD = 0.2;
    Position lastPosition;
    double lastMovementTime;

    public enum FollowPathMethod {
        LINEAR, // not gonna be implemented
        FAST, // only cares about final heading position
    }

    public LocalizerMoveAction(RobotTransform transform, double speed, double preciseSpeed, FollowPathMethod pathMethod) {
        this.transforms = new RobotTransform[] { transform };
        this.fullSpeed = speed;
        this.preciseSpeed = preciseSpeed;
        this.followPathMethod  = pathMethod;
    }

    public LocalizerMoveAction(RobotTransform[] transforms, double speed, double preciseSpeed, FollowPathMethod pathMethod) {
        this.transforms = transforms;
        this.fullSpeed = speed;
        this.preciseSpeed = preciseSpeed;
        this.followPathMethod = pathMethod;
    }

    @Override
    public void init(RobotHardware hardware) {
        lastMovementTime = System.currentTimeMillis();
    }

    double rotationLimit(double rotation) {
        return Math.min(MAX_ROTATION_AMOUNT, Math.max(rotation, -MAX_ROTATION_AMOUNT));
    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        // Get current target to follow
        RobotTransform currentTarget = transforms[currentTransformIndex];

        // Get current robot position and orientation
        Position currentPosition = hardware.localizer.estimatePosition().position;
        Orientation currentOrientation = hardware.localizer.estimateOrientation().orientation;
        if (currentPosition == null || currentOrientation == null) {
            // If either of these are null, it means that the robot's location is unknown
            // This point shouldn't be reached with input from the encoders
            // Return false so that the robot has the opportunity to localize itself again
            return false;
        }
        double currentHeading = currentOrientation.thirdAngle;
        double currentBackwardsHeading = Localizer.headingWrapInDegrees(currentHeading - 180);

        // Calculate distances / angles to target
        double distanceToTargetInInches = Localizer.distance(currentPosition, currentTarget.position, DistanceUnit.INCH);

        double distanceToFinalTargetInInches = Localizer.distance(currentPosition, this.transforms[this.transforms.length-1].position, DistanceUnit.INCH);

        // This is the angular difference between the robot's current position and the target position in the field coordinate space.
        double angDiffBetweenPositionsDegrees = Localizer.atan2InDegrees(currentPosition, currentTarget.position);

        // This is the angular difference between the robot's forward heading and the target heading.
        double angDiffToTargetHeadingDegrees = Localizer.angularDifferenceInDegrees(currentHeading, currentTarget.heading);

        // This is the angular difference between the robot's forward heading and the heading towards the target position.
        double angDiffBetweenForwardAndTargetPosDegrees = Localizer.angularDifferenceInDegrees(currentHeading, angDiffBetweenPositionsDegrees);
        double angDiffBetweenBackwardAndTargetPosDegrees = Localizer.angularDifferenceInDegrees(currentBackwardsHeading, angDiffBetweenPositionsDegrees);
        double smallestDiffToTargetPosDegrees;
        if (Math.abs(angDiffBetweenForwardAndTargetPosDegrees) < Math.abs(angDiffBetweenBackwardAndTargetPosDegrees)){
            smallestDiffToTargetPosDegrees = angDiffBetweenForwardAndTargetPosDegrees;
        } else {
            smallestDiffToTargetPosDegrees = angDiffBetweenBackwardAndTargetPosDegrees;
        }

        double robotMoveAngleRadians = 0;
        double robotRotation = 0;
        double speed = this.fullSpeed;

        boolean withinFastDistanceTolerance = distanceToTargetInInches <= FAST_DISTANCE_TOLERANCE_INCHES;
        boolean withinDistanceTolerance = distanceToTargetInInches <= DISTANCE_TOLERANCE_INCHES;
        boolean withinHeadingTolerance = Math.abs(angDiffToTargetHeadingDegrees) <= HEADING_TOLERANCE_DEGREES;

        switch (followPathMethod) {
            case LINEAR:
                // For the final target, the robot needs to reach the correct target heading.
                robotMoveAngleRadians = Math.toRadians(angDiffBetweenForwardAndTargetPosDegrees);
                robotRotation = rotationLimit(angDiffToTargetHeadingDegrees / 180);

                if (withinDistanceTolerance && withinHeadingTolerance) {
                    currentTransformIndex++;
                } else if (withinDistanceTolerance) {
                    speed = 0;
                    if (Math.abs(angDiffToTargetHeadingDegrees) <= SLOW_ROTATE_THRESHOLD_DEGREES) {
                        robotRotation = angDiffToTargetHeadingDegrees > 0 ? ROTATE_SPEED : -ROTATE_SPEED;
                    } else {
                        robotRotation = angDiffToTargetHeadingDegrees > 0 ? 2 * ROTATE_SPEED : -2 * ROTATE_SPEED;
                    }
                    
                } else {
                    if (distanceToTargetInInches <= SLOWDOWN_DISTANCE_INCHES) {
                        double slope = (preciseSpeed - fullSpeed) / (SLOWEST_DISTANCE_INCHES - SLOWDOWN_DISTANCE_INCHES);
                        speed = Math.max(slope * distanceToTargetInInches - preciseSpeed, preciseSpeed);
                    }
                }
                break;
            case FAST:
                if (currentTransformIndex != (transforms.length - 1)) {
                    // For fast operation, we only care about reaching the target heading for the final position.
                    // So, if the robot is heading towards the last target, it will focus on reaching
                    // the target as quickly as possible by going in the forward direction


                    robotMoveAngleRadians = Math.toRadians(angDiffBetweenForwardAndTargetPosDegrees);
                    robotRotation = rotationLimit(smallestDiffToTargetPosDegrees / 180);

                    if (distanceToFinalTargetInInches <= SLOWDOWN_DISTANCE_INCHES) {
                        double slope = (preciseSpeed - fullSpeed) / (SLOWEST_DISTANCE_INCHES - SLOWDOWN_DISTANCE_INCHES);
                        speed = Math.max(slope * distanceToTargetInInches - preciseSpeed, preciseSpeed);

                        double currentTime = System.currentTimeMillis();
                        if (lastPosition != null) {
                            double deltaDistance = Localizer.distance(lastPosition, currentPosition, DistanceUnit.INCH);
                            if (deltaDistance >= SPEED_STUCK_DIST_THERSHOLD) {
                                lastMovementTime = currentTime;
                            }

                            if (currentTime - lastMovementTime >= SPEED_STUCK_TIMEOUT) {
                                fullSpeed += SPEED_STUCK_BUMP_AMOUNT;
                                preciseSpeed += SPEED_STUCK_BUMP_AMOUNT;
                                lastMovementTime = currentTime;
                            }
                        }
                        lastPosition = currentPosition;
                    }

                    if (withinFastDistanceTolerance) {
                        currentTransformIndex++;
                    }
                } else {
                    // For the final target, the robot needs to reach the correct target heading.
                    robotMoveAngleRadians = Math.toRadians(angDiffBetweenForwardAndTargetPosDegrees);
                    robotRotation = rotationLimit(smallestDiffToTargetPosDegrees / 180);

                    if (withinDistanceTolerance && withinHeadingTolerance) {
                        currentTransformIndex++;
                    } else if (withinDistanceTolerance) {
                        speed = 0;
                        if (Math.abs(angDiffToTargetHeadingDegrees) <= SLOW_ROTATE_THRESHOLD_DEGREES) {
                            robotRotation = angDiffToTargetHeadingDegrees > 0 ? rotateSpeed : -rotateSpeed;
                        } else {
                            robotRotation = angDiffToTargetHeadingDegrees > 0 ? 2 * rotateSpeed : -2 * rotateSpeed;
                        }
                    } else {
                        if (distanceToTargetInInches <= SLOWDOWN_DISTANCE_INCHES) {
                            robotRotation = angDiffToTargetHeadingDegrees / 180;
                            double slope = (preciseSpeed - fullSpeed) / (SLOWEST_DISTANCE_INCHES - SLOWDOWN_DISTANCE_INCHES);
                            speed = Math.max(slope * distanceToTargetInInches - preciseSpeed, preciseSpeed);

                            double currentTime = System.currentTimeMillis();
                            if (lastPosition != null) {
                                double deltaDistance = Localizer.distance(lastPosition, currentPosition, DistanceUnit.INCH);
                                if (deltaDistance >= SPEED_STUCK_DIST_THERSHOLD) {
                                    lastMovementTime = currentTime;
                                }

                                if (currentTime - lastMovementTime >= SPEED_STUCK_TIMEOUT) {
                                    fullSpeed += SPEED_STUCK_BUMP_AMOUNT;
                                    preciseSpeed += SPEED_STUCK_BUMP_AMOUNT;
                                    lastMovementTime = currentTime;
                                }
                            }
                            lastPosition = currentPosition;
                        }
                    }
                }

                break;
        }

        if (currentTransformIndex >= transforms.length) {
            // Once the robot reached the target position, stop moving and end the action
            hardware.omniDrive.stopDrive();
            return true;
        } else {
            String progress = String.format("Current Target X: %.1f Y: %.1f H: %.1f",
                    currentTarget.position.x,
                    currentTarget.position.y,
                    currentTarget.heading);
            hardware.telemetry.addLine(progress);
            hardware.telemetry.addData("LMA Speed", speed);
            hardware.telemetry.addData("LMA Move Angle", Math.toDegrees(robotMoveAngleRadians));
            hardware.telemetry.addData("LMA Rotation", robotRotation);
            hardware.telemetry.addData("LMA ABP", angDiffBetweenPositionsDegrees);
            hardware.telemetry.addData("LMA AFTT", angDiffBetweenForwardAndTargetPosDegrees);
            hardware.telemetry.addData("LMA ATTH", angDiffToTargetHeadingDegrees);

            hardware.omniDrive.move(speed, robotMoveAngleRadians, robotRotation);
            previousHeadingInDegrees = currentHeading;
            previousHeadingTime = System.currentTimeMillis();
            return false;
        }
    }

    @Override
    public Double progress() {
        return null;
    }

    @Override
    public String progressString() {
        return null;
    }

    @Override
    public Object getActionResult() {
        return null;
    }
}
