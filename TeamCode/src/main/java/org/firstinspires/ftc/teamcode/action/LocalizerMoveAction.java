package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    private double rotateSpeed;
    private double lastMovementTime;

    public enum FollowPathMethod {
        LINEAR, // not gonna be implemented
        FAST, // only cares about final heading position
    }

    private LocalizerMoveActionParameters parameters;

    public static class LocalizerMoveActionParameters {

        public LocalizerMoveActionParameters(FollowPathMethod pathMethod, double fullSpeed, double preciseSpeed, double rotateSpeed) {
            this.pathMethod = pathMethod;
            this.fullSpeed = fullSpeed;
            this.preciseSpeed = preciseSpeed;
            this.rotateSpeed = rotateSpeed;
        }

        FollowPathMethod pathMethod;
        double fullSpeed;
        double preciseSpeed;
        double rotateSpeed;
        double slowdownDistanceInches = 12;
        double slowestDistanceInches = 4;
        double slowRotationThresholdDegrees = 15;
        double maxRotationSpeed = 0.5;
        double distanceToleranceInches = 0.7;
        double fastTargetDistanceToleranceInches = 3;
        double headingToleranceDegrees = 1;
        double speedStuckTimeoutMs = 750;
        double speedStuckBumpAmount = 0.15;
        double speedStuckDistanceThresholdInches = 0.2;
    }

    public LocalizerMoveAction(RobotTransform transform, LocalizerMoveActionParameters parameters) {
        this.transforms = new RobotTransform[] { transform };
        this.parameters = parameters;
        this.fullSpeed = parameters.fullSpeed;
        this.preciseSpeed = parameters.preciseSpeed;
        this.rotateSpeed = parameters.rotateSpeed;
    }

    public LocalizerMoveAction(RobotTransform[] transforms, LocalizerMoveActionParameters parameters) {
        this.transforms = transforms;
        this.parameters = parameters;
        this.fullSpeed = parameters.fullSpeed;
        this.preciseSpeed = parameters.preciseSpeed;
        this.rotateSpeed = parameters.rotateSpeed;
    }

    public LocalizerMoveAction(RobotTransform transform, double speed, double preciseSpeed, double rotateSpeed, FollowPathMethod pathMethod) {
        this.transforms = new RobotTransform[] { transform };
        this.parameters = new LocalizerMoveActionParameters(pathMethod, speed, preciseSpeed, rotateSpeed);
        this.fullSpeed = parameters.fullSpeed;
        this.preciseSpeed = parameters.preciseSpeed;
        this.rotateSpeed = parameters.rotateSpeed;
    }

    public LocalizerMoveAction(RobotTransform[] transforms, double speed, double preciseSpeed, double rotateSpeed, FollowPathMethod pathMethod) {
        this.transforms = transforms;
        this.parameters = new LocalizerMoveActionParameters(pathMethod, speed, preciseSpeed, rotateSpeed);
        this.fullSpeed = parameters.fullSpeed;
        this.preciseSpeed = parameters.preciseSpeed;
        this.rotateSpeed = parameters.rotateSpeed;
    }

    double rotationLimit(double rotation) {
        return Math.min(parameters.maxRotationSpeed, Math.max(rotation, -parameters.maxRotationSpeed));
    }

    double speedForDistance(double distance) {
        double slope = (preciseSpeed - fullSpeed) / (parameters.slowestDistanceInches - parameters.slowdownDistanceInches);
        return Math.min(fullSpeed, Math.max(slope * distance - preciseSpeed, preciseSpeed));
    }

    @Override
    public void init(RobotHardware hardware) {
        lastMovementTime = System.currentTimeMillis();
    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        // Get current target to follow
        RobotTransform currentTarget = transforms[currentTransformIndex];

        // Get current robot position and orientation
        Position currentPosition = hardware.localizer.getEstimatedPosition().position;
        Orientation currentOrientation = hardware.localizer.getEstimatedOrientation().orientation;
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

        boolean withinFastDistanceTolerance = distanceToTargetInInches <= parameters.fastTargetDistanceToleranceInches;
        boolean withinDistanceTolerance = distanceToTargetInInches <= parameters.distanceToleranceInches;
        boolean withinHeadingTolerance = Math.abs(angDiffToTargetHeadingDegrees) <= parameters.headingToleranceDegrees;
        boolean withinSlowdownDistance = distanceToFinalTargetInInches <= parameters.slowdownDistanceInches;

        switch (parameters.pathMethod) {
            case LINEAR:
                // For the final target, the robot needs to reach the correct target heading.
                robotMoveAngleRadians = Math.toRadians(angDiffBetweenForwardAndTargetPosDegrees);
                robotRotation = rotationLimit(angDiffToTargetHeadingDegrees / 180);

                if (withinDistanceTolerance && withinHeadingTolerance) {
                    currentTransformIndex++;
                } else if (withinDistanceTolerance) {
                    speed = 0;
                    if (Math.abs(angDiffToTargetHeadingDegrees) <= parameters.slowRotationThresholdDegrees) {
                        robotRotation = angDiffToTargetHeadingDegrees > 0 ? rotateSpeed : -rotateSpeed;
                    } else {
                        robotRotation = angDiffToTargetHeadingDegrees > 0 ? 2 * rotateSpeed : -2 * rotateSpeed;
                    }
                    
                } else {
                    if (distanceToTargetInInches <= parameters.slowdownDistanceInches) {
                        double slope = (preciseSpeed - fullSpeed) / (parameters.slowestDistanceInches - parameters.slowdownDistanceInches);
                        speed = Math.max(slope * distanceToTargetInInches - preciseSpeed, preciseSpeed);
                    }
                }
                break;
            case FAST:
                robotMoveAngleRadians = Math.toRadians(angDiffBetweenForwardAndTargetPosDegrees);
                robotRotation = rotationLimit(smallestDiffToTargetPosDegrees / 180);
                speed = Math.min(speedForDistance(distanceToFinalTargetInInches), speedForDistance(distanceToTargetInInches));

                // If we're not on the final target, we aren't really going to care about precisely getting in the right spot
                if (currentTransformIndex != (transforms.length - 1)) {
                    if (withinFastDistanceTolerance) {
                        currentTransformIndex++;
                    }
                } else {
                    if (withinDistanceTolerance && withinHeadingTolerance) {
                        currentTransformIndex++;
                    } else if (withinDistanceTolerance) {
                        speed = 0; // Don't move, just rotate
                        if (Math.abs(angDiffToTargetHeadingDegrees) <= parameters.slowRotationThresholdDegrees) {
                            robotRotation = angDiffToTargetHeadingDegrees > 0 ? rotateSpeed : -rotateSpeed;
                        } else {
                            robotRotation = angDiffToTargetHeadingDegrees > 0 ? 2 * rotateSpeed : -2 * rotateSpeed;
                        }
                    } else {
                        if (withinSlowdownDistance) {
                            robotRotation = angDiffToTargetHeadingDegrees / 180;
                        }
                    }
                }

                // If we're not moving, bump the power up
                double currentTime = System.currentTimeMillis();
                Localizer.EstimatedPosition lastPosition = hardware.localizer.getLastEstimatedPosition();
                if (lastPosition != null) {
                    if (hardware.localizer.getDeltaDistance(DistanceUnit.INCH) >= parameters.speedStuckDistanceThresholdInches) {
                        lastMovementTime = currentTime;
                    } else if (currentTime - lastMovementTime >= parameters.speedStuckTimeoutMs) {
                        fullSpeed += parameters.speedStuckBumpAmount;
                        preciseSpeed += parameters.speedStuckBumpAmount;
                        lastMovementTime = currentTime;
                    }
                }
                break;
        }

        if (currentTransformIndex >= transforms.length) {
            // Once the robot reached the target position, stop moving and end the action
            hardware.omniDrive.stopDrive();
            return true;
        } else {
            String progress = String.format("[LMA] Current Target (in/deg)", "X: %.1f Y: %.1f H: %.1f",
                    currentTarget.position.x,
                    currentTarget.position.y,
                    currentTarget.heading);
            hardware.telemetry.addLine(progress);
            hardware.telemetry.addData("[LMA] Speed", "%.1f", speed);
            hardware.telemetry.addData("[LMA] Move Angle", "%.1f deg", Math.toDegrees(robotMoveAngleRadians));
            hardware.telemetry.addData("[LMA] Rotation", "%.1f", robotRotation);
            hardware.telemetry.addData("[LMA] ABP", "%.1f deg", angDiffBetweenPositionsDegrees);
            hardware.telemetry.addData("[LMA] AFTT", "%.1f deg", angDiffBetweenForwardAndTargetPosDegrees);
            hardware.telemetry.addData("[LMA] ATTH", "%.1f deg", angDiffToTargetHeadingDegrees);

            hardware.omniDrive.move(speed, robotMoveAngleRadians, robotRotation);
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
