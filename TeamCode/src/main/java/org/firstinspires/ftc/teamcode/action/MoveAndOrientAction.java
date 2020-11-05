package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class MoveAndOrientAction implements Action {

    double startingAngle;
    private double overallDistanceToTarget;
    private double overallAngularDifference;
    private Position finalPosition;
    private double targetHeading;
    private double power;

    private final double DISTANCE_TOLERANCE = 1;
    private final DistanceUnit DISTANCE_TOLERANCE_UNIT = DistanceUnit.INCH;
    private final double HEADING_TOLERANCE = 2;
    private final double DISTANCE_UNTIL_SCALE_DOWN = 8;
    private final DistanceUnit DISTANCE_SCALE_DOWN_UNIT = DistanceUnit.INCH;


    public MoveAndOrientAction(Position finalPosition, double heading, double power) {
        this.finalPosition = finalPosition;
        this.targetHeading = heading;
        this.power = power;
    }

    @Override
    public void init(RobotHardware hardware) {
//        this.overallDistanceToTarget = Localizer.distance(finalPosition, hardware.localizer.estimatePosition());

//        startingAngle = hardware.localizer.estimateOrientation().thirdAngle;

    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        Position currentPosition = hardware.localizer.estimatePosition().position;
        Localizer.EstimatedOrientation currentEstimatedOrientation = hardware.localizer.estimateOrientation();
        if (currentPosition == null || currentEstimatedOrientation == null) {
            return false;
        }

        Orientation currentOrientation = currentEstimatedOrientation.orientation;
        Position inches = currentPosition.toUnit(DistanceUnit.INCH);
        double currentHeading = currentOrientation.thirdAngle;
        double currentDistanceToTarget = Localizer.distance(currentPosition, finalPosition, DistanceUnit.INCH);

        double angularDifferenceBetweenPositions = Localizer.atan2(currentPosition, finalPosition);
        double angularDifferenceToTargetHeading = Localizer.angularDifference(currentHeading, targetHeading);
        double angDiffBetweenForwardAndTargetPos = Localizer.angularDifference(currentHeading, angularDifferenceBetweenPositions);

        hardware.telemetry.addData("current pos", "%.1f %.1f %.1f", inches.x, inches.y, inches.z);
        hardware.telemetry.addData("current heading", currentHeading);
        hardware.telemetry.addData("dist to target", currentDistanceToTarget);
        hardware.telemetry.addData("ang between pos", angularDifferenceBetweenPositions);


        hardware.telemetry.addData("ang to target heading", angularDifferenceToTargetHeading);
        hardware.telemetry.addData("ang forward to target", angDiffBetweenForwardAndTargetPos);


        double  movePower;
        if (Localizer.distance(currentPosition, finalPosition, DISTANCE_SCALE_DOWN_UNIT) <= DISTANCE_UNTIL_SCALE_DOWN) {
            movePower = 0.3;
        } else {
            movePower = power;
        }

        hardware.omniDrive.move(movePower, Math.toRadians(angDiffBetweenForwardAndTargetPos), angularDifferenceToTargetHeading/180);

        boolean withinDistanceTolerance = Localizer.distance(currentPosition, finalPosition, DISTANCE_TOLERANCE_UNIT) <= DISTANCE_TOLERANCE;
        boolean withinHeadingTolerance = Localizer.angularDifference(currentHeading, targetHeading) <= HEADING_TOLERANCE;
        return withinDistanceTolerance && withinHeadingTolerance;
    }

    @Override
    public Double progress() {
        return null;
    }

    @Override
    public String progressString() {
        return null;
    }
}
