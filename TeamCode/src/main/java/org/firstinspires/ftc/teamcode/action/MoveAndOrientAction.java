package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;
import org.firstinspires.ftc.teamcode.playmaker.Localizer.RobotTransform;

public class MoveAndOrientAction implements Action {

    double startingAngle;
    private double overallDistanceToTarget;
    private double overallAngularDifference;
    private RobotTransform transform;
    private double power;

    private final double DISTANCE_TOLERANCE = 1;
    private final DistanceUnit DISTANCE_TOLERANCE_UNIT = DistanceUnit.INCH;
    private final double HEADING_TOLERANCE = 2;
    private final double DISTANCE_UNTIL_SCALE_DOWN = 8;
    private final DistanceUnit DISTANCE_SCALE_DOWN_UNIT = DistanceUnit.INCH;

    public MoveAndOrientAction(DistanceUnit unit, double x, double y, double heading, double power) {
        Position pos = new Position(unit, x, y, 0, 0);
        RobotTransform transform = new RobotTransform(pos, heading);
        this.transform = transform;
        this.power = power;
    }

    public MoveAndOrientAction(RobotTransform transform, double power) {
        this.transform = transform;
        this.power = power;
    }

    @Override
    public void init(RobotHardware hardware) {
//        this.overallDistanceToTarget = Localizer.distance(finalPosition, hardware.localizer.estimatePosition());

//        startingAngle = hardware.localizer.estimateOrientation().thirdAngle;

    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        Position currentPosition = hardware.localizer.getEstimatedPosition().position;
        Localizer.EstimatedOrientation currentEstimatedOrientation = hardware.localizer.getEstimatedOrientation();
        if (currentPosition == null || currentEstimatedOrientation == null) {
            return false;
        }

        Orientation currentOrientation = currentEstimatedOrientation.orientation;
        Position inches = currentPosition.toUnit(DistanceUnit.INCH);
        double currentHeading = currentOrientation.thirdAngle;
        double currentDistanceToTarget = Localizer.distance(currentPosition, transform.position, DistanceUnit.INCH);

        double angularDifferenceBetweenPositions = Localizer.atan2InDegrees(currentPosition, transform.position);
        double angularDifferenceToTargetHeading = Localizer.angularDifferenceInDegrees(currentHeading, transform.heading);
        double angDiffBetweenForwardAndTargetPos = Localizer.angularDifferenceInDegrees(currentHeading, angularDifferenceBetweenPositions);
        double distanceToTargetIn = Localizer.distance(currentPosition, transform.position, DISTANCE_TOLERANCE_UNIT);

        hardware.telemetry.addData("current pos", "%.1f %.1f %.1f", inches.x, inches.y, inches.z);
        hardware.telemetry.addData("current heading", currentHeading);
        hardware.telemetry.addData("dist to target", currentDistanceToTarget);
        hardware.telemetry.addData("ang between pos", angularDifferenceBetweenPositions);

        hardware.telemetry.addData("ang to target heading", angularDifferenceToTargetHeading);
        hardware.telemetry.addData("ang forward to target", angDiffBetweenForwardAndTargetPos);
        hardware.telemetry.addData("Distance to target", distanceToTargetIn);



        double  movePower;
        if (Localizer.distance(currentPosition, transform.position, DISTANCE_SCALE_DOWN_UNIT) <= DISTANCE_UNTIL_SCALE_DOWN) {
            movePower = 0.4;
        } else {
            movePower = power;
        }



        boolean withinDistanceTolerance = Localizer.distance(currentPosition, transform.position, DISTANCE_TOLERANCE_UNIT) <= DISTANCE_TOLERANCE;
        boolean withinHeadingTolerance = Math.abs(Localizer.angularDifferenceInDegrees(currentHeading, transform.heading)) <= HEADING_TOLERANCE;
        if (withinDistanceTolerance && withinHeadingTolerance) {
            hardware.omniDrive.stopDrive();
            return true;
        } else if (withinDistanceTolerance) {
            hardware.omniDrive.rotateRight(angularDifferenceToTargetHeading > 0 ? 0.275 : -0.275);
        } else {
            hardware.omniDrive.move(movePower, Math.toRadians(angDiffBetweenForwardAndTargetPos), angularDifferenceToTargetHeading/180);
        }
        return false;
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
