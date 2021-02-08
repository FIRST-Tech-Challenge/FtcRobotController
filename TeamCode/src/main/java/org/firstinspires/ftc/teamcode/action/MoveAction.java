package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;
import org.firstinspires.ftc.teamcode.util.EncoderDrive;
import org.firstinspires.ftc.teamcode.util.OmniDrive;

/**
 * Created by djfigs1 on 9/30/17.
 */

public class MoveAction implements Action {

    OmniDrive.Direction direction;
    double distance;
    float speed;
    Position initialPosition;
    Orientation initialOrientation;
    LocalizerMoveAction localizerMoveAction;

    private EncoderDrive driver;

    /**
     * This action allows you to move the robot in any of the eight directions for a set distance.
     *
     * @param direction Direction for the robot to move.
     * @param distance The distance (in centimeters) for the robot to move..
     * @param speed How much power is given to each motor.
     */

    public MoveAction(OmniDrive.Direction direction, double distance, float speed) {
        this.direction = direction;
        this.distance = distance;
        this.speed = speed;
    }

    public void init(RobotHardware hardware) {
        Localizer.EstimatedPosition position = hardware.localizer.estimatePosition();
        Localizer.EstimatedOrientation orientation = hardware.localizer.estimateOrientation();
        if (position != null && orientation != null) {
            initialPosition = position.position.toUnit(DistanceUnit.INCH);
            initialOrientation = orientation.orientation;
            double rawX = 0;
            double rawY = 0;

            switch (direction) {
                case FORWARD:
                    rawY = distance;
                    break;
                case LEFT:
                    rawX = -distance;
                    break;
                case RIGHT:
                    rawX = distance;
                    break;
                case BACKWARD:
                    rawY = -distance;
                    break;
                default:
                    return;
            }

            double robotHeading = Math.toRadians(initialOrientation.thirdAngle);
            double newX = -((rawX*Math.cos(robotHeading)) - (rawY*Math.sin(robotHeading)));
            double newY = -((rawX*Math.sin(robotHeading)) + (rawY*Math.cos(robotHeading)));
            Localizer.RobotTransform targetTransform = new Localizer.RobotTransform(DistanceUnit.INCH, initialPosition.x + newX, initialPosition.y + newY, orientation.orientation.thirdAngle);
            localizerMoveAction = new LocalizerMoveAction(targetTransform, UltimateGoalHardware.defaultLocalizerMoveParameters);
            localizerMoveAction.init(hardware);
        }

    }

    public boolean doAction(RobotHardware hardware) {
        if (localizerMoveAction != null) {
            return localizerMoveAction.doAction(hardware);
        }
        return true;
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
