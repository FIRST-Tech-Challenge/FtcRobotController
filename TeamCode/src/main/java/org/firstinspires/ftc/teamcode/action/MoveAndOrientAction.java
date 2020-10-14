package org.firstinspires.ftc.teamcode.action;

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
    private double heading;
    private double power;

    public MoveAndOrientAction(Position finalPosition, double heading, double power) {
        this.finalPosition = finalPosition;
        this.heading = heading;
        this.power = power;
    }

    @Override
    public void init(RobotHardware hardware) {
        this.overallDistanceToTarget = Localizer.distance(finalPosition, hardware.localizer.estimatePosition());


        startingAngle = hardware.localizer.estimateOrientation().thirdAngle;

    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        Position currentPosition = hardware.localizer.estimatePosition();
        double currentHeading = hardware.localizer.estimateOrientation().thirdAngle;
        double currentDistanceToTarget = Localizer.distance(currentPosition, finalPosition);

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
}
