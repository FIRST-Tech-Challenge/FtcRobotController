package org.firstinspires.ftc.teamcode.autonomous.sequences;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.action.ExecuteSequenceAction;
import org.firstinspires.ftc.teamcode.action.LocalizerMoveAction;
import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;
import org.firstinspires.ftc.teamcode.playmaker.Localizer.RobotTransform;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class NavigateAndShootPowerTargetsSequence extends ActionSequence {

    static final double DISTANCE_BETWEEN_POWERSHOT_TARGETS_INCHES = 7.5;
    static final double HALF_FIELD_LENGTH_INCHES = 72;
    public static final RobotTransform SHOOTING_POSITION_CENTER_TARGET_RED_TEAM = new Localizer.RobotTransform(DistanceUnit.INCH, -3, -36, 90);

    public NavigateAndShootPowerTargetsSequence(RobotHardware.Team team) {
        RobotTransform shootingPosition;
        if (team == RobotHardware.Team.RED) {
            shootingPosition = SHOOTING_POSITION_CENTER_TARGET_RED_TEAM.copy();
        } else {
            shootingPosition = Localizer.mirrorTransformOverTeamLine(SHOOTING_POSITION_CENTER_TARGET_RED_TEAM);
        }
        shootingPosition.heading += UltimateGoalHardware.SHOOTER_HEADING_OFFSET;

        // Calculate heading differences between targets. Assumed that we're lined up positionally with the center target
        double distanceToPowershotTarget = HALF_FIELD_LENGTH_INCHES - shootingPosition.position.x;
        double headingDifferenceBetweenTargets = Math.toDegrees(Math.atan(DISTANCE_BETWEEN_POWERSHOT_TARGETS_INCHES / distanceToPowershotTarget));

        RobotTransform leftTarget;
        RobotTransform centerTarget;
        RobotTransform rightTarget;

        leftTarget = shootingPosition.copy();
        leftTarget.heading = Localizer.headingWrapInDegrees(leftTarget.heading + headingDifferenceBetweenTargets + UltimateGoalHardware.SHOOTER_HEADING_OFFSET);
        centerTarget = shootingPosition.copy();
        centerTarget.heading = Localizer.headingWrapInDegrees(centerTarget.heading + UltimateGoalHardware.SHOOTER_HEADING_OFFSET);
        rightTarget = shootingPosition.copy();
        rightTarget.heading = Localizer.headingWrapInDegrees(rightTarget.heading - headingDifferenceBetweenTargets + UltimateGoalHardware.SHOOTER_HEADING_OFFSET);

        addAction(new LocalizerMoveAction(leftTarget, UltimateGoalHardware.defaultLocalizerMoveParameters));
        addAction(new ExecuteSequenceAction(new ShootActionSequence(1)));
        addAction(new LocalizerMoveAction(centerTarget, UltimateGoalHardware.defaultLocalizerMoveParameters));
        addAction(new ExecuteSequenceAction(new ShootActionSequence(1)));
        addAction(new LocalizerMoveAction(rightTarget, UltimateGoalHardware.defaultLocalizerMoveParameters));
        addAction(new ExecuteSequenceAction(new ShootActionSequence(1)));
    }



}
