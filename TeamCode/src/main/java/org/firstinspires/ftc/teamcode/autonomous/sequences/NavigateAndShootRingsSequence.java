package org.firstinspires.ftc.teamcode.autonomous.sequences;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.action.ExecuteSequenceAction;
import org.firstinspires.ftc.teamcode.action.LocalizerMoveAction;
import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;
import org.firstinspires.ftc.teamcode.playmaker.Localizer.RobotTransform;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class NavigateAndShootRingsSequence extends ActionSequence {

    public static final RobotTransform SHOOTING_POSITION_RED_TEAM = new RobotTransform(DistanceUnit.INCH, -3, -36, 90);

    public NavigateAndShootRingsSequence(RobotHardware.Team team) {
        RobotTransform shootingPosition;
        if (team == RobotHardware.Team.RED) {
            shootingPosition = SHOOTING_POSITION_RED_TEAM.copy();
        } else {
            shootingPosition = Localizer.mirrorTransformOverTeamLine(SHOOTING_POSITION_RED_TEAM);
        }
        shootingPosition.heading += UltimateGoalHardware.SHOOTER_HEADING_OFFSET;

        addAction(new LocalizerMoveAction(shootingPosition, UltimateGoalHardware.defaultLocalizerMoveParameters));
        addAction(new ExecuteSequenceAction(new ShootActionSequence(3)));
    }


}
