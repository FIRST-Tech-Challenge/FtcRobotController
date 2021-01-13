package org.firstinspires.ftc.teamcode.autonomous.sequences;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.action.MoveAndOrientAction;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.Localizer.RobotTransform;

public class MovementActionSequence extends ActionSequence {

    public MovementActionSequence() {
        addAction(new MoveAndOrientAction(new RobotTransform(
                new Position(DistanceUnit.INCH, -24, 0, 0, 0),
                -90),
                0.7));
    }
}
