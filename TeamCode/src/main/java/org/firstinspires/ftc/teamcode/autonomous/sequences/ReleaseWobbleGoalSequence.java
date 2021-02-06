package org.firstinspires.ftc.teamcode.autonomous.sequences;

import org.firstinspires.ftc.teamcode.action.ExtendWobbleGoalAction;
import org.firstinspires.ftc.teamcode.action.MoveAction;
import org.firstinspires.ftc.teamcode.action.SetServoAction;
import org.firstinspires.ftc.teamcode.action.WaitAction;
import org.firstinspires.ftc.teamcode.action.WaitForWobbleGoalInPositionAction;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.util.OmniDrive;

public class ReleaseWobbleGoalSequence extends ActionSequence {

    public ReleaseWobbleGoalSequence() {
        addAction(new ExtendWobbleGoalAction(true));
        addAction(new WaitForWobbleGoalInPositionAction());
        addAction(new SetServoAction("wobbleServo", 0.8));
        addAction(new WaitAction(500));
        addAction(new MoveAction(OmniDrive.Direction.LEFT, 3, 0.65f));
        addAction(new ExtendWobbleGoalAction(false));
        addAction(new WaitForWobbleGoalInPositionAction());
    }
}
