package org.firstinspires.ftc.teamcode.autonomous.sequences;

import org.firstinspires.ftc.teamcode.action.MoveAction;
import org.firstinspires.ftc.teamcode.action.SetMotorPowerAction;
import org.firstinspires.ftc.teamcode.action.SetServoAction;
import org.firstinspires.ftc.teamcode.action.WaitAction;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.util.OmniDrive;

public class ReleaseWobbleGoalSequence extends ActionSequence {

    public ReleaseWobbleGoalSequence() {
        addAction(new SetServoAction("wobbleServo", 0));
        addAction(new SetMotorPowerAction("wobble", -0.75));
        addAction(new WaitAction(750));
        addAction(new SetServoAction("wobbleServo", 0.8));
        addAction(new SetMotorPowerAction("wobble", 0));
        addAction(new WaitAction(150));
        addAction(new MoveAction(OmniDrive.Direction.LEFT, 8, 0.45f));
        addAction(new SetServoAction("wobbleServo", 0));
//        addAction(new WaitAction(150));
//        addAction(new SetMotorPowerAction("wobble", 0.75));
//        addAction(new WaitAction(500));
//        addAction(new SetMotorPowerAction("wobble", 0));
    }
}
