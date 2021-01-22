package org.firstinspires.ftc.teamcode.autonomous.sequences;

import org.firstinspires.ftc.teamcode.action.SetMotorPowerAction;
import org.firstinspires.ftc.teamcode.action.SetServoAction;
import org.firstinspires.ftc.teamcode.action.WaitAction;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;

public class ReleaseWobbleGoalSequence extends ActionSequence {

    public ReleaseWobbleGoalSequence() {
        addAction(new SetServoAction("wobbleServo", 0));
        addAction(new SetMotorPowerAction("wobble", -0.65));
        addAction(new WaitAction(500));
        addAction(new SetMotorPowerAction("wobble", 0));
        addAction(new SetServoAction("wobbleServo", 0.4));
        addAction(new WaitAction(300));
    }
}
