package org.firstinspires.ftc.teamcode.autonomous.sequences;

import org.firstinspires.ftc.teamcode.action.SetMotorPowerAction;
import org.firstinspires.ftc.teamcode.action.WaitAction;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;

public class ShootActionSequence extends ActionSequence {

    public ShootActionSequence() {
        addAction(new SetMotorPowerAction("shooterLeft", -0.5));
        addAction(new SetMotorPowerAction("shooterRight", 0.5));
        addAction(new WaitAction(1500));
        addAction(new SetMotorPowerAction("escalator" ,1));
        addAction(new WaitAction(1000));
        addAction(new SetMotorPowerAction("escalator" ,0));
        addAction(new SetMotorPowerAction("shooterLeft", 0));
        addAction(new SetMotorPowerAction("shooterRight", 0));

    }

}
