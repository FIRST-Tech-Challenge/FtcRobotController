package org.firstinspires.ftc.teamcode.autonomous.sequences;

import org.firstinspires.ftc.teamcode.action.EnableShooterAction;
import org.firstinspires.ftc.teamcode.action.SetMotorPowerAction;
import org.firstinspires.ftc.teamcode.action.WaitAction;
import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;

public class ShootActionSequence extends ActionSequence {

    public ShootActionSequence(int numOfRingsToShoot) {
        addAction(new EnableShooterAction(true));
        addAction(new WaitAction(1500));
        addAction(new SetMotorPowerAction("escalator" ,1));
        addAction(new WaitAction(1500));
        addAction(new WaitAction(1600 * (Math.max(1, numOfRingsToShoot)-1)));
        addAction(new SetMotorPowerAction("escalator" ,0));
        addAction(new EnableShooterAction(false));
    }

}
