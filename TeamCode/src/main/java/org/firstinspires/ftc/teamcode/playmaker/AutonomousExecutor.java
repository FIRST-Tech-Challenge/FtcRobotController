package org.firstinspires.ftc.teamcode.playmaker;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * A template class used to create an autonomous opmode for executing action sequences.
 * This utilizes the ActionExecutor and just ties it to the OpMode functions.
 */
public class AutonomousExecutor {

    private ActionSequence actionSequence;
    private ActionExecutor actionExecutor;
    private Autonomous autonomous;

    public AutonomousExecutor(Autonomous autonomous) {
        this.autonomous = autonomous;
    }

    public void init() {
        actionSequence = autonomous.getActionSequence();
        actionExecutor = new ActionExecutor(autonomous.getHardware(), actionSequence);
    }

    public boolean loop() {
        return actionExecutor.loop();
    }

    public void stop() {
        autonomous.getHardware().omniDrive.stopDrive();
        actionExecutor.stop();
    }
}
