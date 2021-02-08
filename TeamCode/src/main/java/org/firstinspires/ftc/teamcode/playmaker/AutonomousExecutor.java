package org.firstinspires.ftc.teamcode.playmaker;

/**
 * A template class used to create an autonomous opmode for executing action sequences.
 * This utilizes the ActionExecutor and just ties it to the OpMode functions.
 */
public class AutonomousExecutor {

    private ActionSequence actionSequence;
    private ActionExecutor actionExecutor;
    private Autonomous autonomous;
    private RobotHardware hardware;

    public AutonomousExecutor(Autonomous autonomous, RobotHardware hardware) {
        this.autonomous = autonomous;
        this.hardware = hardware;
        actionSequence = autonomous.getActionSequence();
        actionExecutor = new ActionExecutor(hardware, actionSequence);
    }
    public boolean loop() {
        return actionExecutor.loop();
    }

    public void stop() {
        hardware.omniDrive.stopDrive();
        actionExecutor.stop();
    }
}
