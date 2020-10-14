package org.firstinspires.ftc.teamcode.playmaker;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * A template class used to create an autonomous opmode for executing action sequences.
 * This utilizes the ActionExecutor and just ties it to the OpMode functions.
 */
public abstract class AutonomousExecutor extends OpMode {

    public RobotHardware hardware;
    private ActionSequence actionSequence;
    private ActionExecutor actionExecutor;

    /**
     * Gets the hardware to be used during autonomous.
     * @return
     */

    public abstract RobotHardware getHardware();

    /**
     * Gets the action sequence to be executed.
     * All subclasses will put their desired action sequence here.
     * @return Action sequence to be executed
     */
    public abstract ActionSequence getActionSequence();

    @Override
    public void init() {
        hardware = getHardware();
        hardware.initializeHardware();
        hardware.initializeAutonomous();
        actionSequence = getActionSequence();
        actionExecutor = new ActionExecutor(hardware, actionSequence);
    }

    @Override
    public void loop() {
        boolean done = actionExecutor.loop();
        if (done) requestOpModeStop();
    }

    @Override
    public void stop() {
        hardware.omniDrive.stopDrive();
        actionExecutor.stop();
    }
}
