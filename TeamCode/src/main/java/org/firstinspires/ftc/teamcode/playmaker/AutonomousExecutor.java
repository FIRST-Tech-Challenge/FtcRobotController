package org.firstinspires.ftc.teamcode.playmaker;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class AutonomousExecutor extends OpMode {

    public RobotHardware hardware;
    private ActionSequence actionSequence;
    private ActionExecutor actionExecutor;

    public abstract RobotHardware getHardware();

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
