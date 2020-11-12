package org.firstinspires.ftc.teamcode.playmaker;

public class HybridOpExecutor {

    public RobotHardware hardware;
    private HybridOp hybridOp;
    private ActionExecutor actionExecutor;
    private boolean inAutonomousMode = false;
    private boolean overrideManualControl = true;


    public HybridOpExecutor(HybridOp hybridOp, RobotHardware hardware) {
        this.hardware = hardware;
        this.hybridOp = hybridOp;
    }

    /**
     * Returns whether the HybridOp is executing autonomously
     * @return
     */
    public boolean isInAutonomousMode() {
        return inAutonomousMode;
    }

    /**
     * Executes a provided action sequence with or without manual control
     * @param sequence The action sequence to be executed
     * @param overrideManualControl Whether to allow manual control while the action sequence is executing
     * @param stopIfAlreadyRunning If the same action sequence is already being executed, stop execution and switch back to manual control. Otherwise, restart execution.
     *
     */
    public void executeActionSequence(ActionSequence sequence, boolean overrideManualControl, boolean stopIfAlreadyRunning) {
        if (stopIfAlreadyRunning && this.inAutonomousMode && actionExecutor.actionSequence.getClass() == sequence.getClass()) {
            this.stopAutonomous();
        } else {
            this.actionExecutor = new ActionExecutor(hardware, sequence);
            this.actionExecutor.init();
            this.inAutonomousMode = true;
            this.overrideManualControl = overrideManualControl;
            if (overrideManualControl && hardware.omniDrive != null) {
                hardware.omniDrive.stopDrive();
            }
        }
    }

    /**
     * Immediately stop autonomous execution
     */
    public void stopAutonomous() {
        this.inAutonomousMode = false;
        this.actionExecutor = null;
    }

    /**
     * Function that is called every loop regardless of in autonomous/manual control
     */
    public void loop() {
        hardware.telemetry.addData("HybridOp Autonomous Mode", inAutonomousMode);

        if (inAutonomousMode && overrideManualControl) {
            inAutonomousMode = !this.actionExecutor.loop();
            hybridOp.autonomous_loop();
        } else if (inAutonomousMode){
            inAutonomousMode = !this.actionExecutor.loop();
            hybridOp.autonomous_loop();
            hybridOp.teleop_loop();
        } else {
            hybridOp.teleop_loop();
        }
    }

}
