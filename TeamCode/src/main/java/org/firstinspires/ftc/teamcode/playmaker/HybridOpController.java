package org.firstinspires.ftc.teamcode.playmaker;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class HybridOpController {

    public RobotHardware hardware;
    private HybridOp hybridOp;
    private ActionExecutor actionExecutor;
    private boolean isAutonomous = false;
    private boolean overrideManualControl = true;


    public HybridOpController(RobotHardware hardware, HybridOp hybridOp) {
        this.hardware = hardware;
        this.hybridOp = hybridOp;
    }

    /**
     * Returns whether the HybridOp is executing autonomously
     * @return
     */
    public boolean isAutonomous() {
        return isAutonomous;
    }

    /**
     * Executes a provided action sequence with or without manual control
     * @param sequence The action sequence to be executed
     * @param overrideManualControl Whether to allow manual control while the action sequence is executing
     */
    public void executeActionSequence(ActionSequence sequence, boolean overrideManualControl) {
        this.actionExecutor = new ActionExecutor(hardware, sequence);
        this.actionExecutor.init();
        this.isAutonomous = true;
        this.overrideManualControl = overrideManualControl;
        if (overrideManualControl) {
            hardware.omniDrive.stopDrive();
        }
    }

    /**
     * Immediately stop autonomous execution
     */
    public void stopAutonomous() {
        this.isAutonomous = false;
        this.actionExecutor = null;
    }

    /**
     * Function that is called every loop regardless of in autonomous/manual control
     */
    public void loop() {
        hardware.telemetry.addData("Autonomous", isAutonomous);
        hardware.hardware_loop();
        hybridOp.hybrid_loop();
        if (isAutonomous && overrideManualControl) {
            isAutonomous = !this.actionExecutor.loop();
            hybridOp.autonomous_loop();
        } else if (isAutonomous){
            isAutonomous = !this.actionExecutor.loop();
            hybridOp.autonomous_loop();
            hybridOp.teleop_loop();
        } else {
            hybridOp.teleop_loop();
        }
    }

}
