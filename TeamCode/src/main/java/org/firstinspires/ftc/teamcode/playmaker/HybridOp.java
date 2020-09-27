package org.firstinspires.ftc.teamcode.playmaker;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class HybridOp extends OpMode {

    public GamepadController gamepadController;
    public RobotHardware hardware;
    private ActionExecutor actionExecutor;
    private boolean isAutonomous = false;
    private boolean haltManual = true;

    public abstract RobotHardware getHardware();

    @Override
    public void init() {
        hardware = getHardware();
        hardware.initializeHardware();
        hardware.initializeTeleOp();
        gamepadController = new GamepadController(gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        telemetry.addData("Autonomous", isAutonomous);
        hardware.hardware_loop();
        this.hybrid_loop();
        gamepadController.controllerLoop(isAutonomous);

        if (isAutonomous && haltManual) {
            this.autonomous_loop();
        } else if (isAutonomous){
            this.autonomous_loop();
            this.teleop_loop();
        } else {
            this.teleop_loop();
        }
    }

    public boolean isAutonomous() {
        return isAutonomous;
    }

    public void executeActionSequence(ActionSequence sequence, boolean haltManual) {
        this.actionExecutor = new ActionExecutor(hardware, sequence);
        this.actionExecutor.init();
        this.isAutonomous = true;
        this.haltManual = haltManual;
        if (haltManual) {
            hardware.omniDrive.stopDrive();
        }
    }

    public void stopAutonomous() {
        this.isAutonomous = false;
        this.actionExecutor = null;
    }

    public void hybrid_loop() {

    }

    public void autonomous_loop() {
        // The Action Executor will return true once it's done. Therefore, when it eventually
        // is true, it'll set the isAutonomous variable to false, which will exit out of
        // autonomous.
        isAutonomous = !this.actionExecutor.loop();

    }

    public void teleop_loop() { }


}
