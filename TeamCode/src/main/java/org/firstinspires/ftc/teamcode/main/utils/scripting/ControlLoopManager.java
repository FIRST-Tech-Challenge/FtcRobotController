package org.firstinspires.ftc.teamcode.main.utils.scripting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * A ControlLoopManager controls the control loop of an OpMode. It's meant to replace opModeIsActive(). It's an implementation of Colin's back button idea but with a few less characters for developer speed.
 */
public class ControlLoopManager {

    private final LinearOpMode OP_MODE;
    private boolean shouldStop;

    public ControlLoopManager(LinearOpMode opMode) {
        OP_MODE = opMode;
    }

    public boolean shouldContinue() {
        return !shouldStop && !OP_MODE.isStopRequested() && !OP_MODE.gamepad1.back && !OP_MODE.gamepad2.back && OP_MODE.opModeIsActive();
    }

    public void requestStop() {
        shouldStop = true;
    }

}
