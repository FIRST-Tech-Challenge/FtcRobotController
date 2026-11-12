package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.*;

/** TEST ONLY, never annotated or packaged: the Blue counterpart, in front of the blue audience cell, error mirrored in X. */
public class MisplacedHandoffBlueTeleOp extends BlueMecanumTeleOp {
    @Override public void runOpMode() {
        HandoffRedTeleOp.saveAutonomousEnd(getAlliance(),
                -MisplacedHandoffRedTeleOp.TRUE_X_METERS - MisplacedHandoffRedTeleOp.ERROR_X_METERS,
                MisplacedHandoffRedTeleOp.TRUE_Y_METERS,
                MisplacedHandoffRedTeleOp.TRUE_HEADING_RAD - MisplacedHandoffRedTeleOp.ERROR_HEADING_RAD);
        super.runOpMode();
    }
}
