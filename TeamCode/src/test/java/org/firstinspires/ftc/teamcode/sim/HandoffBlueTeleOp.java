package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.*;

/** TEST ONLY, never annotated or packaged: the real Blue TeleOp after a Blue autonomous ending at the mirrored pose. */
public class HandoffBlueTeleOp extends BlueMecanumTeleOp {
    @Override public void runOpMode() {
        HandoffRedTeleOp.saveAutonomousEnd(getAlliance(), -HandoffRedTeleOp.X_METERS, -HandoffRedTeleOp.Y_METERS,
                HandoffRedTeleOp.HEADING_RAD + Math.PI);
        super.runOpMode();
    }
}
