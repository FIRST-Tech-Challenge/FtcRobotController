package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.*;

/** TEST ONLY, never annotated or packaged: the real Blue TeleOp handed off at the point mirror of the Red crossing pose. */
public class CrossingHandoffBlueTeleOp extends BlueMecanumTeleOp {
    @Override public void runOpMode() {
        HandoffRedTeleOp.saveAutonomousEnd(getAlliance(), -CrossingHandoffRedTeleOp.X_METERS,
                -CrossingHandoffRedTeleOp.Y_METERS, CrossingHandoffRedTeleOp.HEADING_RAD + Math.PI);
        super.runOpMode();
    }
}
