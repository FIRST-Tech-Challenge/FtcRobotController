package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.*;

/**
 * TEST ONLY, never annotated or packaged: the real Red TeleOp started right after a Red autonomous that ended at
 * POSE. The save runs inside the simulated class loader and clock, exactly where autonomous would have left it.
 */
public class HandoffRedTeleOp extends RedMecanumTeleOp {
    /** Autonomous end pose (meters, CCW radians); the scenario places the plant here. Blue uses the point mirror. */
    static final double X_METERS = -.9, Y_METERS = -.6, HEADING_RAD = .4;

    static void saveAutonomousEnd(TurretSubsystem.Alliance alliance, double x, double y, double heading) {
        AutoPoseHandoff.save(alliance, new MotionSample(System.nanoTime() * 1e-9, x, y, heading,
                0, 0, 0, 0, .02, true, true, "synthetic autonomous end"));
    }
    @Override public void runOpMode() {
        saveAutonomousEnd(getAlliance(), X_METERS, Y_METERS, HEADING_RAD);
        super.runOpMode();
    }
}
