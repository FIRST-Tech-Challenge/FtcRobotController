package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.*;

/**
 * TEST ONLY, never annotated or packaged: the real Red TeleOp handed off in front of the Red audience cell with the
 * chassis turned 45 deg from the line of sight, so driving and strafing both move the robot across the goal bearing.
 * Blue uses the point mirror.
 */
public class CrossingHandoffRedTeleOp extends RedMecanumTeleOp {
    static final double X_METERS = -12.75 * .0254, Y_METERS = -40 * .0254, HEADING_RAD = .75 * Math.PI;

    @Override public void runOpMode() {
        HandoffRedTeleOp.saveAutonomousEnd(getAlliance(), X_METERS, Y_METERS, HEADING_RAD);
        super.runOpMode();
    }
}
