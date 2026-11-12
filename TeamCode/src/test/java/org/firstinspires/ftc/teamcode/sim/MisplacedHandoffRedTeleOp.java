package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.*;

/**
 * TEST ONLY, never annotated or packaged: the real Red TeleOp after an autonomous handoff that is wrong by
 * ERROR (the robot was moved, or pods slipped), while the camera can see the raised audience-cell tags.
 */
public class MisplacedHandoffRedTeleOp extends RedMecanumTeleOp {
    /** True plant pose (meters, CCW radians): in front of the alliance's audience cell, as in the camera scenarios. */
    static final double TRUE_X_METERS = -12.75 * .0254, TRUE_Y_METERS = BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES * .0254,
            TRUE_HEADING_RAD = Math.PI;
    /** Handoff error: beyond the 0.5 m / 8 deg drift limits that the camera used to ignore. */
    static final double ERROR_X_METERS = .6, ERROR_HEADING_RAD = Math.toRadians(12);

    @Override public void runOpMode() {
        HandoffRedTeleOp.saveAutonomousEnd(getAlliance(), TRUE_X_METERS + ERROR_X_METERS, TRUE_Y_METERS,
                TRUE_HEADING_RAD + ERROR_HEADING_RAD);
        super.runOpMode();
    }
}
