package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Blue alliance fallback autonomous (no Pedro): see {@link FallbackAutonomous}. */
@Autonomous(name = "Auto: Blue Fallback (Shoot + Leave)", group = "Competition",
        preselectTeleOp = "Mecanum TeleOp - Blue")
public class BlueFallbackAutonomous extends FallbackAutonomous {
    // STARTING VALUES, UNMEASURED: set from this alliance's actual start spot on the practice field.
    /** Turret heading aiming at the goal from the start spot: the "Turret | HOMED | heading" value on the
     *  Driver Hub (front=0, right/home=90, back=180, left=270 deg). 90 leaves the turret at home. */
    public static final double PRELOAD_TURRET_HEADING_DEG = 90.0;
    /** Leave target from the start pose (inches): + forward / + left. */
    public static final double LEAVE_FORWARD_INCHES = 24.0;
    public static final double LEAVE_LEFT_INCHES = 0.0;

    @Override protected TurretSubsystem.Alliance getAlliance() { return TurretSubsystem.Alliance.BLUE; }
    @Override protected double preloadTurretHeadingDegrees() { return PRELOAD_TURRET_HEADING_DEG; }
    @Override protected double leaveForwardInches() { return LEAVE_FORWARD_INCHES; }
    @Override protected double leaveLeftInches() { return LEAVE_LEFT_INCHES; }
}
