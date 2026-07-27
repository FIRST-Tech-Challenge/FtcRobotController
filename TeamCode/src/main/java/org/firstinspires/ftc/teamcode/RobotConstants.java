package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Every number worth tuning, in one place.
 *
 * These are deliberately {@code public static} and NOT {@code final} so that FTC Dashboard's
 * {@code @Config} annotation can be added to this class later to tune them live from the pit,
 * without having to restructure anything.
 */
public class RobotConstants {

    // ---------------------------------------------------------------------------------------
    // Hardware configuration names - these must match the configuration on the Driver Station
    // ---------------------------------------------------------------------------------------
    public static final String INTAKE_NAME        = "intake";         // M1
    public static final String INDEX_NAME         = "index";          // M2
    public static final String SHOOTER_LEFT_NAME  = "shooter_left";   // M3
    public static final String SHOOTER_RIGHT_NAME = "shooter_right";  // M4

    // ---------------------------------------------------------------------------------------
    // Mechanism speeds - reverse directions use the negative of these same numbers
    // ---------------------------------------------------------------------------------------

    /** M1 forward. Negated for outtake. */
    public static double INTAKE_POWER = 0.8;

    /** M2 forward. Negated to run the index backwards. */
    public static double INDEX_POWER = 1.0;

    /** M3 and M4 for a close shot. Placeholder - tune this on the field. */
    public static double CLOSE_SHOOT_SPEED = 0.7;

    /** M3 and M4 for a far shot. Placeholder - tune this on the field. */
    public static double FAR_SHOOT_SPEED = 1.0;

    // ---------------------------------------------------------------------------------------
    // Motor directions
    // ---------------------------------------------------------------------------------------
    public static final DcMotor.Direction INTAKE_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction INDEX_DIRECTION  = DcMotor.Direction.FORWARD;

    /**
     * Both shooter wheels have to throw the ball the same way. Depending on how M3 and M4 are
     * mounted, one of them will probably need to be REVERSE - if the wheels fight each other on
     * the first test, flip this one.
     */
    public static final DcMotor.Direction SHOOTER_LEFT_DIRECTION  = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction SHOOTER_RIGHT_DIRECTION = DcMotor.Direction.FORWARD;

    // ---------------------------------------------------------------------------------------
    // Driving
    // ---------------------------------------------------------------------------------------

    /** Stick movement smaller than this counts as zero, so a worn stick doesn't creep. */
    public static double DEADBAND = 0.05;

    /** Turning is scaled down separately - full-power turns are hard to control. */
    public static double TURN_SCALE = 0.8;

    /**
     * Slow-speed multiplier. Currently not bound to any button, because Y, X and the D-pad are
     * being held for the Limelight controls. Binding it later is one line in CompTeleOp.
     */
    public static double PRECISION_SPEED = 0.35;

    /** How far a trigger must be pulled before it counts as pressed. */
    public static double TRIGGER_THRESHOLD = 0.5;

    // ---------------------------------------------------------------------------------------
    // Control Hub mounting - change these to match how the hub sits on the robot,
    // or field-centric drive will read the heading off the wrong axis.
    // ---------------------------------------------------------------------------------------
    public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    private RobotConstants() {
        // Constants holder - not meant to be instantiated.
    }
}
