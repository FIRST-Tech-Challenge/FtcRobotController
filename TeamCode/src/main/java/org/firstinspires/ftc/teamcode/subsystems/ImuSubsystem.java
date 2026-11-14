package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * IMU Subsystem
 * Owns the Control Hub's BUILT-IN gyroscope (the "IMU"). It only READS. It never moves a motor and
 * has no gamepad code. Another class (for now TeleOp - Alpha 14 and ImuAlpha) asks it which way the
 * robot is facing.
 *
 * WHY WE HAVE IT: the field-centric drive needs the robot's HEADING. It used to take it from the
 * Pinpoint, but the Pinpoint is a separate gadget on an I2C cable, and when it blinks (a cable
 * wiggles, the battery sags) the heading goes with it and field-centric had to give up. This
 * gyroscope is INSIDE the Control Hub, on the hub's own internal wiring, so it has no cable to
 * wiggle. The drive now asks THIS class for its heading, and the Pinpoint only helps the turret.
 *
 * WHAT IT ANSWERS: "Which way is the robot facing, compared to where it was at INIT?"
 *   getHeadingDegrees()  -> turning LEFT (counterclockwise) is POSITIVE, from -180 to 180.
 * That is the same sign as PinpointSubsystem, TurretSubsystem and Pedro Pathing, so the drive math
 * does not change. Zero is the way the robot faced when this class was created (at INIT).
 *
 * THE HUB'S ORIENTATION (the one thing you must get right): the IMU has no idea how the hub is
 * bolted to the robot, so we tell it with two directions: which way the REV LOGO on the hub faces,
 * and which way the USB PORTS face. If they are wrong, the heading is wrong (it can even run
 * backwards, or change when the robot tips). ImuAlpha checks them in a minute.
 * The two directions must be on DIFFERENT axes (logo UP with USB UP is physically impossible). If
 * they are not, the SDK refuses them. This class catches that, writes it in the log, and says so on
 * the screen, and the TeleOp drives ROBOT-CENTRIC. It never crashes the OpMode over it.
 *
 * It does NOT need the robot to be still at INIT (the hub calibrates itself when it powers on).
 * Resetting the heading to zero is instant.
 *
 * ABSENT DEVICE: if the robot's configuration has no device named IMU_NAME, this class does NOT
 * stop the OpMode. isAvailable() is false, the heading stays 0, and the telemetry says what to fix.
 * (The TeleOp then drives ROBOT-CENTRIC and says so on the screen.)
 *
 * The OpMode must call update() ONCE per loop(). Nothing calls it for us.
 *
 * Robot configuration (on the Driver Station):
 *   The Control Hub's built-in IMU is called "imu" ("IMU (BHI260AP)" on newer Control Hubs, or
 *   "IMU (BNO055)" on older ones). A new configuration normally has it already. If it is missing,
 *   open the Control Hub in the configuration, go to I2C Bus 0, and add the IMU there, named imu.
 */
public class ImuSubsystem {

    // ------------------------------------------------------------------
    // Hardware name. This MUST match the robot configuration exactly.
    // ------------------------------------------------------------------
    public static final String IMU_NAME = "imu";

    // ------------------------------------------------------------------
    // How the Control Hub is mounted on the robot.
    //   LOGO_FACING_DIRECTION: the way the REV logo (the printed word and cube on the lid) faces.
    //   USB_FACING_DIRECTION:  the way the hub's USB ports point.
    // The choices are UP, DOWN, FORWARD, BACKWARD, LEFT and RIGHT, seen from the robot's own point
    // of view (FORWARD is the way the intake faces, UP is toward the ceiling). The two must be on
    // different axes: for example UP with DOWN, or LEFT with RIGHT, is impossible.
    //
    // OUR ROBOT (the coach, 2026-11-12): the hub stands on its end with its USB ports pointing at
    // the CEILING, and the REV logo faces the MIDDLE of the robot. It is bolted to the LEFT wall of
    // the robot (the coach: "left wall from a bird's eye view", which we read as the robot's own
    // left, with the intake at the top of the picture).
    //   USB_FACING_DIRECTION = UP.
    //   LOGO_FACING_DIRECTION = RIGHT, because "toward the middle" from the LEFT wall is to the RIGHT.
    // If the hub is ever moved, the logo direction changes with the wall it is on, because
    // "toward the middle" is a different way from each wall:
    //       hub on the LEFT side of the robot   -> the logo faces RIGHT   (ours)
    //       hub on the RIGHT side of the robot  -> the logo faces LEFT
    //       hub at the BACK of the robot        -> the logo faces FORWARD
    //       hub at the FRONT of the robot       -> the logo faces BACKWARD
    // TODO (issue #88): Not yet run on the robot. ImuAlpha will tell you if it is wrong: with the robot
    // flat, pitch and roll are about 0, and turning the robot LEFT makes the heading go UP. If
    // "left wall" meant the VIEWER's left and not the robot's, the logo would be LEFT instead.
    // ------------------------------------------------------------------
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    // The heading must go UP when the robot turns LEFT (counterclockwise). If ImuAlpha shows it
    // going DOWN while the pitch and roll are right, set this to true. (The SDK documents the IMU's
    // yaw as counterclockwise positive, so false is expected, but test it once.)
    private static final boolean INVERT_HEADING = false;

    // ------------------------------------------------------------------
    // The hardware and the newest answers
    // ------------------------------------------------------------------
    // null when the robot's configuration has no device with IMU_NAME.
    private final IMU imu;

    // true only if initialize() worked. Without it the readings mean nothing.
    private boolean initialized = false;

    // If the hub orientation was refused, what the SDK said. Empty when all is well.
    private String startupProblem = "";

    // The newest answers, saved by update(). They are zero when there is no IMU.
    private double headingDegrees = 0.0;
    private double pitchDegrees = 0.0;
    private double rollDegrees = 0.0;

    // ==================================================================
    // SETUP
    // ==================================================================

    /*
     * Gets the IMU, tells it how the hub is mounted, and makes the way the robot faces NOW the
     * heading zero.
     */
    public ImuSubsystem(HardwareMap hardwareMap) {
        // tryGet gives back null instead of stopping the OpMode when the name is not in the
        // robot configuration.
        imu = hardwareMap.tryGet(IMU.class, IMU_NAME);
        if (imu == null) {
            return;
        }

        // The SDK refuses a logo direction and a USB direction that cannot both be true (for example
        // UP and UP). It does that by throwing an error. We catch it so that a wrong setting here
        // turns field-centric off with a clear message, instead of crashing the whole OpMode at INIT.
        try {
            RevHubOrientationOnRobot hubOrientation =
                    new RevHubOrientationOnRobot(LOGO_FACING_DIRECTION, USB_FACING_DIRECTION);
            initialized = imu.initialize(new IMU.Parameters(hubOrientation));
        } catch (IllegalArgumentException problem) {
            startupProblem = problem.getMessage();
            BiobuzzLog.warning("IMU NOT STARTED: the hub orientation (logo %s, USB %s) was refused: %s",
                    LOGO_FACING_DIRECTION, USB_FACING_DIRECTION, startupProblem);
        }
        if (initialized) {
            imu.resetYaw();
        }
    }

    // ==================================================================
    // ACTIONS
    // ==================================================================

    /*
     * Call this ONCE per loop(). It reads the IMU and saves the newest heading, pitch and roll.
     */
    public void update() {
        if (!isAvailable()) {
            return;
        }
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        headingDegrees = angles.getYaw(AngleUnit.DEGREES);
        pitchDegrees = angles.getPitch(AngleUnit.DEGREES);
        rollDegrees = angles.getRoll(AngleUnit.DEGREES);

        if (INVERT_HEADING) {
            headingDegrees = -headingDegrees;
        }
    }

    /*
     * Makes the way the robot faces RIGHT NOW the new zero. Instant and safe at any time, even
     * while driving.
     */
    public void resetHeading() {
        if (isAvailable()) {
            imu.resetYaw();
        }
        headingDegrees = 0.0;
    }

    // ==================================================================
    // QUESTIONS
    // ==================================================================

    /*
     * Which way the robot faces, in degrees, from -180 to 180. Turning LEFT (counterclockwise) is
     * POSITIVE. Zero is the way the robot faced at INIT (or the last resetHeading()).
     */
    public double getHeadingDegrees() {
        return headingDegrees;
    }

    // How far the robot is tipped nose-up or nose-down, in degrees. For checking the hub's
    // orientation in ImuAlpha. About 0 when the robot sits flat.
    public double getPitchDegrees() {
        return pitchDegrees;
    }

    // How far the robot is tipped onto one side, in degrees. About 0 when the robot sits flat.
    public double getRollDegrees() {
        return rollDegrees;
    }

    /*
     * True when the robot's configuration has a device named IMU_NAME AND it started up. Check this
     * before trusting the heading.
     */
    public boolean isAvailable() {
        return imu != null && initialized;
    }

    /*
     * Shows the heading and what to fix if the IMU is missing. Call it after update().
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- GYRO (Control Hub IMU) ---");
        if (imu == null) {
            telemetry.addData("IMU", "NOT FOUND. Add the Control Hub's IMU, named \"%s\", to I2C Bus 0 in the robot config", IMU_NAME);
            return;
        }
        if (!initialized) {
            if (startupProblem.isEmpty()) {
                telemetry.addData("IMU", "found, but it did NOT start up. Check the hub orientation settings in ImuSubsystem");
            } else {
                telemetry.addData("IMU", "NOT STARTED: the hub orientation was refused (%s). Check LOGO_FACING_DIRECTION and USB_FACING_DIRECTION in ImuSubsystem", startupProblem);
            }
            return;
        }
        telemetry.addData("IMU heading", "%.1f deg (+ left)", headingDegrees);
        telemetry.addData("IMU pitch, roll", "%.1f, %.1f deg (both about 0 when the robot is flat)", pitchDegrees, rollDegrees);
    }
}
