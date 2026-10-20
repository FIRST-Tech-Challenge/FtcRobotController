package org.firstinspires.ftc.teamcode.objects;

/**
 * The math that turns "drive, strafe, rotate" into four wheel powers.
 *
 * <p>No hardware here, on purpose. This class only does arithmetic, so it can be tested on
 * a laptop without a robot.
 *
 * <p>Sign convention (kept exactly from the Decode robot so it drives the same):
 * {@code drive} is the raw left stick Y (pushing forward is negative), {@code strafe} and
 * {@code rotate} are the negated stick X values, and the left motors run FORWARD while the
 * right motors run REVERSE. Every sign is the opposite of the FTC sample, and they cancel
 * out.
 *
 * <p>Wheel power formulas, before scaling:
 * <pre>
 *   front left  = drive + strafe + rotate
 *   back left   = drive - strafe + rotate
 *   front right = drive - strafe - rotate
 *   back right  = drive + strafe - rotate
 * </pre>
 */
public final class MecanumMath {

    /** The four wheel powers, one per motor. */
    public static final class WheelPowers {
        public double frontLeft;
        public double frontRight;
        public double backLeft;
        public double backRight;
    }

    private MecanumMath() {
    }

    /**
     * Robot-centric: "forward" means the way the robot is facing.
     */
    public static WheelPowers RobotCentric(double drive, double strafe, double rotate) {
        // If the inputs add up to more than 1, shrink every wheel by the same amount
        // so no wheel is asked for more than full power.
        double total = Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate);
        double divideBy = Math.max(total, 1);

        WheelPowers powers = new WheelPowers();
        powers.frontLeft = (drive + strafe + rotate) / divideBy;
        powers.backLeft = (drive - strafe + rotate) / divideBy;
        powers.frontRight = (drive - strafe - rotate) / divideBy;
        powers.backRight = (drive + strafe - rotate) / divideBy;
        return powers;
    }

    /**
     * Field-centric: "forward" means away from the driver, whichever way the robot faces.
     *
     * <p>Known issue, kept from Decode on purpose: {@code divideBy} uses the stick values
     * before they are turned, not after. At some headings a wheel can be asked for up to
     * about 1.41, and the motor controller cuts it to 1. See docs/decisions/0002-drive-port.md.
     *
     * @param yawDegrees how far the robot has turned from the driver's forward, counter-clockwise
     */
    public static WheelPowers FieldCentric(double drive, double strafe, double rotate,
                                           double yawDegrees) {
        // Turn the stick direction backward by the robot's heading, so pushing the stick
        // "up" always means "away from the driver" on the field.
        double turnRadians = -yawDegrees / 180 * Math.PI;
        double fieldStrafe = strafe * Math.cos(turnRadians) - drive * Math.sin(turnRadians);
        double fieldDrive = strafe * Math.sin(turnRadians) + drive * Math.cos(turnRadians);

        double total = Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate);
        double divideBy = Math.max(total, 1);

        WheelPowers powers = new WheelPowers();
        powers.frontLeft = (fieldDrive + fieldStrafe + rotate) / divideBy;
        powers.backLeft = (fieldDrive - fieldStrafe + rotate) / divideBy;
        powers.frontRight = (fieldDrive - fieldStrafe - rotate) / divideBy;
        powers.backRight = (fieldDrive + fieldStrafe - rotate) / divideBy;
        return powers;
    }
}
