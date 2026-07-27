package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Drivetrain for a four-wheel X-drive (four omni wheels at the corners, each mounted at 45 degrees
 * so that its axle points at the center of the chassis).
 *
 * Because each wheel pushes along a 45-degree diagonal, an X-drive uses the same power mixing as a
 * mecanum drive: the front-left and back-right wheels push forward-and-right, the front-right and
 * back-left wheels push forward-and-left. Unlike mecanum, an X-drive gets that diagonal from the
 * mounting angle instead of from angled rollers, so there is no roller scrub and the wheels rotate
 * the robot purely tangentially.
 *
 * All three inputs (forward, right, rotate) are in the range -1.0 to +1.0.
 */
public class XDrive {

    // ---------------------------------------------------------------------------------------
    // Configuration - these names must match the robot configuration file on the Driver Station
    // ---------------------------------------------------------------------------------------
    public static final String FRONT_LEFT_NAME  = "front_left_drive";
    public static final String FRONT_RIGHT_NAME = "front_right_drive";
    public static final String BACK_LEFT_NAME   = "back_left_drive";
    public static final String BACK_RIGHT_NAME  = "back_right_drive";
    public static final String IMU_NAME         = "imu";

    /**
     * Motor directions. The left side is reversed here because on most builds the left motors face
     * the opposite way from the right ones. If the robot spins instead of driving forward, flip the
     * direction of whichever pair is fighting the others.
     */
    private static final DcMotor.Direction FRONT_LEFT_DIRECTION  = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction BACK_LEFT_DIRECTION   = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction FRONT_RIGHT_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction BACK_RIGHT_DIRECTION  = DcMotor.Direction.FORWARD;

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final IMU imu;

    /** Scales every wheel power. 1.0 is full speed; lower it for precision driving or demos. */
    private double speedScale = 1.0;

    /**
     * @param hardwareMap   the OpMode's hardwareMap
     * @param logoDirection which way the Control Hub's REV logo points on the robot
     * @param usbDirection  which way the Control Hub's USB ports point on the robot
     */
    public XDrive(HardwareMap hardwareMap,
                  RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
                  RevHubOrientationOnRobot.UsbFacingDirection usbDirection) {

        frontLeft  = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_NAME);
        backLeft   = hardwareMap.get(DcMotorEx.class, BACK_LEFT_NAME);
        backRight  = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_NAME);

        frontLeft.setDirection(FRONT_LEFT_DIRECTION);
        frontRight.setDirection(FRONT_RIGHT_DIRECTION);
        backLeft.setDirection(BACK_LEFT_DIRECTION);
        backRight.setDirection(BACK_RIGHT_DIRECTION);

        for (DcMotorEx motor : new DcMotorEx[] {frontLeft, frontRight, backLeft, backRight}) {
            // Brake makes the robot stop where you let go of the stick instead of coasting.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // RUN_USING_ENCODER holds a commanded velocity, which keeps the four wheels matched.
            // If your drive motors have no encoder cables, change this to RUN_WITHOUT_ENCODER.
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        imu = hardwareMap.get(IMU.class, IMU_NAME);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));
        imu.resetYaw();
    }

    /**
     * Drives from the robot's point of view: +forward is out the front of the robot no matter which
     * way the robot is pointing.
     */
    public void driveRobotCentric(double forward, double right, double rotate) {
        // X-drive mixing. Each wheel gets the projection of the requested motion onto its own
        // 45-degree push direction, plus its share of the rotation.
        double frontLeftPower  = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower   = forward - right + rotate;
        double backRightPower  = forward + right - rotate;

        // If any wheel would exceed full power, scale all four down together. Clipping them
        // individually instead would bend the robot off the direction the driver asked for.
        double largest = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        frontLeft.setPower(speedScale * frontLeftPower / largest);
        frontRight.setPower(speedScale * frontRightPower / largest);
        backLeft.setPower(speedScale * backLeftPower / largest);
        backRight.setPower(speedScale * backRightPower / largest);
    }

    /**
     * Drives from the driver's point of view: pushing the stick away from you sends the robot away
     * from you, whatever direction the robot happens to be facing. Rotation is unaffected.
     */
    public void driveFieldCentric(double forward, double right, double rotate) {
        double heading = getHeading(AngleUnit.RADIANS);

        // Rotate the requested translation backwards by the robot's heading, so that a field
        // direction becomes the equivalent robot direction.
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double robotForward = forward * cos - right * sin;
        double robotRight   = right * cos + forward * sin;

        driveRobotCentric(robotForward, robotRight, rotate);
    }

    /** Cuts all motor power. */
    public void stop() {
        driveRobotCentric(0, 0, 0);
    }

    /**
     * Tells field-centric drive that the robot is currently pointing "away from the driver".
     * Call this at the start of TeleOp and any time the heading drifts.
     */
    public void resetYaw() {
        imu.resetYaw();
    }

    /** Current robot heading, counter-clockwise positive, measured from the last yaw reset. */
    public double getHeading(AngleUnit unit) {
        return imu.getRobotYawPitchRollAngles().getYaw(unit);
    }

    /** @param scale 0.0 to 1.0 - multiplies every wheel power. */
    public void setSpeedScale(double scale) {
        speedScale = Math.max(0.0, Math.min(1.0, scale));
    }

    public double getSpeedScale() {
        return speedScale;
    }
}
