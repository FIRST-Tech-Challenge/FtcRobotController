package org.firstinspires.ftc.teamcode.simloopexample;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A mecanum drivetrain with no path follower in it, written the way an ordinary FTC subsystem is
 * written.
 *
 * <p>It takes three normalised commands and turns them into four motor powers. There is deliberately no
 * odometry, no pose estimate and no follower here: SimLoop simulates the chassis, and whichever follower
 * your team runs - Pedro Pathing, Road Runner, your own - sits on top of a subsystem shaped like this
 * one. An example that shipped a follower would be an example of that follower.
 *
 * <p><b>The robot this belongs to is invented.</b>
 */
public class ExampleDrive {

    /**
     * Motor names in the robot configuration, in {@code [frontLeft, frontRight, backLeft, backRight]}
     * order. That order is not arbitrary: it is the order SimLoop's {@code MecanumDrivePlant} reads its
     * four motors in, so a test wires the plant from this array rather than retyping the names.
     */
    public static final String[] MOTOR_NAMES = {"frontLeft", "frontRight", "backLeft", "backRight"};

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    /**
     * Resolves the four drive motors from the robot's configuration.
     *
     * <p>The right-hand motors are reversed, which is the usual consequence of mounting a gearbox
     * mirrored across the chassis: both sides have to turn the same way to drive the robot forward, and
     * mirrored motors turn opposite ways to do it.
     *
     * @param hardwareMap the OpMode's hardware map on a robot, or a {@code FakeHardwareMap} in a
     *                    simulated test
     */
    public ExampleDrive(HardwareMap hardwareMap) {
        this.frontLeft = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[0]);
        this.frontRight = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[1]);
        this.backLeft = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[2]);
        this.backRight = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[3]);

        this.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        this.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        this.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Commands the chassis. All three inputs are unitless fractions of full effort in [-1, 1]; if their
     * combination would ask a wheel for more than full power, all four are scaled down together so the
     * commanded direction is preserved rather than clipped into a different one.
     *
     * @param forward positive drives the robot toward its own front
     * @param strafe  positive drives the robot toward its own left
     * @param turn    positive turns the robot counter-clockwise seen from above
     */
    public void drive(double forward, double strafe, double turn) {
        double frontLeftPower = forward + strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backLeftPower = forward - strafe + turn;
        double backRightPower = forward + strafe - turn;

        double largest = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (largest > 1.0) {
            frontLeftPower /= largest;
            frontRightPower /= largest;
            backLeftPower /= largest;
            backRightPower /= largest;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    /** Cuts power to all four motors. */
    public void stop() {
        drive(0.0, 0.0, 0.0);
    }
}
