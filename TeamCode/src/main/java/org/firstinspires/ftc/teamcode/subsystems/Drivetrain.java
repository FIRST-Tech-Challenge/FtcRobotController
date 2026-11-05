package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

/*
 * DRIVETRAIN - the four mecanum wheel motors.
 *
 * Put here:
 *  - motor setup in init() (names from RobotConstants, directions, brake mode)
 *  - drive(forward, strafe, turn) with the mecanum math
 *  - optional: field-centric drive (needs heading from Odometry)
 * Do NOT put here: gamepad reading (that belongs in the TeleOp).
 */

public class Drivetrain implements Subsystem {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private double forward, strafe, turn; // latest driver request, applied in update()

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    @Override
    public void init() {
        RobotLog.i("Drivetrain: Initializing motors");
        frontLeft  = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_LEFT_MOTOR);
        backLeft   = hardwareMap.get(DcMotor.class, RobotConstants.BACK_LEFT_MOTOR);
        frontRight = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_RIGHT_MOTOR);
        backRight  = hardwareMap.get(DcMotor.class, RobotConstants.BACK_RIGHT_MOTOR);

        // If a wheel spins the wrong way, flip its direction in RobotConstants.
        frontLeft.setDirection(RobotConstants.FRONT_LEFT_DIRECTION);
        backLeft.setDirection(RobotConstants.BACK_LEFT_DIRECTION);
        frontRight.setDirection(RobotConstants.FRONT_RIGHT_DIRECTION);
        backRight.setDirection(RobotConstants.BACK_RIGHT_DIRECTION);

        // Stop quickly instead of coasting.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotLog.i("Drivetrain: Motors initialized");
    }

    @Override
    public void update() {
        double fl = forward + strafe + turn;
        double bl = forward - strafe + turn;
        double fr = forward - strafe - turn;
        double br = forward + strafe - turn;

        // If any value is over 1.0, scale all of them down so the motion keeps its shape.
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                     Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);

        telemetry.addData("Drive", "fwd %.2f  strafe %.2f  turn %.2f", forward, strafe, turn);
    }

    @Override
    public void stop() {
        forward = strafe = turn = 0;
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    // TODO: field-centric drive and other drive modes

    /**
     * Robot-centric mecanum drive. Each value is -1 to 1.
     * forward: + is forward. strafe: + is right. turn: + is clockwise.
     * This only saves the request; update() sets the motor power.
     */
    public void drive(double forward, double strafe, double turn) {
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
    }
}
