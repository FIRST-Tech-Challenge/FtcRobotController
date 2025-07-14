package ftc_library.hardware;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.RobotLog;

import androidx.annotation.NonNull;

/**
 * RobotHardware is the central place to initialize and store references to all robot hardware.
 * This includes motors, servos, and sensors. Utility methods for basic actions and power curves
 * are also included here for easy access and code reuse.
 */
public class RobotHardware {
    // Drive motors
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    // Manipulator motors
    public DcMotorEx horizontalArm, rightHang, leftHang, verticalArm;
    // Servos for manipulators and mechanisms
    public Servo wrist, spintake, bucketWrist;
    // IMU sensor for orientation (used in field-oriented driving)
    public BHI260IMU imu;

    // Constants for arm and servo positions
    public final double ARM_FORWARD_POWER = -0.5;
    public final double ARM_BACKWARD_POWER = 0.5;
    public final double HANG_POWER = 1.0;
    public static final double REST_POSITION = 0.0;
    public static final double FLIP_POSITION = 0.75;

    /**
     * Initializes all hardware devices using the provided hardware map.
     * This should be called once in your OpMode's init() method.
     */
    public void init(HardwareMap hardwareMap) {
        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        // Initialize manipulator motors
        rightHang = hardwareMap.get(DcMotorEx.class, "rightHang");
        leftHang = hardwareMap.get(DcMotorEx.class, "leftHang");
        verticalArm = hardwareMap.get(DcMotorEx.class, "verticalArm");
        horizontalArm = hardwareMap.get(DcMotorEx.class, "horizontalArm");
        // Initialize servos
        wrist = hardwareMap.get(Servo.class, "wrist");
        spintake = hardwareMap.get(Servo.class, "spintake");
        bucketWrist = hardwareMap.get(Servo.class, "bucketWrist");
        wrist.setDirection(Servo.Direction.FORWARD);
        bucketWrist.setPosition(REST_POSITION);
        // Initialize IMU for orientation sensing
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);
        // Set motor directions as needed
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Utility method to keep a value within a specified range.
     */
    public static double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    /**
     * Applies a power curve to joystick input for smoother robot driving.
     * @param power    The raw joystick value (-1 to 1)
     * @param slowSpeed If true, reduces power for precision movement
     * @return Adjusted power value
     */
    public static double powerCurve(double power, boolean slowSpeed) {
        return slowSpeed ? power / 8.0 : power / 3.0;
    }

    /**
     * Applies a non-linear power curve for arm movement to prevent slamming and allow precision.
     * @param power The raw joystick value for the arm (-1 to 1)
     * @return Adjusted arm power value
     */
    public static double armPowerCurve(double power) {
        final double clam = 2.8;
        double value = Math.pow(Math.abs(power), clam) * Math.signum(power);
        value = Math.min(value, 0.95); // Max up power
        value = Math.max(value, -0.35); // Max down power
        RobotLog.v("Arm power value: " + value);
        return value;
    }

    /**
     * Stops all drive motors immediately.
     */
    public void stopMoving() {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    /**
     * Moves the robot forward or backward at the given power.
     * @param power Positive for forward, negative for backward
     */
    public void moveY(double power) {
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }

    /**
     * Strafes the robot left or right at the given power.
     * @param power Positive for right, negative for left
     */
    public void moveX(double power) {
        frontRight.setPower(-power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(-power);
    }

    /**
     * Initializes an arm motor with PIDF coefficients for position control.
     */
    public static void initArmMotor(@NonNull DcMotorEx motor) {
        motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motor.setPower(0.0);
        motor.setTargetPosition(0);
        motor.setTargetPositionTolerance(10);
        PIDFCoefficients pid = new PIDFCoefficients(35.0, 0.005, 0.002, 0.0, MotorControlAlgorithm.LegacyPID);
        motor.setPIDFCoefficients(RunMode.RUN_TO_POSITION, pid);
        RobotLog.v("Arm PID set to " + pid);
    }

    /**
     * Initializes an arm motor for manual (open loop) control.
     */
    public void initArmMotorSimple(DcMotorEx motor) {
        motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motor.setPower(0.0);
        motor.setTargetPosition(0);
        motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Moves an arm motor to a specific encoder position.
     * @param motor    The motor to move
     * @param position The encoder target position
     * @param slowSpeed If true, moves slower for precision
     */
    protected void moveArmMotor(DcMotorEx motor, int position, boolean slowSpeed) {
        motor.setPower(1.0);
        motor.setTargetPosition(position);
        motor.setMode(RunMode.RUN_TO_POSITION);
    }
}
