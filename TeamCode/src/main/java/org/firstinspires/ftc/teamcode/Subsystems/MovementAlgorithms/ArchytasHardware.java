package org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Hardware wrapper class for the Wolfpack Movement Algorithm
 * Configurable for different robot setups - just change the constants!
 */
public class ArchytasHardware {

    // =============================================================================
    // CONFIGURATION CONSTANTS - CHANGE THESE FOR YOUR ROBOT
    // =============================================================================

    // Motor Names (change these to match your robot configuration)
    public static final String FRONT_LEFT_MOTOR = "frontLeft";      // or "motorFrontLeft", "fl", etc.
    public static final String FRONT_RIGHT_MOTOR = "frontRight";    // or "motorFrontRight", "fr", etc.
    public static final String BACK_LEFT_MOTOR = "backLeft";        // or "motorBackLeft", "bl", etc.
    public static final String BACK_RIGHT_MOTOR = "backRight";      // or "motorBackRight", "br", etc.

    // Encoder Names (if using separate encoder pods - can be same as motors)
    public static final String LEFT_ENCODER = "frontLeft";          // Usually front left motor
    public static final String RIGHT_ENCODER = "frontRight";        // Usually front right motor
    public static final String PERP_ENCODER = "backLeft";           // Usually back left or separate pod

    // IMU Name
    public static final String IMU_NAME = "imu";                    // or "imu1", "adafruit_imu", etc.

    // Physical Constants (MEASURE THESE ON YOUR ROBOT!)
    public static final double ENCODER_TICKS_PER_REV = 537.7;       // GoBILDA 5202/5203: 537.7, REV Hex: 288, NeveRest 40: 1120
    public static final double WHEEL_DIAMETER_INCHES = 4.0;         // Measure your wheels!
    public static final double TRACK_WIDTH_INCHES = 15.0;           // Distance between left and right wheels
    public static final double GEAR_RATIO = 1.0;                   // If you have gear reduction

    // Calculated Constants
    public static final double ENCODER_TICKS_PER_INCH =
            ENCODER_TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES) * GEAR_RATIO;

    // =============================================================================
    // HARDWARE OBJECTS
    // =============================================================================

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx leftEncoder, rightEncoder, perpEncoder;
    private BNO055IMU imu;

    private ElapsedTime runtime = new ElapsedTime();
    private double lastTime = 0;

    // State tracking
    private boolean initialized = false;
    private double imuOffset = 0; // For resetting IMU heading

    public ArchytasHardware() {
        // Constructor - hardware will be initialized in init()
    }

    /**
     * Initialize all hardware components
     */
    public boolean init(HardwareMap hwMap) {
        try {
            // Initialize drive motors
            frontLeft = hwMap.get(DcMotorEx.class, FRONT_LEFT_MOTOR);
            frontRight = hwMap.get(DcMotorEx.class, FRONT_RIGHT_MOTOR);
            backLeft = hwMap.get(DcMotorEx.class, BACK_LEFT_MOTOR);
            backRight = hwMap.get(DcMotorEx.class, BACK_RIGHT_MOTOR);

            // Initialize encoders (might be same as drive motors)
            leftEncoder = hwMap.get(DcMotorEx.class, LEFT_ENCODER);
            rightEncoder = hwMap.get(DcMotorEx.class, RIGHT_ENCODER);
            perpEncoder = hwMap.get(DcMotorEx.class, PERP_ENCODER);

            // Set motor directions (CHANGE THESE IF ROBOT MOVES WRONG DIRECTION!)
            frontLeft.setDirection(DcMotor.Direction.REVERSE);   // Usually reversed
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.REVERSE);    // Usually reversed
            backRight.setDirection(DcMotor.Direction.FORWARD);

            // Set encoder directions (should match wheel movement)
            // If encoders read negative when robot moves forward, reverse them
            leftEncoder.setDirection(DcMotor.Direction.REVERSE);
            rightEncoder.setDirection(DcMotor.Direction.FORWARD);
            perpEncoder.setDirection(DcMotor.Direction.FORWARD);  // Adjust based on mounting

            // Reset encoders
            resetEncoders();

            // Set motor modes
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Set zero power behavior
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Initialize IMU
            initIMU(hwMap);

            runtime.reset();
            initialized = true;
            return true;

        } catch (Exception e) {
            // Hardware initialization failed - return false so OpMode knows
            return false;
        }
    }

    private void initIMU(HardwareMap hwMap) {
        try {
            imu = hwMap.get(BNO055IMU.class, IMU_NAME);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu.initialize(parameters);

            // Wait for IMU calibration (important!)
            while (!imu.isGyroCalibrated()) {
                Thread.sleep(50);
            }

        } catch (Exception e) {
            // IMU failed to initialize - will use encoder-only odometry
            imu = null;
        }
    }

    public void resetEncoders() {
        if (!initialized) return;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetIMUHeading() {
        if (imu != null) {
            imuOffset = getRawIMUHeading();
        }
    }

    /**
     * Set motor powers for mecanum drive
     * Powers should be in range [-1, 1]
     */
    public void setMotorPowers(double[] powers) {
        if (!initialized || powers.length != 4) return;

        frontLeft.setPower(powers[0]);
        frontRight.setPower(powers[1]);
        backLeft.setPower(powers[2]);
        backRight.setPower(powers[3]);
    }

    public void setMotorPowers(double frontLeftPower, double frontRightPower,
                               double backLeftPower, double backRightPower) {
        setMotorPowers(new double[]{frontLeftPower, frontRightPower, backLeftPower, backRightPower});
    }

    public void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    /**
     * Get encoder positions in inches
     */
    public double getLeftEncoderPosition() {
        if (!initialized) return 0;
        return leftEncoder.getCurrentPosition() / ENCODER_TICKS_PER_INCH;
    }

    public double getRightEncoderPosition() {
        if (!initialized) return 0;
        return rightEncoder.getCurrentPosition() / ENCODER_TICKS_PER_INCH;
    }

    public double getPerpEncoderPosition() {
        if (!initialized) return 0;
        return perpEncoder.getCurrentPosition() / ENCODER_TICKS_PER_INCH;
    }

    /**
     * Get IMU heading in radians (-π to π)
     * Returns 0 if IMU is not available
     */
    public double getIMUHeading() {
        if (imu == null) return 0;
        return getRawIMUHeading() - imuOffset;
    }

    private double getRawIMUHeading() {
        if (imu == null) return 0;
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    /**
     * Get time delta since last call (for PID calculations)
     */
    public double getDeltaTime() {
        double currentTime = runtime.seconds();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;
        return deltaTime;
    }

    /**
     * Check if IMU is available and calibrated
     */
    public boolean isIMUAvailable() {
        return imu != null && imu.isGyroCalibrated();
    }

    /**
     * Get IMU calibration status for debugging
     */
    public String getIMUStatus() {
        if (imu == null) return "IMU Not Found";

        try {
            BNO055IMU.CalibrationStatus calibrationStatus = imu.getCalibrationStatus();
            BNO055IMU.SystemStatus systemStatus = imu.getSystemStatus();

            return "System: " + systemStatus.toString() +
                    ", Gyro: " + (imu.isGyroCalibrated() ? "Calibrated" : "Not Calibrated") +
                    ", Accel: " + (imu.isAccelerometerCalibrated() ? "Calibrated" : "Not Calibrated") +
                    ", Mag: " + (imu.isMagnetometerCalibrated() ? "Calibrated" : "Not Calibrated");
        } catch (Exception e) {
            return "IMU Status: Error reading (" + e.getMessage() + ")";
        }
    }
    /**
     * Get motor velocities in inches/sec (if using encoders)
     */
    public double getFrontLeftVelocity() {
        if (!initialized) return 0;
        return frontLeft.getVelocity() / ENCODER_TICKS_PER_INCH;
    }

    public double getFrontRightVelocity() {
        if (!initialized) return 0;
        return frontRight.getVelocity() / ENCODER_TICKS_PER_INCH;
    }

    public double getBackLeftVelocity() {
        if (!initialized) return 0;
        return backLeft.getVelocity() / ENCODER_TICKS_PER_INCH;
    }

    public double getBackRightVelocity() {
        if (!initialized) return 0;
        return backRight.getVelocity() / ENCODER_TICKS_PER_INCH;
    }

    /**
     * Emergency stop - immediately stop all motors
     */
    public void emergencyStop() {
        if (initialized) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }

    public boolean isInitialized() {
        return initialized;
    }

    // =============================================================================
    // CONFIGURATION METHODS FOR TEAMS TO CUSTOMIZE
    // =============================================================================

    /**
     * Call this method to test if motors are wired correctly
     * Each motor should spin forward briefly
     */
    public void testMotors() {
        if (!initialized) return;

        // Test each motor individually
        frontLeft.setPower(0.3);
        try { Thread.sleep(1000); } catch (InterruptedException e) {}
        frontLeft.setPower(0);

        frontRight.setPower(0.3);
        try { Thread.sleep(1000); } catch (InterruptedException e) {}
        frontRight.setPower(0);

        backLeft.setPower(0.3);
        try { Thread.sleep(1000); } catch (InterruptedException e) {}
        backLeft.setPower(0);

        backRight.setPower(0.3);
        try { Thread.sleep(1000); } catch (InterruptedException e) {}
        backRight.setPower(0);
    }

    /**
     * Test drive directions - robot should move forward
     */
    public void testDriveForward() {
        if (!initialized) return;
        setMotorPowers(0.3, 0.3, 0.3, 0.3);
        try { Thread.sleep(2000); } catch (InterruptedException e) {}
        stopMotors();
    }

    /**
     * Test strafe right
     */
    public void testStrafeRight() {
        if (!initialized) return;
        setMotorPowers(0.3, -0.3, -0.3, 0.3);
        try { Thread.sleep(2000); } catch (InterruptedException e) {}
        stopMotors();
    }
}