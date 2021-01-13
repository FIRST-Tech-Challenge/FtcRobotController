package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotUtilities.MovementVars;
import org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition;

import java.util.List;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

/**
 *Created by 7592 Roarbots
 */
public class UltimateGoalRobot
{
    /* Public OpMode members. */
    public final static double SHOOT_POWER = 0.47;
    public final static double SHOOT_VELOCITY = 1120;
    public final static double SHOOT_VELOCITY_ERROR = 20;
    public final static double THROTTLE_TIMEOUT = 7000;
    public final static double STRAFE_MULTIPLIER = 1.5;
    public final static double SLOW_STRAFE_MULTIPLIER = 1.5;
    public final static double MIN_FOUNDATION_SPIN_RATE = 0.19;
    public final static double MIN_FOUNDATION_DRIVE_RATE = 0.18;
    public final static double MIN_FOUNDATION_STRAFE_RATE = 0.19;
    public final static double MIN_SPIN_RATE = 0.12;
    public final static double MIN_DRIVE_RATE = 0.10;
    public final static double MIN_STRAFE_RATE = 0.19;
    public final static double MIN_DRIVE_MAGNITUDE = Math.sqrt(MIN_DRIVE_RATE*MIN_DRIVE_RATE+MIN_DRIVE_RATE*MIN_DRIVE_RATE);
    public final static double MIN_FOUNDATION_DRIVE_MAGNITUDE = Math.sqrt(MIN_FOUNDATION_DRIVE_RATE*MIN_FOUNDATION_DRIVE_RATE+MIN_FOUNDATION_DRIVE_RATE*MIN_FOUNDATION_DRIVE_RATE);

    // Robot Controller Config Strings
    public final static String IMU = "imu";
    public final static String FRONT_LEFT_MOTOR = "FrontLeft";
    public final static String FRONT_RIGHT_MOTOR = "FrontRight";
    public final static String REAR_LEFT_MOTOR = "RearLeft";
    public final static String REAR_RIGHT_MOTOR = "RearRight";
    public final static String INTAKE_MOTOR = "Intake";
    public final static String WOBBLE_MOTOR = "Wobble";
    public final static String SHOOTER_MOTOR = "Shooter";
    public final static String EMPTY_MOTOR = "Empty";
    public final static String CLAW_SERVO = "Claw";
    public final static String FLAP_SERVO = "Flap";
    public final static String INJECTOR_SERVO = "Injector";

    public enum FLAP_POSITION {
        POWERSHOT,
        HIGH_GOAL
    }
    public FLAP_POSITION flapPosition;
    public final static double FLAP_POWERSHOT = 0.620;
    public double powerShotOffset = 0.0;
    public final static double FLAP_HIGH_GOAL = 0.525;
    public double highGoalOffset = 0.0;
    public double flapAngle;

    // We need both hubs here because one has the motors, and the other has the
    // odometry encoders.
    public final static String CTRL_HUB = "Control Hub 1";
    public final static String EX_HUB = "Expansion Hub 3";

    List<LynxModule> allHubs;

    // These motors have the odometry encoders attached
    protected DcMotorEx intake = null;
    protected DcMotorEx wobble = null;
    protected DcMotorEx empty = null;

    // Other motors
    protected DcMotorEx frontLeft = null;
    protected DcMotorEx frontRight = null;
    protected DcMotorEx rearLeft = null;
    protected DcMotorEx rearRight = null;
    protected DcMotorEx shooter = null;

    // Servos
    protected Servo flap = null;
    protected Servo claw = null;
    protected Servo injector = null;

    // Sensors
    protected BNO055IMU imu = null;

    // Tracking variables
    private static final int encoderClicksPerSecond = 2800;
    protected double frontLeftMotorPower = 0.0;
    protected double rearLeftMotorPower = 0.0;
    protected double frontRightMotorPower = 0.0;
    protected double rearRightMotorPower = 0.0;
    protected double shooterMotorTargetVelocity = 0;
    protected double intakeMotorPower = 0.0;
    protected double wobbleMotorPower = 0.0;

    public boolean defaultInputShaping = true;
    protected boolean imuRead = false;
    protected double imuValue = 0.0;
    protected double strafeMultiplier = STRAFE_MULTIPLIER;

    // Servo Timer variables
    private ElapsedTime clawTimer;
    private ElapsedTime flapTimer;
    private ElapsedTime injectTimer;

    public static boolean encodersReset = false;
    public boolean forceReset = false;
    public boolean disableDriverCentric = false;

    public double xAngle, yAngle, zAngle;
    /* local OpMode members. */
    protected HardwareMap hwMap  =  null;

    /**
     * If the motion value is less than the threshold, the controller will be
     * considered at rest
     */
    protected static float joystickDeadzone = 0.07f;
    private static final float MAX_MOTION_RANGE = 1.0f;
    private static final float MIN_MOTION_RANGE = (float)MIN_DRIVE_RATE;

    // Used to clean up the slop in the joysticks.
    public static float cleanMotionValues(float number) {
        // apply deadzone
        if (number < joystickDeadzone && number > -joystickDeadzone) return 0.0f;
        // apply trim
        if (number >  MAX_MOTION_RANGE) return  MAX_MOTION_RANGE;
        if (number < -MAX_MOTION_RANGE) return -MAX_MOTION_RANGE;
        // scale values "between deadzone and trim" to be "between Min range and Max range"
        if (number > 0)
            number = (float) Range.scale(number, joystickDeadzone, MAX_MOTION_RANGE, MIN_MOTION_RANGE, MAX_MOTION_RANGE);
        else
            number = (float)Range.scale(number, -joystickDeadzone, -MAX_MOTION_RANGE, -MIN_MOTION_RANGE, -MAX_MOTION_RANGE);

        return number;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // The hubs are used for resetting reads for bulk reads.
        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Define and Initialize Motors
        frontLeft = hwMap.get(DcMotorEx.class, FRONT_LEFT_MOTOR);
        frontRight = hwMap.get(DcMotorEx.class, FRONT_RIGHT_MOTOR);
        rearLeft = hwMap.get(DcMotorEx.class, REAR_LEFT_MOTOR);
        rearRight = hwMap.get(DcMotorEx.class, REAR_RIGHT_MOTOR);
        intake = hwMap.get(DcMotorEx.class, INTAKE_MOTOR);
        wobble = hwMap.get(DcMotorEx.class, WOBBLE_MOTOR);
        empty = hwMap.get(DcMotorEx.class, EMPTY_MOTOR);
        shooter = hwMap.get(DcMotorEx.class, SHOOTER_MOTOR);


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        // Changing these values affect the direction of the encoder reads.
        wobble.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        empty.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        setAllDriveZero();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobble.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        empty.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set the stop mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        empty.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize servos
        flap = hwMap.get(Servo.class, FLAP_SERVO);
        injector = hwMap.get(Servo.class, INJECTOR_SERVO);
        claw = hwMap.get(Servo.class, CLAW_SERVO);

        injector.setPosition(INJECTOR_HOME);
        claw.setPosition(CLAW_CLOSED);
        clawClosed = true;
        setShooterFlapPowerShot();

        // Define and initialize timers.
        clawTimer = new ElapsedTime();
        flapTimer = new ElapsedTime();
        injectTimer = new ElapsedTime();

        // Let's try to tweak the PIDs
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(200,
                3, 0, 0, MotorControlAlgorithm.PIDF));
//		  frontLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10,
//                3, 0, 12, MotorControlAlgorithm.PIDF));
//        frontRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10,
//                3, 0, 12, MotorControlAlgorithm.PIDF));
//        rearLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10,
//                3, 0, 12, MotorControlAlgorithm.PIDF));
//        rearRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10,
//                3, 0, 12, MotorControlAlgorithm.PIDF));

        initIMU();
    }

    public int getLeftEncoderWheelPosition() {
        // This is to compensate for GF having a negative left.
        return -intake.getCurrentPosition();
    }

    public int getRightEncoderWheelPosition() {
        return wobble.getCurrentPosition();
    }

    public int getStrafeEncoderWheelPosition() {
        return empty.getCurrentPosition();
    }

    public void setInputShaping(boolean inputShapingEnabled) {
        defaultInputShaping = inputShapingEnabled;
    }

    public void initIMU()
    {
        // Init IMU code
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, IMU);
        imu.initialize(parameters);
    }

    public void resetReads() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        // The IMU is handled separately because it uses I2C which is not part of the bulk read.
        imuRead = false;
    }

    public double readIMU()
    {
        if(!imuRead) {
            // Read IMU Code
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            imuValue = (double)angles.firstAngle;
            imuRead = true;
        }

        return imuValue;
    }

    public void setShooterFlapPowerShot() {
        flapAngle = FLAP_POWERSHOT + powerShotOffset;
        flap.setPosition(flapAngle);
        flapPosition = FLAP_POSITION.POWERSHOT;
    }

    public void setShooterFlapHighGoal() {
        flapAngle = FLAP_HIGH_GOAL + highGoalOffset;
        flap.setPosition(flapAngle);
        flapPosition = FLAP_POSITION.HIGH_GOAL;
    }

    public void toggleShooter() {
        if(shooterMotorTargetVelocity != SHOOT_VELOCITY) {
            shooter.setVelocity(SHOOT_VELOCITY);
            shooterMotorTargetVelocity = SHOOT_VELOCITY;
        } else {
            shooter.setVelocity(0);
            shooterMotorTargetVelocity = 0;
        }
    }

    public void setIntakeMotorPower(double power) {
        if (Math.abs(power - intakeMotorPower) > 0.005) {
            intakeMotorPower = power;
            intake.setPower(power);
        }
    }

    public void setWobbleMotorPower(double power) {
        if (Math.abs(power - wobbleMotorPower) > 0.005) {
            wobbleMotorPower = power;
            wobble.setPower(power);
        }
    }

    public void setFrontLeftMotorPower(double power)
    {
        if(Math.abs(power - frontLeftMotorPower) > 0.005)
        {
            frontLeftMotorPower = power;
            frontLeft.setPower(power);
        }
    }

    public void setRearLeftMotorPower(double power)
    {
        if(Math.abs(power - rearLeftMotorPower) > 0.005)
        {
            rearLeftMotorPower = power;
            rearLeft.setPower(power);
        }
    }

    public void setFrontRightMotorPower(double power)
    {
        if(Math.abs(power - frontRightMotorPower) > 0.005)
        {
            frontRightMotorPower = power;
            frontRight.setPower(power);
        }
    }

    public void setRearRightMotorPower(double power)
    {
        if(Math.abs(power - rearRightMotorPower) > 0.005)
        {
            rearRightMotorPower = power;
            rearRight.setPower(power);
        }
    }

    public void setAllDrive(double power) {
        setFrontLeftMotorPower(power);
        setFrontRightMotorPower(power);
        setRearRightMotorPower(power);
        setRearLeftMotorPower(power);
    }

    public void setAllDriveZero()
    {
        setAllDrive(0.0);
    }

    /**
     *
     * @param xPower - -1.0 to 1.0 power in the X axis
     * @param yPower - -1.0 to 1.0 power in the Y axis
     * @param spin - -1.0 to 1.0 power to rotate the robot, reduced to MAX_SPIN_RATE
     * @param angleOffset - The offset from the gyro to run at, such as drive compensation
     */
    public void drive(double xPower, double yPower, double spin, double angleOffset, boolean inputShaping) {
        double gyroAngle = angleOffset;
        if(!disableDriverCentric) {
            gyroAngle += readIMU();
        }

        double joystickMagnitude = sqrt(xPower*xPower + yPower*yPower);
        double driveAngle = atan2(yPower, xPower);
        double robotDriveAngle = driveAngle - Math.toRadians(gyroAngle) + Math.toRadians(90);
        double newPower = driverInputShaping(joystickMagnitude, inputShaping);

        MovementVars.movement_turn = driverInputSpinShaping(spin, inputShaping);
        MovementVars.movement_x = newPower * cos(robotDriveAngle);
        MovementVars.movement_y = newPower * sin(robotDriveAngle);

		ApplyMovement();
    }

    protected double driverInputShaping( double valueIn, boolean inputShaping) {
        double aValue = 0.77;
        double valueOut = 0.0;

        if(valueIn == 0.0) {
            valueOut = 0.0;
        } else {
            if (inputShaping) {
                valueOut = aValue * Math.pow(valueIn, 3) + (1 - aValue) * valueIn;
                valueOut = Math.copySign(Math.max(MIN_DRIVE_RATE, Math.abs(valueOut)), valueOut);
            } else {
                valueOut = valueIn;
            }
        }

        return valueOut;
    }

    protected double driverInputSpinShaping( double valueIn, boolean inputShaping) {
        double aValue = 0.77;
        double valueOut;

        if(valueIn == 0.0) {
            valueOut = 0.0;
        } else {
            if (inputShaping) {
                valueOut = aValue * Math.pow(valueIn, 3) + (1 - aValue) * valueIn;
                valueOut = Math.copySign(Math.max(MIN_SPIN_RATE, Math.abs(valueOut)), valueOut);
            } else {
                valueOut = valueIn;
            }
        }

        return valueOut;
    }

    public void disableDriveEncoders()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoders()
    {
        int sleepTime = 0;
        int encoderCount = frontLeft.getCurrentPosition();

        // The Odometry Encoders
        wobble.setMode(RunMode.STOP_AND_RESET_ENCODER);
        empty.setMode(RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(RunMode.STOP_AND_RESET_ENCODER);

        shooter.setMode(RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);

        while((encoderCount != 0) && (sleepTime < 1000)) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) { break; }
            sleepTime += 10;
            resetReads();
            encoderCount = frontLeft.getCurrentPosition();
        }

        // The Odometry Encoders
        wobble.setMode(RunMode.RUN_WITHOUT_ENCODER);
        empty.setMode(RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(RunMode.RUN_WITHOUT_ENCODER);

        shooter.setMode(RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(RunMode.RUN_USING_ENCODER);
        frontRight.setMode(RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(RunMode.RUN_USING_ENCODER);
        rearRight.setMode(RunMode.RUN_USING_ENCODER);
    }

    /**
     * @param targetAngle  - The angle the robot should try to face when reaching destination.
     * @param pullingFoundation - If we are pulling the foundation.
     * @param resetDriveAngle - When we start a new drive, need to reset the starting drive angle.
     * @return - Boolean true we have reached destination, false we have not
     */
    public double lastDriveAngle;
    public boolean rotateToAngle(double targetAngle, boolean pullingFoundation, boolean resetDriveAngle) {
        boolean reachedDestination = false;
        double errorMultiplier = pullingFoundation ? 0.04 : 0.016;
        double minSpinRate = pullingFoundation ? MIN_FOUNDATION_SPIN_RATE : MIN_SPIN_RATE;
        double deltaAngle = MyPosition.AngleWrap(targetAngle - MyPosition.worldAngle_rad);
        double turnSpeed = Math.toDegrees(deltaAngle) * errorMultiplier;

        // This should be set on the first call to start us on a new path.
        if(resetDriveAngle) {
            lastDriveAngle = deltaAngle;
        }

        // We are done if we are within 2 degrees
        if(Math.abs(Math.toDegrees(deltaAngle)) < 2) {
            // We have reached our destination if the angle is close enough
            setAllDriveZero();
            reachedDestination = true;
            // We are done when we flip signs.
        } else if(lastDriveAngle < 0) {
            // We have reached our destination if the delta angle sign flips from last reading
            if(deltaAngle >= 0) {
                setAllDriveZero();
                reachedDestination = true;
            } else {
                // We still have some turning to do.
                MovementVars.movement_x = 0;
                MovementVars.movement_y = 0;
                if(turnSpeed > -minSpinRate) {
                    turnSpeed = -minSpinRate;
                }
                MovementVars.movement_turn = turnSpeed;
                ApplyMovement();
            }
        } else {
            // We have reached our destination if the delta angle sign flips
            if(deltaAngle <= 0) {
                setAllDriveZero();
                reachedDestination = true;
            } else {
                // We still have some turning to do.
                MovementVars.movement_x = 0;
                MovementVars.movement_y = 0;
                if(turnSpeed < minSpinRate) {
                    turnSpeed = minSpinRate;
                }
                MovementVars.movement_turn = turnSpeed;
                ApplyMovement();
            }
        }
        lastDriveAngle = deltaAngle;

        return reachedDestination;
    }

    /**
     * @param x           - The X field coordinate to go to.
     * @param y           - The Y field coordinate to go to.
     * @param targetAngle - The angle the robot should try to face when reaching destination in radians.
     * @param minSpeed    - The minimum speed that allows movement.
     * @param maxSpeed    - Sets the maximum speed to drive.
     * @param errorMultiplier - Sets the proportional speed to slow down.
     * @param allowedError - Sets the allowable error to claim target reached.
     * @param passThrough - Allows waypoint to be a drive through where the robot won't slow down.
     * @return - Boolean true we have reached destination, false we have not
     */
    public boolean driveToXY(double x, double y, double targetAngle, double minSpeed,
                             double maxSpeed, double errorMultiplier, double allowedError,
                             boolean passThrough) {
        boolean reachedDestination = false;
        double deltaX = x - MyPosition.worldXPosition;
        double deltaY = y - MyPosition.worldYPosition;
        double driveAngle = Math.atan2(deltaY, deltaX);
        double deltaAngle = MyPosition.AngleWrap(targetAngle - MyPosition.worldAngle_rad);
        double magnitude = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double driveSpeed;
        double turnSpeed = Math.toDegrees(deltaAngle) * errorMultiplier;
        // Have to convert from world angles to robot centric angles.
        double robotDriveAngle = driveAngle - MyPosition.worldAngle_rad + Math.toRadians(90);

        // This will allow us to do multi-point routes without huge slowdowns.
        // Such use cases will be changing angles, or triggering activities at
        // certain points.
        if(!passThrough) {
            driveSpeed = magnitude * errorMultiplier;
        } else {
            driveSpeed = maxSpeed;
        }

        if(driveSpeed < minSpeed) {
            driveSpeed = minSpeed;
        } else if (driveSpeed > maxSpeed) {
            driveSpeed = maxSpeed;
        }

        // Check if we passed through our point
        if(magnitude <= allowedError) {
            reachedDestination = true;
            if(!passThrough) {
                setAllDriveZero();
            } else {
                // This can happen if the robot is already at error distance for drive through
                MovementVars.movement_x = driveSpeed * Math.cos(robotDriveAngle);
                MovementVars.movement_y = driveSpeed * Math.sin(robotDriveAngle);
                MovementVars.movement_turn = turnSpeed;
                ApplyMovement();
            }
        } else {
            MovementVars.movement_x = driveSpeed * Math.cos(robotDriveAngle);
            MovementVars.movement_y = driveSpeed * Math.sin(robotDriveAngle);
            MovementVars.movement_turn = turnSpeed;
            ApplyMovement();
        }

        return reachedDestination;
    }
    /**
     * @param x           - The X field coordinate to go to.
     * @param y           - The Y field coordinate to go to.
     * @param targetAngle  - The angle the robot should try to face when reaching destination in radians.
     * @param maxSpeed    - Sets the speed when we are driving through the point.
     * @param passThrough - Slows the robot down to stop at destination coordinate.
     * @param pullingFoundation - If we are pulling the foundation.
     * @return - Boolean true we have reached destination, false we have not
     */
    public boolean driveToXY(double x, double y, double targetAngle, double maxSpeed,
                             boolean passThrough, boolean pullingFoundation) {
        double errorMultiplier = pullingFoundation ? 0.020 : 0.014;
        double minDriveMagnitude = pullingFoundation ? MIN_FOUNDATION_DRIVE_MAGNITUDE : MIN_DRIVE_MAGNITUDE;
        double allowedError = 2;

        if(passThrough) {
            allowedError = 7;
        }
        return (driveToXY(x, y, targetAngle, minDriveMagnitude, maxSpeed, errorMultiplier,
                allowedError, passThrough));
    }

    // Odometry updates
    private long lastUpdateTime = 0;

    /**converts movement_y, movement_x, movement_turn into motor powers */
    public void ApplyMovement() {
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16){
            return;
        }
        lastUpdateTime = currTime;

        // 2.1 is the ratio between the minimum power to strafe, 0.19, and driving, 0.09.
        double tl_power_raw = MovementVars.movement_y-MovementVars.movement_turn+MovementVars.movement_x*strafeMultiplier;
        double bl_power_raw = MovementVars.movement_y-MovementVars.movement_turn-MovementVars.movement_x*strafeMultiplier;
        double br_power_raw = -MovementVars.movement_y-MovementVars.movement_turn-MovementVars.movement_x*strafeMultiplier;
        double tr_power_raw = -MovementVars.movement_y-MovementVars.movement_turn+MovementVars.movement_x*strafeMultiplier;

        //find the maximum of the powers
        double maxRawPower = Math.abs(tl_power_raw);
        if(Math.abs(bl_power_raw) > maxRawPower){ maxRawPower = Math.abs(bl_power_raw);}
        if(Math.abs(br_power_raw) > maxRawPower){ maxRawPower = Math.abs(br_power_raw);}
        if(Math.abs(tr_power_raw) > maxRawPower){ maxRawPower = Math.abs(tr_power_raw);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }
        tl_power_raw *= scaleDownAmount;
        bl_power_raw *= scaleDownAmount;
        br_power_raw *= scaleDownAmount;
        tr_power_raw *= scaleDownAmount;

        //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
        setFrontLeftMotorPower(tl_power_raw);
        setFrontRightMotorPower(tr_power_raw);
        setRearRightMotorPower(br_power_raw);
        setRearLeftMotorPower(bl_power_raw);
    }

    /** Grab activity closes or opens the wobble arm claw. **/
    public final static double CLAW_TIME = 500.0;
    public final static double CLAW_CLOSED = 0.165;
    public final static double CLAW_OPEN = 0.40;
    public boolean clawClosed = false;
    public enum GRABBING {
        IDLE,
        CLOSING
    }

    public GRABBING grabState = GRABBING.IDLE;
    public void startClawToggle() {
        if(grabState == GRABBING.IDLE) {
            if(clawClosed) {
                claw.setPosition(CLAW_OPEN);
            } else {
                claw.setPosition(CLAW_CLOSED);
            }
            clawTimer.reset();
            grabState = GRABBING.CLOSING;
        }
    }

    public void performClawToggle() {
        switch(grabState) {
            case CLOSING:
                if(clawTimer.milliseconds() >= CLAW_TIME) {
                    grabState = GRABBING.IDLE;
                    clawClosed = !clawClosed;
                }
                break;
            case IDLE:
            default:
                break;
        }
    }

    /** Inject activity pushes a disk into the shooter and resets th
     * e injector. **/
    public final static double INJECTOR_FIRE_TIME = 300.0;
    public final static double INJECTOR_RESET_TIME = 200.0;
    public final static double INJECTOR_HOME_TIME = 100.0;
    public final static double SHOOTER_THROTTLE_DELAY = 500.0;
    public final static double INJECTOR_HOME = 0.512;
    public final static double INJECTOR_RESET = 0.450;
    public final static double INJECTOR_FIRE = 0.800;
    public final static int VELOCITY_SUCCESS_CHECKS = 3;
    public boolean disableVelocityCheck = false;
    public int sequentialStableVelocityChecks = 0;
    public enum INJECTING {
        IDLE,
        THROTTLING_UP,
        FIRING,
        RESETTING,
        HOMING
    }

    public INJECTING injectState = INJECTING.IDLE;
    public void startInjecting() {
        if(injectState == INJECTING.IDLE) {
            // If the shooter isn't on, fire it up.
            if(shooterMotorTargetVelocity != SHOOT_VELOCITY) {
                toggleShooter();
            }
            sequentialStableVelocityChecks = 0;
            injectTimer.reset();
            injectState = INJECTING.THROTTLING_UP;
        }
    }

    public void performInjecting() {
        switch(injectState) {
            case THROTTLING_UP:
                boolean throttledUp = false;
                // If the stable velocity isn't working, just wait for a small timer.
                // Will have to reset the velocity check by driver input.
                if(disableVelocityCheck) {
                    if(injectTimer.milliseconds() >= SHOOTER_THROTTLE_DELAY) {
                        throttledUp = true;
                    }
                } else {
                    // This means we can not reach target velocity, so disable
                    // velocity targeting and just use a timer in the future.
                    if(injectTimer.milliseconds() > THROTTLE_TIMEOUT) {
                        disableVelocityCheck = true;
                        throttledUp = true;
                    } else {
                        // Verify this loop is at target velocity.
                        if(Math.abs(shooter.getVelocity() - SHOOT_VELOCITY) <= SHOOT_VELOCITY_ERROR) {
                            sequentialStableVelocityChecks++;
                            if(sequentialStableVelocityChecks >= VELOCITY_SUCCESS_CHECKS) {
                                throttledUp = true;
                            }
                        } else {
                            sequentialStableVelocityChecks = 0;
                        }
                    }

                }
                if(throttledUp) {
                    injector.setPosition(INJECTOR_FIRE);
                    injectTimer.reset();
                    injectState = INJECTING.FIRING;
                }
                break;
            case FIRING:
                if(injectTimer.milliseconds() >= INJECTOR_FIRE_TIME) {
                    injector.setPosition(INJECTOR_RESET);
                    injectTimer.reset();
                    injectState = INJECTING.RESETTING;
                }
                break;
            case RESETTING:
                if(injectTimer.milliseconds() >= INJECTOR_RESET_TIME) {
                    injector.setPosition(INJECTOR_HOME);
                    injectTimer.reset();
                    injectState = INJECTING.HOMING;
                }
                break;
            case HOMING:
                if(injectTimer.milliseconds() >= INJECTOR_HOME_TIME) {
                    injectState = INJECTING.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    }

    /** In case we want to create a short cut to fire 3 as quickly as possible. **/
    public enum TRIPLE_INJECTING {
        IDLE,
        FIRING_ONE,
        FIRING_TWO,
        FIRING_THREE
    }
    public TRIPLE_INJECTING tripleInjectState = TRIPLE_INJECTING.IDLE;
    public void startTripleInjecting() {
        if (tripleInjectState == TRIPLE_INJECTING.IDLE) {
            startInjecting();
            tripleInjectState = TRIPLE_INJECTING.FIRING_ONE;
        }
    }

    public void performTripleInjecting() {
        switch(tripleInjectState) {
            case FIRING_ONE:
                if(injectState == INJECTING.IDLE) {
                    tripleInjectState = TRIPLE_INJECTING.FIRING_TWO;
                    startInjecting();
                }
                break;
            case FIRING_TWO:
                if(injectState == INJECTING.IDLE) {
                    tripleInjectState = TRIPLE_INJECTING.FIRING_THREE;
                    startInjecting();
                }
                break;
            case FIRING_THREE:
                if(injectState == INJECTING.IDLE) {
                    tripleInjectState = TRIPLE_INJECTING.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    }
}

