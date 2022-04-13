package org.firstinspires.ftc.teamcode.AutonomousDrive;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.MILLIAMPS;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.copySign;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareDrivers.MaxSonarI2CXL;
import org.firstinspires.ftc.teamcode.RobotUtilities.MovementVars;
import org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition;

public class HardwareRobot {
    //====== REV CONTROL/EXPANSION HUBS =====
    LynxModule controlHub;
    LynxModule expansionHub;

    //====== INERTIAL MEASUREMENT UNIT (IMU) =====
    protected BNO055IMU imu    = null;
    public double headingAngle = 0.0;
    public double tiltAngle    = 0.0;

    //====== MECANUM DRIVETRAIN MOTORS (RUN_USING_ENCODER) =====
    protected DcMotorEx frontLeftMotor     = null;
    public int          frontLeftMotorTgt  = 0;       // RUN_TO_POSITION target encoder count
    public int          frontLeftMotorPos  = 0;       // current encoder count
    public double       frontLeftMotorVel  = 0.0;     // encoder counts per second
    public double       frontLeftMotorAmps = 0.0;     // current power draw (Amps)

    protected DcMotorEx frontRightMotor    = null;
    public int          frontRightMotorTgt = 0;       // RUN_TO_POSITION target encoder count
    public int          frontRightMotorPos = 0;       // current encoder count
    public double       frontRightMotorVel = 0.0;     // encoder counts per second
    public double       frontRightMotorAmps= 0.0;     // current power draw (Amps)

    protected DcMotorEx rearLeftMotor      = null;
    public int          rearLeftMotorTgt   = 0;       // RUN_TO_POSITION target encoder count
    public int          rearLeftMotorPos   = 0;       // current encoder count
    public double       rearLeftMotorVel   = 0.0;     // encoder counts per second
    public double       rearLeftMotorAmps  = 0.0;     // current power draw (Amps)

    protected DcMotorEx rearRightMotor     = null;
    public int          rearRightMotorTgt  = 0;       // RUN_TO_POSITION target encoder count
    public int          rearRightMotorPos  = 0;       // current encoder count
    public double       rearRightMotorVel  = 0.0;     // encoder counts per second
    public double       rearRightMotorAmps = 0.0;     // current power draw (Amps)

    private long lastUpdateTime = 0;
    public boolean disableDriverCentric = true;

    public final static double MIN_SPIN_RATE = 0.07;
    public final static double MIN_DRIVE_RATE = 0.07;
    public final static double STRAFE_MULTIPLIER = 1.25;

    //====== NAVIGATION DISTANCE SENSORS =====
    private MaxSonarI2CXL sonarRangeL = null;   // Must include MaxSonarI2CXL.java in teamcode folder
    private MaxSonarI2CXL sonarRangeR = null;
    private MaxSonarI2CXL sonarRangeF = null;
    private MaxSonarI2CXL sonarRangeB = null;

    protected HardwareMap hwMap = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean isAutonomous, boolean hasLeftRange,
                     boolean hasRightRange, boolean hasFrontRange, boolean hasRearRange) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Configure REV control/expansion hubs for bulk reads (faster!)
        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            if(module.isParent()) {
                controlHub = module;
            } else {
                expansionHub = module;
            }
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Define and Initialize drivetrain motors
        frontLeftMotor  = hwMap.get(DcMotorEx.class,"FrontLeft");  // Expansion Hub port 2 (REVERSE)
        frontRightMotor = hwMap.get(DcMotorEx.class,"FrontRight"); // Expansion Hub port 0 (forward)
        rearLeftMotor   = hwMap.get(DcMotorEx.class,"RearLeft");   // Expansion Hub port 3 (REVERSE)
        rearRightMotor  = hwMap.get(DcMotorEx.class,"RearRight");  // Expansion Hub port 1 (forward)

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);  // goBilda fwd/rev opposite of Matrix motors!
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all drivetrain motors to zero power
        driveTrainMotorsZero();

        // Set all drivetrain motors to run with encoders.
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set all drivetrain motors to brake when at zero power
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Instantiate Maxbotics ultrasonic range sensors (sensors wired to I2C ports)
        if(hasLeftRange) {
            sonarRangeL = hwMap.get(MaxSonarI2CXL.class, "left_ultrasonic");
        }
        if(hasRightRange) {
            sonarRangeR = hwMap.get(MaxSonarI2CXL.class, "right_ultrasonic");
        }
        if(hasFrontRange) {
            sonarRangeF = hwMap.get(MaxSonarI2CXL.class, "front_ultrasonic");
        }
        if(hasRearRange) {
            sonarRangeB = hwMap.get(MaxSonarI2CXL.class, "back_ultrasonic");
        }
    }

    /*--------------------------------------------------------------------------------------------*/
    public void readBulkData() {
        // For MANUAL mode, we must clear the BulkCache once per control cycle
        expansionHub.clearBulkCache();
        controlHub.clearBulkCache();
        // Get a fresh set of values for this cycle
        //   getCurrentPosition() / getTargetPosition() / getTargetPositionTolerance()
        //   getPower() / getVelocity() / getCurrent()
        frontLeftMotorPos  = frontLeftMotor.getCurrentPosition();
        frontLeftMotorVel  = frontLeftMotor.getVelocity();
        frontLeftMotorAmps = frontLeftMotor.getCurrent(MILLIAMPS);
        frontRightMotorPos = frontRightMotor.getCurrentPosition();
        frontRightMotorVel = frontRightMotor.getVelocity();
        frontRightMotorAmps= frontRightMotor.getCurrent(MILLIAMPS);
        rearRightMotorPos  = rearRightMotor.getCurrentPosition();
        rearRightMotorVel  = rearRightMotor.getVelocity();
        rearRightMotorAmps = rearRightMotor.getCurrent(MILLIAMPS);
        rearLeftMotorPos   = rearLeftMotor.getCurrentPosition();
        rearLeftMotorVel   = rearLeftMotor.getVelocity();
        rearLeftMotorAmps  = rearLeftMotor.getCurrent(MILLIAMPS);
    } // readBulkData

    /**
     * @return range value in cm, -99 for no range sensor installed.
     */
    public int readLeftRangeSensor() {
        if(sonarRangeL != null) {
            return sonarRangeL.getDistanceAsync();
        }

        return -99;
    } // readLeftRangeSensor

    /**
     * @return range value in cm, -99 for no range sensor installed.
     */
    public int readRightRangeSensor() {
        if(sonarRangeR != null) {
            return sonarRangeR.getDistanceAsync();
        }

        return -99;
    } // readRightRangeSensor

    /**
     * @return range value in cm, -99 for no range sensor installed.
     */
    public int readFrontRangeSensor() {
        if(sonarRangeF != null) {
            return sonarRangeF.getDistanceAsync();
        }

        return -99;
    } // readFrontRangeSensor

    /**
     * @return range value in cm, -99 for no range sensor installed.
     */
    public int readBackRangeSensor() {
        if(sonarRangeB != null) {
            return sonarRangeB.getDistanceAsync();
        }

        return -99;
    } // readBackRangeSensor

    /*--------------------------------------------------------------------------------------------*/
    public double headingIMU()
    {
        Orientation angles = imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES );
        headingAngle = angles.firstAngle;
        tiltAngle = angles.secondAngle;
        MyPosition.worldAngle_rad = Math.toRadians(-headingAngle);
        return -headingAngle;  // degrees (+90 is CW; -90 is CCW)
    } // headingIMU

    /*--------------------------------------------------------------------------------------------*/
    public void initIMU()
    {
        // Define and initialize REV Expansion Hub IMU
        BNO055IMU.Parameters imu_params = new BNO055IMU.Parameters();
        imu_params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu_params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu_params.calibrationDataFile = "BNO055IMUCalibration.json"; // located in FIRST/settings folder
        imu_params.loggingEnabled = false;
        imu_params.loggingTag = "IMU";
        imu_params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize( imu_params );
    } // initIMU()

    /*--------------------------------------------------------------------------------------------*/
    public void driveTrainMotors( double frontLeft, double frontRight, double rearLeft, double rearRight )
    {
        frontLeftMotor.setPower( frontLeft );
        frontRightMotor.setPower( frontRight );
        rearLeftMotor.setPower( rearLeft );
        rearRightMotor.setPower( rearRight );
    } // driveTrainMotors

    /*--------------------------------------------------------------------------------------------*/
    public void driveTrainMotorsZero()
    {
        frontLeftMotor.setPower( 0.0 );
        frontRightMotor.setPower( 0.0 );
        rearLeftMotor.setPower( 0.0 );
        rearRightMotor.setPower( 0.0 );
    } // driveTrainMotorsZero

    /**
     *
     * @param targetAngle - Angle to rotate to in degrees towards
     */
    public boolean rotateTowardsAngle(double targetAngle) {
        boolean reachedDestination = false;
        double errorMultiplier = 0.016;
        double minSpinRate = MIN_SPIN_RATE;
        double deltaAngle = MyPosition.AngleWrap(Math.toRadians(targetAngle) - MyPosition.worldAngle_rad);
        double turnSpeed = -Math.toDegrees(deltaAngle) * errorMultiplier;


        // We are done if we are within 2 degrees
        if(abs(Math.toDegrees(deltaAngle)) < 0.5) {
            // We have reached our destination if the angle is close enough
            driveTrainMotorsZero();
            reachedDestination = true;
        } else {
            // We still have some turning to do.
            MovementVars.movement_x = 0;
            MovementVars.movement_y = 0;
            turnSpeed = copySign(max(minSpinRate, abs(turnSpeed)), turnSpeed);
            MovementVars.movement_turn = turnSpeed;
            ApplyMovement();
        }

        return reachedDestination;
    }

    /**
     * @param speed        - The driving power
     * @param rotateSpeed  - The rotational speed to correct heading errors
     * @param driveAngle   - The angle of movement to drive the robot
     * @param headingAngle - The heading angle to hold while driving
     */
    public void driveAtHeading(double speed, double rotateSpeed, double driveAngle, double headingAngle) {
        double xPower = 0.0;
        double yPower = 0.0;
        double deltaAngle = 0.0;
        final double SAME_ANGLE = 1;
        MyPosition.worldAngle_rad = Math.toRadians(headingIMU());
        deltaAngle = MyPosition.AngleWrap(Math.toRadians(headingAngle) - MyPosition.worldAngle_rad);

        if (Math.abs(deltaAngle) > SAME_ANGLE) {
            if (deltaAngle > 0.0) {
                rotateSpeed = -rotateSpeed;
            }
        } else {
            rotateSpeed = 0.0;
        }

        xPower = speed * Math.cos(Math.toRadians(driveAngle));
        yPower = speed * Math.sin(Math.toRadians(driveAngle));
        drive(xPower, yPower, rotateSpeed, 0.0, false);
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
            gyroAngle += headingIMU();
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

    /**converts movement_y, movement_x, movement_turn into motor powers */
    public void ApplyMovement() {
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16){
            return;
        }
        lastUpdateTime = currTime;

        double tl_power_raw = MovementVars.movement_y-MovementVars.movement_turn+MovementVars.movement_x*STRAFE_MULTIPLIER;
        double bl_power_raw = MovementVars.movement_y-MovementVars.movement_turn-MovementVars.movement_x*STRAFE_MULTIPLIER;
        double br_power_raw = -MovementVars.movement_y-MovementVars.movement_turn-MovementVars.movement_x*STRAFE_MULTIPLIER;
        double tr_power_raw = -MovementVars.movement_y-MovementVars.movement_turn+MovementVars.movement_x*STRAFE_MULTIPLIER;

        //find the maximum of the powers
        double maxRawPower = abs(tl_power_raw);
        if(abs(bl_power_raw) > maxRawPower){ maxRawPower = abs(bl_power_raw);}
        if(abs(br_power_raw) > maxRawPower){ maxRawPower = abs(br_power_raw);}
        if(abs(tr_power_raw) > maxRawPower){ maxRawPower = abs(tr_power_raw);}

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
        driveTrainMotors( tl_power_raw, tr_power_raw, bl_power_raw, br_power_raw );
    }

    protected double driverInputSpinShaping( double valueIn, boolean inputShaping) {
        double aValue = 0.77;
        double valueOut;

        if(valueIn == 0.0) {
            valueOut = 0.0;
        } else {
            if (inputShaping) {
                valueOut = aValue * Math.pow(valueIn, 3) + (1 - aValue) * valueIn;
                valueOut = Math.copySign(max(MIN_SPIN_RATE, abs(valueOut)), valueOut);
            } else {
                valueOut = valueIn;
            }
        }

        return valueOut;
    }

    protected double driverInputShaping( double valueIn, boolean inputShaping) {
        double aValue = 0.77;
        double valueOut = 0.0;

        if(valueIn == 0.0) {
            valueOut = 0.0;
        } else {
            if (inputShaping) {
                valueOut = aValue * Math.pow(valueIn, 3) + (1 - aValue) * valueIn;
                valueOut = Math.copySign(max(MIN_DRIVE_RATE, abs(valueOut)), valueOut);
            } else {
                valueOut = valueIn;
            }
        }

        return valueOut;
    }
}
