
package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import static java.lang.Thread.sleep;

/** Mecanum drivetrain subsystem */
public class Drive extends Subsystem {
    private HardwareMap hardwareMap;
    private LinearOpMode opMode;
    //DC Motors
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;

    // use motor encoder for odometry
    public DcMotorEx odL;
    public DcMotorEx odB;
    public DcMotorEx odR;

    //Sensors
    private BNO055IMU imu;

    private double robotCurrentPosX;    // unit in mm
    private double robotCurrentPosY;    // unit in mm
    private double robotCurrentAngle;   // unit in degrees

    private int encoderOffsetFL = 0;
    private int encoderOffsetFR = 0;
    private int encoderOffsetRL = 0;
    private int encoderOffsetRR = 0;

    private int odometryCountOffsetL = 0;
    private int odometryCountOffsetR = 0;
    private int odometryCountOffsetB = 0;
    private int odometryCountL = 0;
    private int odometryCountR = 0;
    private int odometryCountB = 0;
    private static final double ODOMETRY_mm_PER_COUNT               = 38.85*3.14159265/8192.0;
    private static final double ODOMETRY_RADIUS_X                   = 201.0;
    private static final double ODOMETRY_RADIUS_Y                   = 178.0;

    //DO WITH ENCODERS
    private static final double     DRIVE_GEAR_REDUCTION            = 1.0 ;     // This is < 1.0 if geared UP

    private static final double     TICKS_PER_MOTOR_REV_20          = 537.6;    // AM Orbital 20 motor
    private static final double     RPM_MAX_NEVERREST_20            = 340;
    private static final double     ANGULAR_V_MAX_NEVERREST_20      = (TICKS_PER_MOTOR_REV_20 * RPM_MAX_NEVERREST_20) / 60.0;

    //NEW Chassis
    private static final double     MOTOR_TICK_PER_REV_YELLOJACKET312   = 537.6;
    private static final double     GOBUILDA_MECANUM_DIAMETER_MM        = 96.0;
    private static final double     COUNTS_PER_MM                       = (MOTOR_TICK_PER_REV_YELLOJACKET312 * DRIVE_GEAR_REDUCTION) / (GOBUILDA_MECANUM_DIAMETER_MM * Math.PI);


    private static final double     WHEEL_DIAMETER_INCHES           = 100.0/25.4 ;     // For figuring circumference
    private static final double     WHEEL_DIAMETER_MM               = 100.0;
    private static final double     COUNTS_PER_INCH                 = (TICKS_PER_MOTOR_REV_20 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double     COUNTS_CORRECTION_X             = 1.167;
    private static final double     COUNTS_CORRECTION_Y             = 0.9918;
    private static final double     COUNTS_PER_DEGREE             = 10.833*0.99;     // 975 ticks per 90 degrees

    private static final double     DRIVE_SPEED             = 0.40;
    private static final double     DRIVE_SPEED_X             = 0.35;
    private static final double     DRIVE_SPEED_Y             = 0.40;
    private static final double     TURN_SPEED              = 0.40;
    private static boolean          driveFullPower          = false;
    private static double           motorKp                 = 0.015;
    private static double           motorKi                 = 0.02;
    private static double           motorKd                 = 0.0003;
    private static double           motorRampTime           = 0.3;

    private static final double     ROBOT_INIT_POS_X    = 15.0;
    private static final double     ROBOT_INIT_POS_Y    = 15.0;
    private static final double     ROBOT_INIT_ANGLE    = 45.0;
    private static final float      mmPerInch        = 25.4f;

    private boolean isBlue = false;


    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    private long startTime;

    public Drive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx rearLeft, DcMotorEx rearRight, DcMotorEx odL, DcMotorEx odB, DcMotorEx odR, BNO055IMU imu, LinearOpMode opMode, ElapsedTime timer) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
        this.odL = odL;
        this.odB = odB;
        this.odR = odR;
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.imu = imu;
        this.timer = timer;
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private int getOdometryCountL() {
        odometryCountL = -odL.getCurrentPosition() - odometryCountOffsetL;
        return odometryCountL;
    }

    private int getOdometryCountB() {
        odometryCountB = odB.getCurrentPosition() - odometryCountOffsetB;
        return odometryCountB;
    }

    private int getOdometryCountR() {
        odometryCountR = odR.getCurrentPosition() - odometryCountOffsetR;
        return odometryCountR;
    }

    private void resetOdometry() {
        odometryCountOffsetL = -odL.getCurrentPosition();
        odometryCountOffsetB = odB.getCurrentPosition();
        odometryCountOffsetR = odR.getCurrentPosition();
    }

    private void updateOdometry() {
        getOdometryCountL();
        getOdometryCountB();
        getOdometryCountR();
    }

    public double getAngularVMaxNeverrest20(){
        return ANGULAR_V_MAX_NEVERREST_20;
    }

    /**
     * Stops all drive motors
     */


    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    public void checkAndStopMotors() {
        if (!frontLeft.isBusy()) { frontLeft.setPower(0); }
        if (!frontRight.isBusy()) { frontRight.setPower(0); }
        if (!rearLeft.isBusy()) { rearLeft.setPower(0); }
        if (!rearRight.isBusy()) { rearRight.setPower(0); }
    }

    /**
     * Sets all drive motors to specified run mode
     */
    public void setRunMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        rearLeft.setMode(mode);
        rearRight.setMode(mode);
    }

    /**
     * Initialize MaxVelocity of drive motors
     */
    public void initMaxVelocity() {
        frontLeft.setVelocity(ANGULAR_V_MAX_NEVERREST_20);
        frontRight.setVelocity(ANGULAR_V_MAX_NEVERREST_20);
        rearLeft.setVelocity(ANGULAR_V_MAX_NEVERREST_20);
        rearRight.setVelocity(ANGULAR_V_MAX_NEVERREST_20);
    }

    /**
     * Sets all drive motors to specified zero power behavior
     */
    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        frontLeft.setZeroPowerBehavior(mode);
        frontRight.setZeroPowerBehavior(mode);
        rearLeft.setZeroPowerBehavior(mode);
        rearRight.setZeroPowerBehavior(mode);
    }


    public void turn(double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        rearLeft.setPower(-power);
        rearRight.setPower(power);
    }

    // robot move in all directions
    public double[] calcMotorPowers(double leftStickX, double leftStickY, double rightStickX) {
        double r = Math.hypot(leftStickX, leftStickY);
        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
        double lrPower = r * Math.sin(robotAngle) + rightStickX;
        double lfPower = r * Math.cos(robotAngle) + rightStickX;
        double rrPower = r * Math.cos(robotAngle) - rightStickX;
        double rfPower = r * Math.sin(robotAngle) - rightStickX;
        return new double[]{lfPower, rfPower, lrPower, rrPower};
    }

    // robot only move in forward/backward/left/right directions
    public double[] calcMotorPowers2(double leftStickX, double leftStickY, double rightStickX) {
        if(Math.abs(leftStickX) >= Math.abs((leftStickY))){
            leftStickY = 0;
        }
        else{
            leftStickX = 0;
        }
        double r = Math.hypot(leftStickX, leftStickY);
        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
        double lrPower = r * Math.sin(robotAngle) + rightStickX;
        double lfPower = r * Math.cos(robotAngle) + rightStickX;
        double rrPower = r * Math.cos(robotAngle) - rightStickX;
        double rfPower = r * Math.sin(robotAngle) - rightStickX;
        return new double[]{lfPower, rfPower, lrPower, rrPower};
    }

    public void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        rearLeft.setPower(power);
        rearRight.setPower(power);
    }

    public void setDrivePowers(double[] powers) {
        frontLeft.setPower(powers[0]);
        frontRight.setPower(powers[1]);
        rearLeft.setPower(powers[2]);
        rearRight.setPower(powers[3]);
    }

    public void setDriveFullPower(boolean fullPower) {
        driveFullPower = fullPower;
    }

    public void setTargetPosition(int targetPosition) {
        frontLeft.setTargetPosition(targetPosition);
        frontRight.setTargetPosition(targetPosition);
        rearLeft.setTargetPosition(targetPosition);
        rearRight.setTargetPosition(targetPosition);
    }

    public int[] getCurrentPositions() {
        return new int[] {
                frontLeft.getCurrentPosition() - encoderOffsetFL,
                frontRight.getCurrentPosition() - encoderOffsetFR,
                rearLeft.getCurrentPosition() - encoderOffsetRL,
                rearRight.getCurrentPosition() - encoderOffsetRR
        };
    }

    public int[] getDriveMotorEncoders() {
        return new int[] {
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                rearLeft.getCurrentPosition(),
                rearRight.getCurrentPosition()
        };
    }

    public void resetDriveMotorEncoders() {
        encoderOffsetFL = frontLeft.getCurrentPosition();
        encoderOffsetFR = frontRight.getCurrentPosition();
        encoderOffsetRL = rearLeft.getCurrentPosition();
        encoderOffsetRR = rearRight.getCurrentPosition();
    }

    /**
     * Positive encoder values correspond to rightward robot movement
     */
    public void strafe(int targetPosition) {
        frontLeft.setTargetPosition(targetPosition);
        frontRight.setTargetPosition(-targetPosition);
        rearLeft.setTargetPosition(-targetPosition);
        rearRight.setTargetPosition(targetPosition);
    }

    public void turnRobotByTick(double angle) {
//        this.turnByTick(TURN_SPEED, angle);
        if (angle > 0.0) {
            allMotorPIDControl((int) (angle*COUNTS_PER_DEGREE), TURN_SPEED * ANGULAR_V_MAX_NEVERREST_20, ANGULAR_V_MAX_NEVERREST_20,
                    motorRampTime, false, true, false, true, motorKp, motorKi, motorKd);
        }
        else {
            allMotorPIDControl((int) (-angle*COUNTS_PER_DEGREE), TURN_SPEED * ANGULAR_V_MAX_NEVERREST_20, ANGULAR_V_MAX_NEVERREST_20,
                    motorRampTime, true, false, true, false, motorKp, motorKi, motorKd);
        }
//        robotCurrentPosX += ROBOT_HALF_LENGTH * (Math.cos((robotCurrentAngle+degrees)*Math.PI/180.0)
//                - Math.cos(robotCurrentAngle*Math.PI/180.0));
//        robotCurrentPosY += ROBOT_HALF_LENGTH * (Math.sin((robotCurrentAngle+degrees)*Math.PI/180.0)
//                - Math.sin(robotCurrentAngle*Math.PI/180.0));
        robotCurrentAngle += angle;
        // Display it for the driver.
        opMode.telemetry.addData("turnRobot",  "turn to %7.2f degrees", robotCurrentAngle);
        opMode.telemetry.update();
//        opMode.sleep(100);
    }

    public void turnByTick(double power, double angle) {
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);
        rearLeft.setTargetPosition(0);
        rearRight.setTargetPosition(0);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (driveFullPower) {
            setDrivePower(1.0);
        }
        else {
            setDrivePower(power);
        }
        // convert from degrees to motor counts
        int tickCount = (int) (angle * COUNTS_PER_DEGREE);
        frontLeft.setTargetPosition(-tickCount);
        frontRight.setTargetPosition(tickCount);
        rearLeft.setTargetPosition(-tickCount);
        rearRight.setTargetPosition(tickCount);
        startTime = timer.nanoseconds();
        while (frontLeft.isBusy() && frontRight.isBusy() && rearLeft.isBusy() && rearRight.isBusy()) {
            logDriveEncoders();
            checkAndStopMotors();
        }
        stop();
        logDriveEncoders();
    }

    public void turnRobot(double degrees) {
        this.turnByAngle(TURN_SPEED, degrees);
//        robotCurrentPosX += ROBOT_HALF_LENGTH * (Math.cos((robotCurrentAngle+degrees)*Math.PI/180.0)
//                - Math.cos(robotCurrentAngle*Math.PI/180.0));
//        robotCurrentPosY += ROBOT_HALF_LENGTH * (Math.sin((robotCurrentAngle+degrees)*Math.PI/180.0)
//                - Math.sin(robotCurrentAngle*Math.PI/180.0));
        robotCurrentAngle += degrees;
        // Display it for the driver.
        opMode.telemetry.addData("turnRobot",  "turn to %7.2f degrees", robotCurrentAngle);
        opMode.telemetry.update();
//        opMode.sleep(100);
    }

    public double getYaw() {
        return imu.getAngularOrientation().firstAngle;
    }

    public void turnByAngle(double power, double turnAngle) {
        double initialAngle = getYaw();
        double currentAngle;
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (turnAngle > 0.0) {
            // counter-clockwise
            currentAngle = initialAngle;
            while (Math.abs(currentAngle - initialAngle - turnAngle) > 2) {
                turn(-power);
                currentAngle = getYaw();
                if (currentAngle < initialAngle) {
                    // angle wraparound
                    currentAngle += 360.0;
                }
            }
        } else {
            // clockwise
            currentAngle = initialAngle;
            while (Math.abs(currentAngle - initialAngle - turnAngle) > 2) {
                turn(power);
                currentAngle = getYaw();
                if (currentAngle > initialAngle) {
                    // angle wraparound
                    currentAngle -= 360.0;
                }
            }
        }
        stop();
        opMode.telemetry.addData("initial angle",  "%7.2f degrees", initialAngle);
        opMode.telemetry.addData("last read angle",  "%7.2f degrees", currentAngle);
        opMode.telemetry.addData("final angle",  "%7.2f degrees", getYaw());
        opMode.sleep(3000);
        opMode.telemetry.addData("final2 angle",  "%7.2f degrees", getYaw());
        opMode.telemetry.update();
        opMode.sleep(3000);
    }

    public void moveToPos2D(double power, double targetPositionX, double targetPositionY){
        // move to X, Y position relative to the robot coordinate system
        // the center of robot is 0,0
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);
        rearLeft.setTargetPosition(0);
        rearRight.setTargetPosition(0);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        // convert from inches to motor counts
        // correct for X and Y motion asymmetry
        double distanceCountX, distanceCountY;
        distanceCountX = targetPositionX * COUNTS_PER_MM * COUNTS_CORRECTION_X;
        distanceCountY = targetPositionY * COUNTS_PER_MM * COUNTS_CORRECTION_Y;
        if (driveFullPower) {
            setPower2D(distanceCountX, distanceCountY, 1.0);
        }
        else {
            setPower2D(distanceCountX, distanceCountY, power);
        }
        setTargetPosition2D(distanceCountX, distanceCountY);
        startTime = timer.nanoseconds();
        while (frontLeft.isBusy() && frontRight.isBusy() && rearLeft.isBusy() && rearRight.isBusy()) {
            logDriveEncoders();
            checkAndStopMotors();
        }
        stop();
        logDriveEncoders();
    }

    public void setPower2D(double targetPositionX, double targetPositionY, double motorPower) {
        // distribute power appropriately according to the direction of motion
        double[] motorPowers = calcMotorPowers2D(targetPositionX, targetPositionY, motorPower);
        rearLeft.setPower(motorPowers[0]);
        frontLeft.setPower(motorPowers[1]);
        rearRight.setPower(motorPowers[2]);
        frontRight.setPower(motorPowers[3]);
    }

    public void setTargetPosition2D(double targetPositionX, double targetPositionY) {
        // set motor rotation targets appropriately according to the direction of motion
//        frontLeft.setTargetPosition((int)  ((+ targetPositionX + targetPositionY)*Math.sqrt(2.0)));
//        frontRight.setTargetPosition((int) ((- targetPositionX + targetPositionY)*Math.sqrt(2.0)));
//        rearLeft.setTargetPosition((int)   ((- targetPositionX + targetPositionY)*Math.sqrt(2.0)));
//        rearRight.setTargetPosition((int)  ((+ targetPositionX + targetPositionY)*Math.sqrt(2.0)));
        frontLeft.setTargetPosition((int)  (+ targetPositionX + targetPositionY));
        frontRight.setTargetPosition((int) (- targetPositionX + targetPositionY));
        rearLeft.setTargetPosition((int)   (- targetPositionX + targetPositionY));
        rearRight.setTargetPosition((int)  (+ targetPositionX + targetPositionY));
    }

    public double[] calcMotorPowers2D(double targetPositionX, double targetPositionY, double motorPower)
    {
        // targetPositionX and targetPositionY determine the direction of movement
        // motorPower determines the magnitude of motor power
        double angleScale = Math.abs(targetPositionX) + Math.abs(targetPositionY);
        double lrPower = motorPower * (- targetPositionX + targetPositionY) / angleScale;
        double lfPower = motorPower * (+ targetPositionX + targetPositionY) / angleScale;
        double rrPower = motorPower * (+ targetPositionX + targetPositionY) / angleScale;
        double rfPower = motorPower * (- targetPositionX + targetPositionY) / angleScale;
        return new double[]{lrPower, lfPower, rrPower, rfPower};
    }

    public void moveToPosABS(double targetPositionX, double targetPositionY) {
        // move to (targetPositionX, targetPositionY) in absolute field coordinate
        double  deltaX = targetPositionX - robotCurrentPosX;    // in absolute field coordinate
        double  deltaY = targetPositionY - robotCurrentPosY;    // in absolute field coordinate
        double  distanceCountX, distanceCountY;  // distance in motor count in robot coordinate
        // rotate vector from field coordinate to robot coordinate
        distanceCountX = deltaX * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0)
                + deltaY * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0);
        distanceCountY = deltaX * Math.cos(robotCurrentAngle*Math.PI/180.0)
                + deltaY * Math.sin(robotCurrentAngle*Math.PI/180.0);
        this.moveToPos2D(DRIVE_SPEED, distanceCountX, distanceCountY);
        robotCurrentPosX = targetPositionX;
        robotCurrentPosY = targetPositionY;
        // Display it for the driver.
        opMode.telemetry.addData("moveToPosABS",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void moveToPosREL(double targetPositionX, double targetPositionY) {
        // move to (targetPositionX, targetPositionY) in relative robot coordinate
        this.moveToPos2D(DRIVE_SPEED, targetPositionX, targetPositionY);
        robotCurrentPosX += targetPositionY * Math.cos(robotCurrentAngle*Math.PI/180.0)
                + targetPositionX * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0);
        robotCurrentPosY += targetPositionY * Math.sin(robotCurrentAngle*Math.PI/180.0)
                + targetPositionX * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveToPosREL",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void moveForward_odometry(double distance) throws InterruptedException {
        moveForward_odometry(distance, DRIVE_SPEED_Y);
    }

    public void moveForward_odometry(double distance, double motorSpeed) throws InterruptedException {
        resetOdometry();
//        this.moveToPos2D(motorSpeed, 0.0, distance);
        allMotorPIDControl( (int) (distance*COUNTS_PER_MM * COUNTS_CORRECTION_Y), motorSpeed * ANGULAR_V_MAX_NEVERREST_20, ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime, true, true, true, true, motorKp, motorKi, motorKd);
        robotCurrentPosX += distance * Math.cos(robotCurrentAngle*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin(robotCurrentAngle*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveForward",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        updateOdometry();
        opMode.telemetry.addData("odometry",  " L %7.2f R %7.2f B %7.2f", odometryCountL*ODOMETRY_mm_PER_COUNT, odometryCountR*ODOMETRY_mm_PER_COUNT, odometryCountB*ODOMETRY_mm_PER_COUNT);
        opMode.telemetry.update();
//        sleep(1000);
        double angleError = (((double) odometryCountR) - ((double) odometryCountL))*0.5*ODOMETRY_mm_PER_COUNT*(180.0/3.14159265)/ODOMETRY_RADIUS_X;
        turnRobotByTick(-angleError);
        updateOdometry();
        opMode.telemetry.addData("correction angle",  " %7.2f", -angleError);
        opMode.telemetry.addData("odometry",  " L %7.2f R %7.2f B %7.2f", odometryCountL*ODOMETRY_mm_PER_COUNT, odometryCountR*ODOMETRY_mm_PER_COUNT, odometryCountB*ODOMETRY_mm_PER_COUNT);
        opMode.telemetry.update();
        if (odometryCountB*ODOMETRY_mm_PER_COUNT > 25.0) {
            moveLeft(odometryCountB*ODOMETRY_mm_PER_COUNT);
        }
        if (odometryCountB*ODOMETRY_mm_PER_COUNT < -25.0) {
            moveRight(-odometryCountB*ODOMETRY_mm_PER_COUNT);
        }
//        sleep(1000);
//        sleep(100);
    }

    public void moveForward(double distance) {
        moveForward(distance, DRIVE_SPEED_Y);
    }

    public void moveForward(double distance, double motorSpeed) {
        //        this.moveToPos2D(motorSpeed, 0.0, distance);
        allMotorPIDControl( (int) (distance*COUNTS_PER_MM * COUNTS_CORRECTION_Y), motorSpeed * ANGULAR_V_MAX_NEVERREST_20, ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime, true, true, true, true, motorKp, motorKi, motorKd);
        robotCurrentPosX += distance * Math.cos(robotCurrentAngle*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin(robotCurrentAngle*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveForward",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void moveBackward_odometry(double distance) throws InterruptedException {
        moveBackward_odometry(distance, DRIVE_SPEED_Y);
    }

    public void moveBackward_odometry(double distance, double motorSpeed) throws InterruptedException {
        resetOdometry();
//        this.moveToPos2D(motorSpeed, 0.0, -distance);
        allMotorPIDControl((int) (distance*COUNTS_PER_MM * COUNTS_CORRECTION_Y), motorSpeed * ANGULAR_V_MAX_NEVERREST_20, ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime, false, false, false, false, motorKp, motorKi, motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+180.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+180.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveBackward",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        updateOdometry();
        opMode.telemetry.addData("odometry",  " L %7.2f R %7.2f B %7.2f", odometryCountL*ODOMETRY_mm_PER_COUNT, odometryCountR*ODOMETRY_mm_PER_COUNT, odometryCountB*ODOMETRY_mm_PER_COUNT);
        opMode.telemetry.update();
//        sleep(1000);
        double angleError = (((double) odometryCountR) - ((double) odometryCountL))*0.5*ODOMETRY_mm_PER_COUNT*(180.0/3.14159265)/ODOMETRY_RADIUS_X;
        turnRobotByTick(-angleError);
        updateOdometry();
        opMode.telemetry.addData("correction angle",  " %7.2f", -angleError);
        opMode.telemetry.addData("odometry",  " L %7.2f R %7.2f B %7.2f", odometryCountL*ODOMETRY_mm_PER_COUNT, odometryCountR*ODOMETRY_mm_PER_COUNT, odometryCountB*ODOMETRY_mm_PER_COUNT);
        opMode.telemetry.update();
        if (odometryCountB*ODOMETRY_mm_PER_COUNT > 25.0) {
            moveLeft(odometryCountB*ODOMETRY_mm_PER_COUNT);
        }
        if (odometryCountB*ODOMETRY_mm_PER_COUNT < -25.0) {
            moveRight(-odometryCountB*ODOMETRY_mm_PER_COUNT);
        }
//        sleep(1000);
//        sleep(100);
    }

    public void moveBackward(double distance) {
        moveBackward(distance, DRIVE_SPEED_Y);
    }

    public void moveBackward(double distance, double motorSpeed) {
//        this.moveToPos2D(motorSpeed, 0.0, -distance);
        allMotorPIDControl((int) (distance*COUNTS_PER_MM * COUNTS_CORRECTION_Y), motorSpeed * ANGULAR_V_MAX_NEVERREST_20, ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime, false, false, false, false, motorKp, motorKi, motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+180.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+180.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveBackward",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void moveLeft_odometry(double distance) throws InterruptedException {
        moveLeft_odometry(distance, DRIVE_SPEED_X);
    }

    public void moveLeft_odometry(double distance, double motorSpeed) throws InterruptedException {
        resetOdometry();
//        this.moveToPos2D(motorSpeed, -distance, 0.0);
        allMotorPIDControl((int) (distance*COUNTS_PER_MM * COUNTS_CORRECTION_X), motorSpeed * ANGULAR_V_MAX_NEVERREST_20, ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime, false, true, true, false, motorKp, motorKi, motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+90.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+90.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveLeft",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        updateOdometry();
        opMode.telemetry.addData("odometry",  " L %7.2f R %7.2f B %7.2f", odometryCountL*ODOMETRY_mm_PER_COUNT, odometryCountR*ODOMETRY_mm_PER_COUNT, odometryCountB*ODOMETRY_mm_PER_COUNT);
        opMode.telemetry.update();
//        sleep(1000);
        double angleError = (((double) odometryCountR) - ((double) odometryCountL))*0.5*ODOMETRY_mm_PER_COUNT*(180.0/3.14159265)/ODOMETRY_RADIUS_X;
        turnRobotByTick(-angleError);
        updateOdometry();
        opMode.telemetry.addData("correction angle",  " %7.2f", -angleError);
        opMode.telemetry.addData("odometry",  " L %7.2f R %7.2f B %7.2f", odometryCountL*ODOMETRY_mm_PER_COUNT, odometryCountR*ODOMETRY_mm_PER_COUNT, odometryCountB*ODOMETRY_mm_PER_COUNT);
        opMode.telemetry.update();
        double offsetY = (((double) odometryCountR) + ((double) odometryCountL))*0.5;
        if (offsetY*ODOMETRY_mm_PER_COUNT > 25.0) {
            moveBackward(offsetY*ODOMETRY_mm_PER_COUNT);
        }
        if (offsetY*ODOMETRY_mm_PER_COUNT < -25.0) {
            moveForward(-offsetY*ODOMETRY_mm_PER_COUNT);
        }
//        sleep(1000);
//        sleep(100);
    }

    public void moveLeft(double distance) {
        moveLeft(distance, DRIVE_SPEED_X);
    }

    public void moveLeft(double distance, double motorSpeed) {
//        this.moveToPos2D(motorSpeed, -distance, 0.0);
        allMotorPIDControl((int) (distance*COUNTS_PER_MM * COUNTS_CORRECTION_X), motorSpeed * ANGULAR_V_MAX_NEVERREST_20, ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime, false, true, true, false, motorKp, motorKi, motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+90.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+90.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveLeft",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void moveRight_odometry(double distance) throws InterruptedException {
        moveRight_odometry(distance, DRIVE_SPEED_X);
    }

    public void moveRight_odometry(double distance, double motorSpeed) throws InterruptedException {
        resetOdometry();
//        this.moveToPos2D(motorSpeed, distance, 0.0);
        allMotorPIDControl((int) (distance*COUNTS_PER_MM * COUNTS_CORRECTION_X), motorSpeed * ANGULAR_V_MAX_NEVERREST_20, ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime, true, false, false, true, motorKp, motorKi, motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveRight",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        updateOdometry();
        opMode.telemetry.addData("odometry",  " L %7.2f R %7.2f B %7.2f", odometryCountL*ODOMETRY_mm_PER_COUNT, odometryCountR*ODOMETRY_mm_PER_COUNT, odometryCountB*ODOMETRY_mm_PER_COUNT);
        opMode.telemetry.update();
//        sleep(1000);
        double angleError = (((double) odometryCountR) - ((double) odometryCountL))*0.5*ODOMETRY_mm_PER_COUNT*(180.0/3.14159265)/ODOMETRY_RADIUS_X;
        turnRobotByTick(-angleError);
        updateOdometry();
        opMode.telemetry.addData("correction angle",  " %7.2f", -angleError);
        opMode.telemetry.addData("odometry",  " L %7.2f R %7.2f B %7.2f", odometryCountL*ODOMETRY_mm_PER_COUNT, odometryCountR*ODOMETRY_mm_PER_COUNT, odometryCountB*ODOMETRY_mm_PER_COUNT);
        opMode.telemetry.update();
        double offsetY = (((double) odometryCountR) + ((double) odometryCountL))*0.5;
        if (offsetY*ODOMETRY_mm_PER_COUNT > 25.0) {
            moveBackward(offsetY*ODOMETRY_mm_PER_COUNT);
        }
        if (offsetY*ODOMETRY_mm_PER_COUNT < -25.0) {
            moveForward(-offsetY*ODOMETRY_mm_PER_COUNT);
        }
//        sleep(1000);
//        sleep(100);
    }

    public void moveRight(double distance) {
        moveRight(distance, DRIVE_SPEED_X);
    }

    public void moveRight(double distance, double motorSpeed) {
//        this.moveToPos2D(motorSpeed, distance, 0.0);
        allMotorPIDControl((int) (distance*COUNTS_PER_MM * COUNTS_CORRECTION_X), motorSpeed * ANGULAR_V_MAX_NEVERREST_20, ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime, true, false, false, true, motorKp, motorKi, motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveRight",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void printMotorPIDCoefficients() {
        PIDFCoefficients pidCoeff;
        pidCoeff = getMotorPIDCoefficients(frontLeft, DcMotor.RunMode.RUN_TO_POSITION);
        opMode.telemetry.addData("Front Left ", "P: %.2f I: %.2f D: %.2f F: %.2f A: %s",
                pidCoeff.p, pidCoeff.i, pidCoeff.d, pidCoeff.f, pidCoeff.algorithm.toString());
        pidCoeff = getMotorPIDCoefficients(frontRight, DcMotor.RunMode.RUN_TO_POSITION);
        opMode.telemetry.addData("Front Right", "P: %.2f I: %.2f D: %.2f F: %.2f A: %s",
                pidCoeff.p, pidCoeff.i, pidCoeff.d, pidCoeff.f, pidCoeff.algorithm.toString());
        pidCoeff = getMotorPIDCoefficients(rearLeft, DcMotor.RunMode.RUN_TO_POSITION);
        opMode.telemetry.addData("Rear Left  ", "P: %.2f I: %.2f D: %.2f F: %.2f A: %s",
                pidCoeff.p, pidCoeff.i, pidCoeff.d, pidCoeff.f, pidCoeff.algorithm.toString());
        pidCoeff = getMotorPIDCoefficients(rearRight, DcMotor.RunMode.RUN_TO_POSITION);
        opMode.telemetry.addData("Rear Right ", "P: %.2f I: %.2f D: %.2f F: %.2f A: %s",
                pidCoeff.p, pidCoeff.i, pidCoeff.d, pidCoeff.f, pidCoeff.algorithm.toString());
        opMode.telemetry.update();
    }

    public void setMotorKp(double motorKPFL, double motorKPFR, double motorKPRL, double motorKPRR) {
        frontLeft.setPositionPIDFCoefficients(motorKPFL);
        frontRight.setPositionPIDFCoefficients(motorKPFR);
        rearLeft.setPositionPIDFCoefficients(motorKPRL);
        rearRight.setPositionPIDFCoefficients(motorKPRR);
    }

    public void setMotorPID(double Kp, double Ki, double Kd, double Kf) {
        PIDFCoefficients pidCoeff = new PIDFCoefficients();
        pidCoeff.p = Kp;
        pidCoeff.i = Ki;
        pidCoeff.d = Kd;
        pidCoeff.f = Kf;
        pidCoeff.algorithm = MotorControlAlgorithm.PIDF;
        setMotorPIDCoefficients(frontLeft, DcMotor.RunMode.RUN_TO_POSITION, pidCoeff);
        setMotorPIDCoefficients(frontRight, DcMotor.RunMode.RUN_TO_POSITION, pidCoeff);
        setMotorPIDCoefficients(rearLeft, DcMotor.RunMode.RUN_TO_POSITION, pidCoeff);
        setMotorPIDCoefficients(rearRight, DcMotor.RunMode.RUN_TO_POSITION, pidCoeff);
    }

    public PIDFCoefficients getMotorPIDCoefficients(DcMotorEx motor, DcMotor.RunMode mode) {
        // get a reference to the motor controller and cast it as an extended functionality controller.
        // we assume it's a REV Robotics Expansion Hub (which supports the extended controller functions).
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx) motor.getController();

        // get the port number of our configured motor.
        int motorIndex = motor.getPortNumber();

        // get the PID coefficients for the specific motor mode.
        PIDFCoefficients pidOrig = motorControllerEx.getPIDFCoefficients(motorIndex, mode);

        return pidOrig;
    }

    public void setMotorPIDCoefficients(DcMotorEx motor, DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) {
        // get a reference to the motor controller and cast it as an extended functionality controller.
        // we assume it's a REV Robotics Expansion Hub (which supports the extended controller functions).
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx) motor.getController();

        // get the port number of our configured motor.
        int motorIndex = motor.getPortNumber();

        // get the PID coefficients for the specific motor mode.
        motorControllerEx.setPIDFCoefficients(motorIndex, mode, pidfCoefficients);
    }

    public void logDriveEncoders() {
        int currentCountFL = frontLeft.getCurrentPosition();
        double currentTimeFL = ((double) (timer.nanoseconds() - startTime)) * 1.0e-6;
        int currentCountFR = frontRight.getCurrentPosition();
        double currentTimeFR = ((double) (timer.nanoseconds() - startTime)) * 1.0e-6;
        int currentCountRL = rearLeft.getCurrentPosition();
        double currentTimeRL = ((double) (timer.nanoseconds() - startTime)) * 1.0e-6;
        int currentCountRR = rearRight.getCurrentPosition();
        double currentTimeRR = ((double) (timer.nanoseconds() - startTime)) * 1.0e-6;
        String output = String.format("FL %.3f, %d, FR %.3f %d, RL %.3f %d, RR %.3f %d",
                currentTimeFL, currentCountFL, currentTimeFR, currentCountFR, currentTimeRL, currentCountRL, currentTimeRR, currentCountRR);
        Log.d("motorEnc", output);
    }

    /**
     * PID motor control program to ensure all four motors are synchronized
     * @param tickCount: absolute value of target tickcount of motor
     * @param peakSpeed: peak speed of motor rotation in tick per second
     * @param maxSpeed: max speed of motor rotation in tick per second
     * @param rampTime: motor speed ramp up/down time in sec
     * @param motorFLForward: front left motor is forward
     * @param motorFRForward: front right motor is forward
     * @param motorRLForward: rear left motor is forward
     * @param motorRRForward: rear right motor is forward
     * @param Kp: coefficient Kp
     * @param Ki: coefficient Ki
     * @param Kd: coefficient Kd
     *          by Andrew Chiang on 1/28/2020
     */
    public void allMotorPIDControl(int tickCount, double peakSpeed, double maxSpeed, double rampTime,
                                   boolean motorFLForward, boolean motorFRForward, boolean motorRLForward, boolean motorRRForward,
                                   double Kp, double Ki, double Kd) {
        stop();
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean isMotorFLDone = false;
        boolean isMotorFRDone = false;
        boolean isMotorRLDone = false;
        boolean isMotorRRDone = false;
        boolean isMotorFLNotMoving = false;
        boolean isMotorFRNotMoving = false;
        boolean isMotorRLNotMoving = false;
        boolean isMotorRRNotMoving = false;
        boolean isTimeOutStarted = false;
        boolean isTimeOutExceeded = false;
        double timeOutPeriod = 0.1;         // program will time out if the motors got stuck for more than 0.1 second
        double timeOutStartedTime = 0.0;
        int    timeOutThreshold = 3;        // motor is considered to be stuck if the motor count does not change more than 2 ticks
        double acculErrorFL = 0.0;
        double acculErrorFR = 0.0;
        double acculErrorRL = 0.0;
        double acculErrorRR = 0.0;
        double prevErrorFL = 0.0;
        double prevErrorFR = 0.0;
        double prevErrorRL = 0.0;
        double prevErrorRR = 0.0;
        double prevTimeFL = 0.0;
        double prevTimeFR = 0.0;
        double prevTimeRL = 0.0;
        double prevTimeRR = 0.0;
        boolean initialized = false;        // disable Ki and Kd terms in first iteration
        int currentCount, targetCount;
        int prevCountFL = 0;
        int prevCountFR = 0;
        int prevCountRL = 0;
        int prevCountRR = 0;
        double currentError = 0.0;
        double currentTargetSpeed;
        double currentPower = 0.0;
        double alpha = 0.95;
        double startTime = ((double) timer.nanoseconds()) * 1.0e-9;
        double currentTime = 0.0;
        double errorSlope = 0.0;
        while (((!isMotorFLDone) || (!isMotorFRDone) || (!isMotorRLDone) || (!isMotorRRDone)) && (!isTimeOutExceeded)){
            if (!isMotorFLDone) {
                currentCount = frontLeft.getCurrentPosition();                          // get current motor tick
                currentTime = ((double) timer.nanoseconds()) * 1.0e-9 - startTime;      // get current time
                targetCount = getTargetTickCount(tickCount, peakSpeed, rampTime, currentTime);  // get integrated target tick on the speed profile
                currentTargetSpeed = getTargetSpeed(tickCount, peakSpeed, rampTime, currentTime); // get the target speed on the speed profile
                if (initialized) {  // check if the motor is rotating
                    if (Math.abs(currentCount - prevCountFL) < timeOutThreshold) {
                        isMotorFLNotMoving = true;
                    }
                    else {
                        isMotorFLNotMoving = false;
                    }
                }
                if (motorFLForward) { // tick count increasing
                    if (currentCount >= tickCount) {
                        isMotorFLDone = true;
                        isMotorFLNotMoving = true;
                        frontLeft.setPower(0.0);
                    }
                    else {
                        currentError = (double) (currentCount-targetCount);
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorFL = acculErrorFL*alpha + currentError*(currentTime-prevTimeFL);  // integrate error
                            errorSlope = (currentError - prevErrorFL)/(currentTime-prevTimeFL);         // error slope
                            currentPower = currentTargetSpeed/maxSpeed - currentError*Kp - acculErrorFL*Ki - errorSlope*Kd; // apply PID correction
                        }
                        else { // at the first point, use Kp only
                            currentPower = currentTargetSpeed/maxSpeed - currentError*Kp;
                        }
                        if (currentPower > 1.0) currentPower = 1.0;
                        if (currentPower < 0.0) currentPower = 0.0;
                        frontLeft.setPower(currentPower);
                    }
                }
                else { // motorFLForward is false, tick count negative and decreasing
                    if (currentCount <= -tickCount) {
                        isMotorFLDone = true;
                        isMotorFLNotMoving = true;
                        frontLeft.setPower(0.0);
                    }
                    else {
                        currentError = (double) (-currentCount-targetCount);
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorFL = acculErrorFL*alpha + currentError*(currentTime-prevTimeFL);  // integrate error
                            errorSlope = (currentError - prevErrorFL)/(currentTime-prevTimeFL);         // error slope
                            currentPower = -currentTargetSpeed/maxSpeed + currentError*Kp + acculErrorFL*Ki + errorSlope*Kd; // apply PID correction
                        }
                        else { // at the first point, use Kp only
                            currentPower = -currentTargetSpeed/maxSpeed + currentError*Kp;
                        }
                        if (currentPower < -1.0) currentPower = -1.0;
                        if (currentPower > 0.0) currentPower = 0.0;
                        frontLeft.setPower(currentPower);
                    }
                }
                prevErrorFL = currentError;
                prevTimeFL = currentTime;
                prevCountFL = currentCount;
            } // if (!isMotorFLDone)
            if (!isMotorFRDone) {
                currentCount = frontRight.getCurrentPosition();
                currentTime = ((double) timer.nanoseconds()) * 1.0e-9 - startTime;
                targetCount = getTargetTickCount(tickCount, peakSpeed, rampTime, currentTime);
                currentTargetSpeed = getTargetSpeed(tickCount, peakSpeed, rampTime, currentTime);
                if (initialized) {  // check if the motor is rotating
                    if (Math.abs(currentCount - prevCountFR) < timeOutThreshold) {
                        isMotorFRNotMoving = true;
                    }
                    else {
                        isMotorFRNotMoving = false;
                    }
                }
                if (motorFRForward) {
                    if (currentCount >= tickCount) {
                        isMotorFRDone = true;
                        isMotorFRNotMoving = true;
                        frontRight.setPower(0.0);
                    }
                    else {
                        currentError = (double) (currentCount-targetCount);
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorFR = acculErrorFR*alpha + currentError*(currentTime-prevTimeFR);  // integrate error
                            errorSlope = (currentError - prevErrorFR)/(currentTime-prevTimeFR);         // error slope
                            currentPower = currentTargetSpeed/maxSpeed - currentError*Kp - acculErrorFR*Ki - errorSlope*Kd; // apply PID correction
                        }
                        else { // at the first point, use Kp only
                            currentPower = currentTargetSpeed/maxSpeed - currentError*Kp;
                        }
                        if (currentPower > 1.0) currentPower = 1.0;
                        if (currentPower < 0.0) currentPower = 0.0;
                        frontRight.setPower(currentPower);
                    }
                }
                else { // motorFRForward is false
                    if (currentCount <= -tickCount) {
                        isMotorFRDone = true;
                        isMotorFRNotMoving = true;
                        frontRight.setPower(0.0);
                    }
                    else {
                        currentError = (double) (-currentCount-targetCount);
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorFR = acculErrorFR*alpha + currentError*(currentTime-prevTimeFR);  // integrate error
                            errorSlope = (currentError - prevErrorFR)/(currentTime-prevTimeFR);         // error slope
                            currentPower = -currentTargetSpeed/maxSpeed + currentError*Kp + acculErrorFR*Ki + errorSlope*Kd; // apply PID correction
                        }
                        else { // at the first point, use Kp only
                            currentPower = -currentTargetSpeed/maxSpeed + currentError*Kp;
                        }
                        if (currentPower < -1.0) currentPower = -1.0;
                        if (currentPower > 0.0) currentPower = 0.0;
                        frontRight.setPower(currentPower);
                    }
                }
                prevErrorFR = currentError;
                prevTimeFR = currentTime;
                prevCountFR = currentCount;
            } // if (!isMotorFRDone)
            if (!isMotorRLDone) {
                currentCount = rearLeft.getCurrentPosition();
                currentTime = ((double) timer.nanoseconds()) * 1.0e-9 - startTime;
                targetCount = getTargetTickCount(tickCount, peakSpeed, rampTime, currentTime);
                currentTargetSpeed = getTargetSpeed(tickCount, peakSpeed, rampTime, currentTime);
                if (initialized) {  // check if the motor is rotating
                    if (Math.abs(currentCount - prevCountRL) < timeOutThreshold) {
                        isMotorRLNotMoving = true;
                    }
                    else {
                        isMotorRLNotMoving = false;
                    }
                }
                if (motorRLForward) {
                    if (currentCount >= tickCount) {
                        isMotorRLDone = true;
                        isMotorRLNotMoving = true;
                        rearLeft.setPower(0.0);
                    }
                    else {
                        currentError = (double) (currentCount-targetCount);
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorRL = acculErrorRL*alpha + currentError*(currentTime-prevTimeRL);  // integrate error
                            errorSlope = (currentError - prevErrorRL)/(currentTime-prevTimeRL);         // error slope
                            currentPower = currentTargetSpeed/maxSpeed - currentError*Kp - acculErrorRL*Ki - errorSlope*Kd; // apply PID correction
                        }
                        else { // at the first point, use Kp only
                            currentPower = currentTargetSpeed/maxSpeed - currentError*Kp;
                        }
                        if (currentPower > 1.0) currentPower = 1.0;
                        if (currentPower < 0.0) currentPower = 0.0;
                        rearLeft.setPower(currentPower);
                    }
                }
                else { // motorFLForward is false
                    if (currentCount <= -tickCount) {
                        isMotorRLDone = true;
                        isMotorRLNotMoving = true;
                        rearLeft.setPower(0.0);
                    }
                    else {
                        currentError = (double) (-currentCount-targetCount);
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorRL = acculErrorRL*alpha + currentError*(currentTime-prevTimeRL);  // integrate error
                            errorSlope = (currentError - prevErrorRL)/(currentTime-prevTimeRL);         // error slope
                            currentPower = -currentTargetSpeed/maxSpeed + currentError*Kp + acculErrorRL*Ki + errorSlope*Kd; // apply PID correction
                        }
                        else { // at the first point, use Kp only
                            currentPower = -currentTargetSpeed/maxSpeed + currentError*Kp;
                        }
                        if (currentPower < -1.0) currentPower = -1.0;
                        if (currentPower > 0.0) currentPower = 0.0;
                        rearLeft.setPower(currentPower);
                    }
                }
                prevErrorRL = currentError;
                prevTimeRL = currentTime;
                prevCountRL = currentCount;
            } // if (!isMotorRLDone)
            if (!isMotorRRDone) {
                currentCount = rearRight.getCurrentPosition();
                currentTime = ((double) timer.nanoseconds()) * 1.0e-9 - startTime;
                targetCount = getTargetTickCount(tickCount, peakSpeed, rampTime, currentTime);
                currentTargetSpeed = getTargetSpeed(tickCount, peakSpeed, rampTime, currentTime);
                if (initialized) {  // check if the motor is rotating
                    if (Math.abs(currentCount - prevCountRR) < timeOutThreshold) {
                        isMotorRRNotMoving = true;
                    }
                    else {
                        isMotorRRNotMoving = false;
                    }
                }
                if (motorRRForward) {
                    currentError = (double) (currentCount-targetCount);
                    if (currentCount >= tickCount) {
                        isMotorRRDone = true;
                        isMotorRRNotMoving = true;
                        currentPower = 0.0;
                        rearRight.setPower(0.0);
                    }
                    else {
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorRR = acculErrorRR*alpha + currentError*(currentTime-prevTimeRR);  // integrate error
                            errorSlope = (currentError - prevErrorRR)/(currentTime-prevTimeRR);         // error slope
                            currentPower = currentTargetSpeed/maxSpeed - currentError*Kp - acculErrorRR*Ki - errorSlope*Kd; // apply PID correction
                        }
                        else { // at the first point, use Kp only
                            currentPower = currentTargetSpeed/maxSpeed - currentError*Kp;
                        }
                        if (currentPower > 1.0) currentPower = 1.0;
                        if (currentPower < 0.0) currentPower = 0.0;
                        rearRight.setPower(currentPower);
                    }
                }
                else { // motorFLForward is false
                    currentError = (double) (-currentCount-targetCount);
                    if (currentCount <= -tickCount) {
                        isMotorRRDone = true;
                        isMotorRRNotMoving = true;

                        currentPower = 0.0;
                        rearRight.setPower(0.0);
                    }
                    else {
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorRR = acculErrorRR*alpha + currentError*(currentTime-prevTimeRR);  // integrate error
                            errorSlope = (currentError - prevErrorRR)/(currentTime-prevTimeRR);         // error slope
                            currentPower = -currentTargetSpeed/maxSpeed + currentError*Kp + acculErrorRR*Ki + errorSlope*Kd; // apply PID correction
                        }
                        else { // at the first point, use Kp only
                            currentPower = -currentTargetSpeed/maxSpeed + currentError*Kp;
                        }
                        if (currentPower < -1.0) currentPower = -1.0;
                        if (currentPower > 0.0) currentPower = 0.0;
                        rearRight.setPower(currentPower);
                    }
                }
                prevErrorRR = currentError;
                prevTimeRR = currentTime;
                prevCountRR = currentCount;
            } // if (!isMotorRRDone)
            initialized = true;         // enable Ki and Kd terms
            if (isMotorFLNotMoving && isMotorFRNotMoving && isMotorRLNotMoving && isMotorRRNotMoving) {
                if (isTimeOutStarted) {
                    if (currentTime - timeOutStartedTime > timeOutPeriod) {
                        isTimeOutExceeded = true;
                    }
                }
                else { // time out was not started yet
                    isTimeOutStarted = true;
                    timeOutStartedTime = currentTime;
                }
            }
            else {
                isTimeOutStarted = false;
                isTimeOutExceeded = false;
            }
            String output = String.format("FL %.1f, %d, FR %.1f %d, RL %.1f %d, RR %.1f %d %.1f %.3f %.1f %.3f %s %s %s %s %s %.1f %s",
                    prevTimeFL*1000.0, prevCountFL, prevTimeFR*1000.0, prevCountFR, prevTimeRL*1000.0, prevCountRL,
                    prevTimeRR*1000.0, prevCountRR, currentError, acculErrorRR, errorSlope, currentPower,
                    isMotorFLNotMoving?"Y":"N", isMotorFRNotMoving?"Y":"N", isMotorRLNotMoving?"Y":"N", isMotorRRNotMoving?"Y":"N",
                    isTimeOutStarted?"Y":"N", timeOutStartedTime*1000.0, isTimeOutExceeded?"Y":"N");
            Log.d("motorEnc", output);
        }

    }

    private int getTargetTickCount(int tickCount, double speed, double rampTime, double elapsedTime) {
        int targetTick;
        double tickCountD = (double) tickCount;
        double speedOffset = speed * 0.15; // ramp up and ramp down with this speed offset so that there is no time the speed is close to zero
        double speedExcess= speed - speedOffset;

        if (tickCountD < rampTime*(speed + speedOffset)) {  // distance is shorter than a complete ramp up/ramp down cycle
            double halfTime = (Math.sqrt(speedOffset*speedOffset + 4.0*tickCountD*speedExcess/rampTime) - speedOffset) * rampTime * 0.5 / speedExcess;
            if (elapsedTime < halfTime) { // during ramp up time
                targetTick = (int) ((0.5*speedExcess * elapsedTime/rampTime + speedOffset) * elapsedTime);
            }
            else {  // during ramp down time
                double remainTime = halfTime + halfTime - elapsedTime;
                targetTick = tickCount - ((int) ((0.5*speedExcess * remainTime/rampTime + speedOffset) * remainTime));
            }
        }
        else { // distance is long enough to reach the cruise speed
            if (elapsedTime < rampTime) { // during ramp up time
                targetTick = (int) ((0.5*speedExcess * elapsedTime/rampTime + speedOffset) * elapsedTime);
            }
            else if (tickCountD - speedOffset*rampTime > speed * elapsedTime) { // during constant speed period
                targetTick = (int) (speed * (elapsedTime-rampTime*0.5) + 0.5*rampTime*speedOffset);
            }
            else {  // during ramp down time
                double remainTime = (tickCountD - speedOffset*rampTime)/speed + rampTime - elapsedTime;
                targetTick = tickCount - ((int) ((0.5*speedExcess * remainTime/rampTime + speedOffset) * remainTime));
            }
        }
        if (targetTick > tickCount) targetTick = tickCount;
        return targetTick;
    }

    private double getTargetSpeed(int tickCount, double speed, double rampTime, double elapsedTime) {
        double targetSpeed;
        double tickCountD = (double) tickCount;
        double speedOffset = speed * 0.15; // ramp up and ramp down with this speed offset so that there is no time the speed is close to zero
        double speedExcess= speed - speedOffset;

        if (tickCountD < rampTime*(speed + speedOffset)) {  // distance is shorter than a complete ramp up/ramp down cycle
            double halfTime = (Math.sqrt(speedOffset*speedOffset + 4.0*tickCountD*speedExcess/rampTime) - speedOffset) * rampTime * 0.5 / speedExcess;
            if (elapsedTime < halfTime) { // during ramp up time
                targetSpeed = speedExcess * elapsedTime/rampTime + speedOffset;
            }
            else {  // during ramp down time
                double remainTime = halfTime + halfTime - elapsedTime;
                targetSpeed = speedExcess * remainTime/rampTime + speedOffset;
            }
        }
        else { // distance is long enough to reach the cruise speed
            if (elapsedTime < rampTime) { // during ramp up time
                targetSpeed = speedExcess * elapsedTime/rampTime + speedOffset;
            }
            else if (tickCountD - speedOffset*rampTime > speed * elapsedTime) { // during constant speed period
                targetSpeed = speed;
            }
            else {  // during ramp down time
                double remainTime = (tickCountD - speedOffset*rampTime)/speed + rampTime - elapsedTime;
                targetSpeed = speedExcess * remainTime/rampTime + speedOffset;
            }
        }
        if (targetSpeed < speedOffset) targetSpeed = speedOffset;
        return targetSpeed;
    }

    public static class Odometry
    {
        TrcPose2D position;
        TrcPose2D velocity;

        /**
         * Constructor: Create an instance of the object.
         */
        Odometry()
        {
            position = new TrcPose2D();
            velocity = new TrcPose2D();
        }   //Odometry

        /**
         * Constructor: Create an instance of the object.
         *
         * @param position specifies the initial position.
         * @param velocity specifies the initial velocity.
         */
        Odometry(TrcPose2D position, TrcPose2D velocity)
        {
            this.position = position;
            this.velocity = velocity;
        }   //Odometry

        /**
         * This method returns the string representation of the object.
         *
         * @return string representation of the object.
         */
        @Override
        public String toString()
        {
            return "position=" + position.toString() + ", velocity=" + velocity.toString();
        }   //toString

        /**
         * This method creates and returns a copy of this odometry.
         *
         * @return a copy of this odometry.
         */
        public Odometry clone()
        {
            return new Odometry(position.clone(), velocity.clone());
        }   //clone

        /**
         * This method sets the position info of the odometry to the given pose.
         *
         * @param pose specifies the pose to set the position info to.
         */
        void setPositionAs(TrcPose2D pose)
        {
            this.position.setAs(pose);
        }   //setPositionAs

        /**
         * This method sets the velocity info of the odometry to the given pose.
         *
         * @param pose specifies the pose to set the velocity info to.
         */
        void setVelocityAs(TrcPose2D pose)
        {
            this.velocity.setAs(pose);
        }   //setVelocityAs

    }   //class Odometry
}
