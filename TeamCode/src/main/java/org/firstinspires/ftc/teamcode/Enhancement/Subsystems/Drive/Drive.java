package org.firstinspires.ftc.teamcode.Enhancement.Subsystems.Drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.Enhancement.Config.DriveConfig;
import org.firstinspires.ftc.teamcode.Enhancement.Robot;
import org.firstinspires.ftc.teamcode.Enhancement.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Util.QuickTelemetry;

import java.util.List;
import java.util.Locale;

/**
 * Mecanum drivetrain subsystem
 * <p>
 * Mostly repetitive code.
 */
public class Drive extends Subsystem {
    // Sensors
    private final BNO055IMU imu;
    /**
     * DC Motors initialization.
     */
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;
    // use motor encoder for odometry
    public DcMotorEx odL;
    public DcMotorEx odB;
    public DcMotorEx odR;
    private double robotCurrentPosX;    // unit in mm
    private double robotCurrentPosY;    // unit in mm
    private double robotCurrentAngle;   // unit in degrees


    private OpenGLMatrix lastLocation = null;

    private long startTime;

    /**
     * The class instantiation.
     *
     * @param telemetry   quick telemetry
     * @param hardwareMap the hardware map
     * @param timer       the timer
     * @param motors      given in a list for readability
     * @param imu         the imu
     * @see Robot
     * @see DcMotorEx
     * @see BNO055IMU
     */
    public Drive(QuickTelemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer, List<DcMotorEx> motors, BNO055IMU imu) {
        super(telemetry, hardwareMap, timer);
        telemetry.telemetry(3, "motors init started");
        this.frontLeft = motors.get(0);
        this.frontRight = motors.get(1);
        this.rearLeft = motors.get(2);
        this.rearRight = motors.get(3);
        telemetry.telemetry(2, "motors init finished");

        this.odL = odL;
        this.odB = odB;
        this.odR = odR;

        this.imu = imu;

        setZeroPowerBehavior();
        telemetry.telemetry(2, "Drive initialized", "Drive initialized");
    }

    private int getOdometryCountL() {
        DriveConfig.odometryCountL = -odL.getCurrentPosition() - DriveConfig.odometryCountOffsetL;
        return DriveConfig.odometryCountL;
    }

    private int getOdometryCountB() {
        DriveConfig.odometryCountB = odB.getCurrentPosition() - DriveConfig.odometryCountOffsetB;
        return DriveConfig.odometryCountB;
    }

    private int getOdometryCountR() {
        DriveConfig.odometryCountR = odR.getCurrentPosition() - DriveConfig.odometryCountOffsetR;
        return DriveConfig.odometryCountR;
    }

    private void resetOdometry() {
        DriveConfig.odometryCountOffsetL = -odL.getCurrentPosition();
        DriveConfig.odometryCountOffsetB = odB.getCurrentPosition();
        DriveConfig.odometryCountOffsetR = odR.getCurrentPosition();
    }

    private void updateOdometry() {
        getOdometryCountL();
        getOdometryCountB();
        getOdometryCountR();
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

    /**
     * Stops all drive motors if they are not busy
     */
    public void checkAndStopMotors() {
        if (!frontLeft.isBusy()) {
            frontLeft.setPower(0);
        }
        if (!frontRight.isBusy()) {
            frontRight.setPower(0);
        }
        if (!rearLeft.isBusy()) {
            rearLeft.setPower(0);
        }
        if (!rearRight.isBusy()) {
            rearRight.setPower(0);
        }
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
     * Sets all drive motors to specified zero power behavior
     */
    private void setZeroPowerBehavior() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        rearLeft.setPower(power);
        rearRight.setPower(power);
    }

    public int[] currentMotorsPos() {
        return new int[]{frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), rearLeft.getCurrentPosition(), rearRight.getCurrentPosition()};
    }

    public void resetDriveMotorEncoders() {
        DriveConfig.encoderOffsetFL = frontLeft.getCurrentPosition();
        DriveConfig.encoderOffsetFR = frontRight.getCurrentPosition();
        DriveConfig.encoderOffsetRL = rearLeft.getCurrentPosition();
        DriveConfig.encoderOffsetRR = rearRight.getCurrentPosition();
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
            allMotorPIDControl((int) (angle * DriveConfig.COUNTS_PER_DEGREE), DriveConfig.TURN_SPEED * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                    DriveConfig.motorRampTime, false, true, false, true, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
        } else {
            allMotorPIDControl((int) (-angle * DriveConfig.COUNTS_PER_DEGREE), DriveConfig.TURN_SPEED * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                    DriveConfig.motorRampTime, true, false, true, false, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
        }
//        robotCurrentPosX += ROBOT_HALF_LENGTH * (Math.cos((robotCurrentAngle+degrees)*Math.PI/180.0)
//                - Math.cos(robotCurrentAngle*Math.PI/180.0));
//        robotCurrentPosY += ROBOT_HALF_LENGTH * (Math.sin((robotCurrentAngle+degrees)*Math.PI/180.0)
//                - Math.sin(robotCurrentAngle*Math.PI/180.0));
        robotCurrentAngle += angle;
        // Display it for the driver.
        telemetry.telemetry(4, "turnRobot", "turn to %7.2f degrees", robotCurrentAngle);
    }

    public void turnByTick(double power, double angle) {
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);
        rearLeft.setTargetPosition(0);
        rearRight.setTargetPosition(0);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (DriveConfig.driveFullPower) {
            setDrivePower(1.0);
        } else {
            setDrivePower(power);
        }
        // convert from degrees to motor counts
        int tickCount = (int) (angle * DriveConfig.COUNTS_PER_DEGREE);
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
        this.turnByAngle(DriveConfig.TURN_SPEED, degrees);
//        robotCurrentPosX += ROBOT_HALF_LENGTH * (Math.cos((robotCurrentAngle+degrees)*Math.PI/180.0)
//                - Math.cos(robotCurrentAngle*Math.PI/180.0));
//        robotCurrentPosY += ROBOT_HALF_LENGTH * (Math.sin((robotCurrentAngle+degrees)*Math.PI/180.0)
//                - Math.sin(robotCurrentAngle*Math.PI/180.0));
        robotCurrentAngle += degrees;
        // Display it for the driver.
        telemetry.telemetry(4, "turnRobot", "turn to %7.2f degrees", robotCurrentAngle);
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
        telemetry.telemetry(5, "initial angle", "%7.2f degrees", initialAngle);
        telemetry.telemetry(5, "last read angle", "%7.2f degrees", currentAngle);
        telemetry.telemetry(5, "final angle", "%7.2f degrees", getYaw());
        telemetry.telemetry(5, "final2 angle", "%7.2f degrees", getYaw());
    }

    public void moveToPos2D(double power, double targetPositionX, double targetPositionY) {
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
        distanceCountX = targetPositionX * DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_X;
        distanceCountY = targetPositionY * DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_Y;
        if (DriveConfig.driveFullPower) {
            setPower2D(distanceCountX, distanceCountY, 1.0);
        } else {
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

    /**
     * Distribute power appropriately according to the direction of motion.
     *
     * @param targetPositionX
     * @param targetPositionY
     * @param motorPower
     */
    public void setPower2D(double targetPositionX, double targetPositionY, double motorPower) {
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
        frontLeft.setTargetPosition((int) (+targetPositionX + targetPositionY));
        frontRight.setTargetPosition((int) (-targetPositionX + targetPositionY));
        rearLeft.setTargetPosition((int) (-targetPositionX + targetPositionY));
        rearRight.setTargetPosition((int) (+targetPositionX + targetPositionY));
    }

    public double[] calcMotorPowers2D(double targetPositionX, double targetPositionY, double motorPower) {
        // targetPositionX and targetPositionY determine the direction of movement
        // motorPower determines the magnitude of motor power
        double angleScale = Math.abs(targetPositionX) + Math.abs(targetPositionY);
        double lrPower = motorPower * (-targetPositionX + targetPositionY) / angleScale;
        double lfPower = motorPower * (+targetPositionX + targetPositionY) / angleScale;
        return new double[]{lrPower, lfPower, lfPower, lrPower};
    }

    public void moveToPosABS(double targetPositionX, double targetPositionY) {
        // move to (targetPositionX, targetPositionY) in absolute field coordinate
        double deltaX = targetPositionX - robotCurrentPosX;    // in absolute field coordinate
        double deltaY = targetPositionY - robotCurrentPosY;    // in absolute field coordinate
        double distanceCountX, distanceCountY;  // distance in motor count in robot coordinate
        // rotate vector from field coordinate to robot coordinate
        distanceCountX = deltaX * Math.cos((robotCurrentAngle - 90.0) * Math.PI / 180.0)
                + deltaY * Math.sin((robotCurrentAngle - 90.0) * Math.PI / 180.0);
        distanceCountY = deltaX * Math.cos(robotCurrentAngle * Math.PI / 180.0)
                + deltaY * Math.sin(robotCurrentAngle * Math.PI / 180.0);
        this.moveToPos2D(DriveConfig.DRIVE_SPEED, distanceCountX, distanceCountY);
        robotCurrentPosX = targetPositionX;
        robotCurrentPosY = targetPositionY;
        // Display it for the driver.
        telemetry.telemetry(4, "moveToPosABS", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
    }

    public void moveToPosREL(double targetPositionX, double targetPositionY) {
        // move to (targetPositionX, targetPositionY) in relative robot coordinate
        this.moveToPos2D(DriveConfig.DRIVE_SPEED, targetPositionX, targetPositionY);
        robotCurrentPosX += targetPositionY * Math.cos(robotCurrentAngle * Math.PI / 180.0)
                + targetPositionX * Math.cos((robotCurrentAngle - 90.0) * Math.PI / 180.0);
        robotCurrentPosY += targetPositionY * Math.sin(robotCurrentAngle * Math.PI / 180.0)
                + targetPositionX * Math.sin((robotCurrentAngle - 90.0) * Math.PI / 180.0);
        // Display it for the driver.
        telemetry.telemetry(4, "moveToPosREL", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
    }

    public void moveForward_odometry(double distance) throws InterruptedException {
        moveForward_odometry(distance, DriveConfig.DRIVE_SPEED_Y);
    }

    public void moveForward_odometry(double distance, double motorSpeed) throws InterruptedException {
        resetOdometry();
//        this.moveToPos2D(motorSpeed, 0.0, distance);
        moveForward(distance, motorSpeed);
        updateOdometry();
        telemetry.telemetry(4, "odometry", " L %7.2f R %7.2f B %7.2f", DriveConfig.odometryCountL * DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountR * DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountB * DriveConfig.ODOMETRY_mm_PER_COUNT);
        double angleError = (((double) DriveConfig.odometryCountR) - ((double) DriveConfig.odometryCountL)) * 0.5 * DriveConfig.ODOMETRY_mm_PER_COUNT * (180.0 / 3.14159265) / DriveConfig.ODOMETRY_RADIUS_X;
        turnRobotByTick(-angleError);
        updateOdometry();
        telemetry.telemetry(4, "correction angle", " %7.2f", -angleError);
        telemetry.telemetry(4, "odometry", " L %7.2f R %7.2f B %7.2f", DriveConfig.odometryCountL * DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountR * DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountB * DriveConfig.ODOMETRY_mm_PER_COUNT);
        if (DriveConfig.odometryCountB * DriveConfig.ODOMETRY_mm_PER_COUNT > 25.0) {
            moveLeft(DriveConfig.odometryCountB * DriveConfig.ODOMETRY_mm_PER_COUNT);
        }
        if (DriveConfig.odometryCountB * DriveConfig.ODOMETRY_mm_PER_COUNT < -25.0) {
            moveRight(-DriveConfig.odometryCountB * DriveConfig.ODOMETRY_mm_PER_COUNT);
        }
    }

    public void moveForward(double distance) {
        moveForward(distance, DriveConfig.DRIVE_SPEED_Y);
    }

    public void moveForward(double distance, double motorSpeed) {
        //        this.moveToPos2D(motorSpeed, 0.0, distance);
        allMotorPIDControl((int) (distance * DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_Y), motorSpeed * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                DriveConfig.motorRampTime, true, true, true, true, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
        robotCurrentPosX += distance * Math.cos(robotCurrentAngle * Math.PI / 180.0);
        robotCurrentPosY += distance * Math.sin(robotCurrentAngle * Math.PI / 180.0);
        // Display it for the driver.
        telemetry.telemetry(4, "moveForward", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
    }

    public void moveBackward_odometry(double distance) throws InterruptedException {
        moveBackward_odometry(distance, DriveConfig.DRIVE_SPEED_Y);
    }

    public void moveBackward_odometry(double distance, double motorSpeed) throws InterruptedException {
        resetOdometry();
//        this.moveToPos2D(motorSpeed, 0.0, -distance);
        allMotorPIDControl((int) (distance * DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_Y), motorSpeed * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                DriveConfig.motorRampTime, false, false, false, false, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle + 180.0) * Math.PI / 180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle + 180.0) * Math.PI / 180.0);
        // Display it for the driver.
        telemetry.telemetry(4, "moveBackward", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
        updateOdometry();
        telemetry.telemetry(4, "odometry", " L %7.2f R %7.2f B %7.2f", DriveConfig.odometryCountL * DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountR * DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountB * DriveConfig.ODOMETRY_mm_PER_COUNT);
        double angleError = (((double) DriveConfig.odometryCountR) - ((double) DriveConfig.odometryCountL)) * 0.5 * DriveConfig.ODOMETRY_mm_PER_COUNT * (180.0 / 3.14159265) / DriveConfig.ODOMETRY_RADIUS_X;
        turnRobotByTick(-angleError);
        updateOdometry();
        telemetry.telemetry(4, "correction angle", " %7.2f", -angleError);
        telemetry.telemetry(4, "odometry", " L %7.2f R %7.2f B %7.2f", DriveConfig.odometryCountL * DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountR * DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountB * DriveConfig.ODOMETRY_mm_PER_COUNT);
        if (DriveConfig.odometryCountB * DriveConfig.ODOMETRY_mm_PER_COUNT > 25.0) {
            moveLeft(DriveConfig.odometryCountB * DriveConfig.ODOMETRY_mm_PER_COUNT);
        }
        if (DriveConfig.odometryCountB * DriveConfig.ODOMETRY_mm_PER_COUNT < -25.0) {
            moveRight(-DriveConfig.odometryCountB * DriveConfig.ODOMETRY_mm_PER_COUNT);
        }
    }

    public void moveBackward(double distance) {
        moveBackward(distance, DriveConfig.DRIVE_SPEED_Y);
    }

    public void moveBackward(double distance, double motorSpeed) {
//        this.moveToPos2D(motorSpeed, 0.0, -distance);
        allMotorPIDControl((int) (distance * DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_Y), motorSpeed * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                DriveConfig.motorRampTime, false, false, false, false, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle + 180.0) * Math.PI / 180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle + 180.0) * Math.PI / 180.0);
        // Display it for the driver.
        telemetry.telemetry(4, "moveBackward", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
    }

    public void moveLeft_odometry(double distance) {
        moveLeft_odometry(distance, DriveConfig.DRIVE_SPEED_X);
    }

    public void moveLeft_odometry(double distance, double motorSpeed) {
        resetOdometry();
//        this.moveToPos2D(motorSpeed, -distance, 0.0);
        allMotorPIDControl((int) (distance * DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_X), motorSpeed * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                DriveConfig.motorRampTime, false, true, true, false, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle + 90.0) * Math.PI / 180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle + 90.0) * Math.PI / 180.0);
        // Display it for the driver.
        telemetry.telemetry(4, "moveLeft", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
        updateOdometry();
        telemetry.telemetry(4, "odometry", " L %7.2f R %7.2f B %7.2f", DriveConfig.odometryCountL * DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountR * DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountB * DriveConfig.ODOMETRY_mm_PER_COUNT);
//        sleep(1000);
        double angleError = (((double) DriveConfig.odometryCountR) - ((double) DriveConfig.odometryCountL)) * 0.5 * DriveConfig.ODOMETRY_mm_PER_COUNT * (180.0 / 3.14159265) / DriveConfig.ODOMETRY_RADIUS_X;
        turnRobotByTick(-angleError);
        updateOdometry();
        telemetry.telemetry(4, "correction angle", " %7.2f", -angleError);
        telemetry.telemetry(4, "odometry", " L %7.2f R %7.2f B %7.2f", DriveConfig.odometryCountL * DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountR * DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountB * DriveConfig.ODOMETRY_mm_PER_COUNT);
        double offsetY = (((double) DriveConfig.odometryCountR) + ((double) DriveConfig.odometryCountL)) * 0.5;
        if (offsetY * DriveConfig.ODOMETRY_mm_PER_COUNT > 25.0) {
            moveBackward(offsetY * DriveConfig.ODOMETRY_mm_PER_COUNT);
        }
        if (offsetY * DriveConfig.ODOMETRY_mm_PER_COUNT < -25.0) {
            moveForward(-offsetY * DriveConfig.ODOMETRY_mm_PER_COUNT);
        }
    }

    public void moveLeft(double distance) {
        moveLeft(distance, DriveConfig.DRIVE_SPEED_X);
    }

    public void moveLeft(double distance, double motorSpeed) {
//        this.moveToPos2D(motorSpeed, -distance, 0.0);
        allMotorPIDControl((int) (distance * DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_X), motorSpeed * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                DriveConfig.motorRampTime, false, true, true, false, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle + 90.0) * Math.PI / 180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle + 90.0) * Math.PI / 180.0);
        // Display it for the driver.
        telemetry.telemetry(4, "moveLeft", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
//        sleep(100);
    }

    public void moveRight_odometry(double distance) throws InterruptedException {
        moveRight_odometry(distance, DriveConfig.DRIVE_SPEED_X);
    }

    public void moveRight_odometry(double distance, double motorSpeed) throws InterruptedException {
        resetOdometry();
//        this.moveToPos2D(motorSpeed, distance, 0.0);
        allMotorPIDControl((int) (distance * DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_X), motorSpeed * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                DriveConfig.motorRampTime, true, false, false, true, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle - 90.0) * Math.PI / 180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle - 90.0) * Math.PI / 180.0);
        // Display it for the driver.
        telemetry.telemetry(4, "moveRight", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
        updateOdometry();
        telemetry.telemetry(4, "odometry", " L %7.2f R %7.2f B %7.2f", DriveConfig.odometryCountL * DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountR * DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountB * DriveConfig.ODOMETRY_mm_PER_COUNT);
//        sleep(1000);
        double angleError = (((double) DriveConfig.odometryCountR) - ((double) DriveConfig.odometryCountL)) * 0.5 * DriveConfig.ODOMETRY_mm_PER_COUNT * (180.0 / 3.14159265) / DriveConfig.ODOMETRY_RADIUS_X;
        turnRobotByTick(-angleError);
        updateOdometry();
        telemetry.telemetry(4, "correction angle", " %7.2f", -angleError);
        telemetry.telemetry("odometry", " L %7.2f R %7.2f B %7.2f", DriveConfig.odometryCountL * DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountR * DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountB * DriveConfig.ODOMETRY_mm_PER_COUNT);
        double offsetY = (((double) DriveConfig.odometryCountR) + ((double) DriveConfig.odometryCountL)) * 0.5;
        if (offsetY * DriveConfig.ODOMETRY_mm_PER_COUNT > 25.0) {
            moveBackward(offsetY * DriveConfig.ODOMETRY_mm_PER_COUNT);
        }
        if (offsetY * DriveConfig.ODOMETRY_mm_PER_COUNT < -25.0) {
            moveForward(-offsetY * DriveConfig.ODOMETRY_mm_PER_COUNT);
        }
    }

    public void moveRight(double distance) {
        moveRight(distance, DriveConfig.DRIVE_SPEED_X);
    }

    public void moveRight(double distance, double motorSpeed) {
//        this.moveToPos2D(motorSpeed, distance, 0.0);
        allMotorPIDControl((int) (distance * DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_X), motorSpeed * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                DriveConfig.motorRampTime, true, false, false, true, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle - 90.0) * Math.PI / 180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle - 90.0) * Math.PI / 180.0);
        // Display it for the driver.
        telemetry.telemetry(4, "moveRight", "move to %7.2f, %7.2f", robotCurrentPosX, robotCurrentPosY);
//        sleep(100);
    }

    public void printMotorPIDCoefficients() {
        PIDFCoefficients pidCoeff;
        pidCoeff = getMotorPIDCoefficients(frontLeft, DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.telemetry(1, "Front Left ", "P: %.2f I: %.2f D: %.2f F: %.2f A: %s",
                pidCoeff.p, pidCoeff.i, pidCoeff.d, pidCoeff.f, pidCoeff.algorithm.toString());
        pidCoeff = getMotorPIDCoefficients(frontRight, DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.telemetry(1, "Front Right", "P: %.2f I: %.2f D: %.2f F: %.2f A: %s",
                pidCoeff.p, pidCoeff.i, pidCoeff.d, pidCoeff.f, pidCoeff.algorithm.toString());
        pidCoeff = getMotorPIDCoefficients(rearLeft, DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.telemetry(1, "Rear Left  ", "P: %.2f I: %.2f D: %.2f F: %.2f A: %s",
                pidCoeff.p, pidCoeff.i, pidCoeff.d, pidCoeff.f, pidCoeff.algorithm.toString());
        pidCoeff = getMotorPIDCoefficients(rearRight, DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.telemetry(1, "Rear Right ", "P: %.2f I: %.2f D: %.2f F: %.2f A: %s",
                pidCoeff.p, pidCoeff.i, pidCoeff.d, pidCoeff.f, pidCoeff.algorithm.toString());
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
        return motorControllerEx.getPIDFCoefficients(motorIndex, mode);
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
        String output = String.format(Locale.US, "FL %.3f, %d, FR %.3f %d, RL %.3f %d, RR %.3f %d",
                currentTimeFL, currentCountFL, currentTimeFR, currentCountFR, currentTimeRL, currentCountRL, currentTimeRR, currentCountRR);
        telemetry.telemetry(3, "motorEnc", output);
    }

    /**
     * PID motor control program to ensure all four motors are synchronized
     *
     * @param tickCount:      absolute value of target tickcount of motor
     * @param peakSpeed:      peak speed of motor rotation in tick per second
     * @param maxSpeed:       max speed of motor rotation in tick per second
     * @param rampTime:       motor speed ramp up/down time in sec
     * @param motorFLForward: front left motor is forward
     * @param motorFRForward: front right motor is forward
     * @param motorRLForward: rear left motor is forward
     * @param motorRRForward: rear right motor is forward
     * @param Kp:             coefficient Kp
     * @param Ki:             coefficient Ki
     * @param Kd:             coefficient Kd
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
        double timeOutPeriod = 0.1; // program will time out if the motors got stuck for more than 0.1 second
        double timeOutStartedTime = 0.0;
        int timeOutThreshold = 3; // motor is considered to be stuck if the motor count does not change more than 2 ticks
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
        boolean initialized = false; // disable Ki and Kd terms in first iteration
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
        while (((!isMotorFLDone) || (!isMotorFRDone) || (!isMotorRLDone) || (!isMotorRRDone)) && (!isTimeOutExceeded)) {
            if (!isMotorFLDone) {
                currentCount = frontLeft.getCurrentPosition(); // get current motor tick
                currentTime = ((double) timer.nanoseconds()) * 1.0e-9 - startTime; // get current time
                targetCount = getTargetTickCount(tickCount, peakSpeed, rampTime, currentTime);  // get integrated target tick on the speed profile
                currentTargetSpeed = getTargetSpeed(tickCount, peakSpeed, rampTime, currentTime); // get the target speed on the speed profile
                if (initialized) {  // check if the motor is rotating
                    isMotorFLNotMoving = Math.abs(currentCount - prevCountFL) < timeOutThreshold;
                }
                if (motorFLForward) { // tick count increasing
                    if (currentCount >= tickCount) {
                        isMotorFLDone = true;
                        frontLeft.setPower(0.0);
                    } else {
                        currentError = currentCount - targetCount;
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorFL = acculErrorFL * alpha + currentError * (currentTime - prevTimeFL); // integrate error
                            errorSlope = (currentError - prevErrorFL) / (currentTime - prevTimeFL); // error slope
                            currentPower = currentTargetSpeed / maxSpeed - currentError * Kp - acculErrorFL * Ki - errorSlope * Kd; // apply PID correction
                        } else { // at the first point, use Kp only
                            currentPower = currentTargetSpeed / maxSpeed - currentError * Kp;
                        }
                        if (currentPower > 1.0) frontLeft.setPower(1.0);
                        if (currentPower < 0.0) frontLeft.setPower(0.0);
                    }
                } else { // motorFLForward is false, tick count negative and decreasing
                    if (currentCount <= -tickCount) {
                        isMotorFLDone = true;
                        frontLeft.setPower(0.0);
                    } else {
                        currentError = -currentCount - targetCount;
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorFL = acculErrorFL * alpha + currentError * (currentTime - prevTimeFL);  // integrate error
                            errorSlope = (currentError - prevErrorFL) / (currentTime - prevTimeFL);         // error slope
                            currentPower = -currentTargetSpeed / maxSpeed + currentError * Kp + acculErrorFL * Ki + errorSlope * Kd; // apply PID correction
                        } else { // at the first point, use Kp only
                            currentPower = -currentTargetSpeed / maxSpeed + currentError * Kp;
                        }
                        if (currentPower < -1.0) frontLeft.setPower(-1.0);
                        if (currentPower > 0.0) frontLeft.setPower(0.0);
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
                    isMotorFRNotMoving = Math.abs(currentCount - prevCountFR) < timeOutThreshold;
                }
                if (motorFRForward) {
                    if (currentCount >= tickCount) {
                        isMotorFRDone = true;
                        frontRight.setPower(0.0);
                    } else {
                        currentError = currentCount - targetCount;
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorFR = acculErrorFR * alpha + currentError * (currentTime - prevTimeFR);  // integrate error
                            errorSlope = (currentError - prevErrorFR) / (currentTime - prevTimeFR);         // error slope
                            currentPower = currentTargetSpeed / maxSpeed - currentError * Kp - acculErrorFR * Ki - errorSlope * Kd; // apply PID correction
                        } else { // at the first point, use Kp only
                            currentPower = currentTargetSpeed / maxSpeed - currentError * Kp;
                        }
                        if (currentPower > 1.0) frontRight.setPower(1.0);
                        if (currentPower < 0.0) frontRight.setPower(0.0);
                    }
                } else { // motorFRForward is false
                    if (currentCount <= -tickCount) {
                        isMotorFRDone = true;
                        frontRight.setPower(0.0);
                    } else {
                        currentError = -currentCount - targetCount;
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorFR = acculErrorFR * alpha + currentError * (currentTime - prevTimeFR);  // integrate error
                            errorSlope = (currentError - prevErrorFR) / (currentTime - prevTimeFR);         // error slope
                            currentPower = -currentTargetSpeed / maxSpeed + currentError * Kp + acculErrorFR * Ki + errorSlope * Kd; // apply PID correction
                        } else { // at the first point, use Kp only
                            currentPower = -currentTargetSpeed / maxSpeed + currentError * Kp;
                        }
                        if (currentPower < -1.0) frontRight.setPower(-1.0);
                        if (currentPower > 0.0) frontRight.setPower(0.0);
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
                    isMotorRLNotMoving = Math.abs(currentCount - prevCountRL) < timeOutThreshold;
                }
                if (motorRLForward) {
                    if (currentCount >= tickCount) {
                        isMotorRLDone = true;
                        rearLeft.setPower(0.0);
                    } else {
                        currentError = currentCount - targetCount;
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorRL = acculErrorRL * alpha + currentError * (currentTime - prevTimeRL);  // integrate error
                            errorSlope = (currentError - prevErrorRL) / (currentTime - prevTimeRL);         // error slope
                            currentPower = currentTargetSpeed / maxSpeed - currentError * Kp - acculErrorRL * Ki - errorSlope * Kd; // apply PID correction
                        } else { // at the first point, use Kp only
                            currentPower = currentTargetSpeed / maxSpeed - currentError * Kp;
                        }
                        if (currentPower > 1.0) currentPower = 1.0;
                        rearLeft.setPower(currentPower);
                        if (currentPower < 0.0) currentPower = 0.0;
                        rearLeft.setPower(currentPower);
                    }
                } else { // motorFLForward is false
                    if (currentCount <= -tickCount) {
                        isMotorRLDone = true;
                        rearLeft.setPower(0.0);
                    } else {
                        currentError = -currentCount - targetCount;
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorRL = acculErrorRL * alpha + currentError * (currentTime - prevTimeRL);  // integrate error
                            errorSlope = (currentError - prevErrorRL) / (currentTime - prevTimeRL);         // error slope
                            currentPower = -currentTargetSpeed / maxSpeed + currentError * Kp + acculErrorRL * Ki + errorSlope * Kd; // apply PID correction
                        } else { // at the first point, use Kp only
                            currentPower = -currentTargetSpeed / maxSpeed + currentError * Kp;
                        }
                        if (currentPower < -1.0) currentPower = -1.0;
                        rearLeft.setPower(currentPower);
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
                    isMotorRRNotMoving = Math.abs(currentCount - prevCountRR) < timeOutThreshold;
                }
                if (motorRRForward) {
                    currentError = currentCount - targetCount;
                    if (currentCount >= tickCount) {
                        isMotorRRDone = true;
                        rearRight.setPower(0.0);
                    } else {
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorRR = acculErrorRR * alpha + currentError * (currentTime - prevTimeRR);  // integrate error
                            errorSlope = (currentError - prevErrorRR) / (currentTime - prevTimeRR);         // error slope
                            currentPower = currentTargetSpeed / maxSpeed - currentError * Kp - acculErrorRR * Ki - errorSlope * Kd; // apply PID correction
                        } else { // at the first point, use Kp only
                            currentPower = currentTargetSpeed / maxSpeed - currentError * Kp;
                        }
                        if (currentPower > 1.0) currentPower = 1.0;
                        rearRight.setPower(currentPower);
                        if (currentPower < 0.0) currentPower = 0.0;
                        rearRight.setPower(currentPower);
                    }
                } else { // motorFLForward is false
                    currentError = -currentCount - targetCount;
                    if (currentCount <= -tickCount) {
                        isMotorRRDone = true;
                        rearRight.setPower(0.0);
                    } else {
                        if (initialized) { // after the first point, the previous data is valid
                            acculErrorRR = acculErrorRR * alpha + currentError * (currentTime - prevTimeRR);  // integrate error
                            errorSlope = (currentError - prevErrorRR) / (currentTime - prevTimeRR);         // error slope
                            currentPower = -currentTargetSpeed / maxSpeed + currentError * Kp + acculErrorRR * Ki + errorSlope * Kd; // apply PID correction
                        } else { // at the first point, use Kp only
                            currentPower = -currentTargetSpeed / maxSpeed + currentError * Kp;
                        }
                        if (currentPower < -1.0) currentPower = -1.0;
                        rearRight.setPower(currentPower);
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
                } else { // time out was not started yet
                    isTimeOutStarted = true;
                    timeOutStartedTime = currentTime;
                }
            } else {
                isTimeOutStarted = false;
                isTimeOutExceeded = false;
            }
            String output = String.format(Locale.US, "FL %.1f, %d, FR %.1f %d, RL %.1f %d, RR %.1f %d %.1f %.3f %.1f %.3f %s %s %s %s %s %.1f %s",
                    prevTimeFL * 1000.0, prevCountFL, prevTimeFR * 1000.0, prevCountFR, prevTimeRL * 1000.0, prevCountRL,
                    prevTimeRR * 1000.0, prevCountRR, currentError, acculErrorRR, errorSlope, currentPower,
                    isMotorFLNotMoving ? "Y" : "N", isMotorFRNotMoving ? "Y" : "N", isMotorRLNotMoving ? "Y" : "N", isMotorRRNotMoving ? "Y" : "N",
                    isTimeOutStarted ? "Y" : "N", timeOutStartedTime * 1000.0, isTimeOutExceeded ? "Y" : "N");
            telemetry.telemetry(2, "motorEnc", output);
        }

    }

    private int getTargetTickCount(int tickCount, double speed, double rampTime, double elapsedTime) {
        double speedOffset = speed * 0.15; // ramp up and ramp down with this speed offset so that there is no time the speed is close to zero
        double speedExcess = speed - speedOffset;

        int targetTick = (int) ((0.5 * speedExcess * elapsedTime / rampTime + speedOffset) * elapsedTime);
        if (tickCount < rampTime * (speed + speedOffset)) {  // distance is shorter than a complete ramp up/ramp down cycle
            double halfTime = (Math.sqrt(speedOffset * speedOffset + 4.0 * tickCount * speedExcess / rampTime) - speedOffset) * rampTime * 0.5 / speedExcess;
            if (!(elapsedTime < halfTime)) { // during ramp down time
                double remainTime = halfTime + halfTime - elapsedTime;
                targetTick = tickCount - ((int) ((0.5 * speedExcess * remainTime / rampTime + speedOffset) * remainTime));
            }
        } else { // distance is long enough to reach the cruise speed
            if (tickCount - speedOffset * rampTime > speed * elapsedTime) { // during constant speed period
                targetTick = (int) (speed * (elapsedTime - rampTime * 0.5) + 0.5 * rampTime * speedOffset);
            } else if (!(elapsedTime < rampTime)) { // during ramp up time
                double remainTime = (tickCount - speedOffset * rampTime) / speed + rampTime - elapsedTime;
                targetTick = tickCount - ((int) ((0.5 * speedExcess * remainTime / rampTime + speedOffset) * remainTime));
            }
        }
        if (targetTick > tickCount) {
            targetTick = tickCount;
        }
        return targetTick;
    }

    private double getTargetSpeed(int tickCount, double speed, double rampTime, double elapsedTime) {
        double targetSpeed;
        double speedOffset = speed * 0.15; // ramp up and ramp down with this speed offset so that there is no time the speed is close to zero
        double speedExcess = speed - speedOffset;

        if (tickCount < rampTime * (speed + speedOffset)) {  // distance is shorter than a complete ramp up/ramp down cycle
            double halfTime = (Math.sqrt(speedOffset * speedOffset + 4.0 * tickCount * speedExcess / rampTime) - speedOffset) * rampTime * 0.5 / speedExcess;
            if (elapsedTime < halfTime) { // during ramp up time
                targetSpeed = speedExcess * elapsedTime / rampTime + speedOffset;
            } else {  // during ramp down time
                double remainTime = halfTime + halfTime - elapsedTime;
                targetSpeed = speedExcess * remainTime / rampTime + speedOffset;
            }
        } else { // distance is long enough to reach the cruise speed
            if (elapsedTime < rampTime) { // during ramp up time
                targetSpeed = speedExcess * elapsedTime / rampTime + speedOffset;
            } else if (tickCount - speedOffset * rampTime > speed * elapsedTime) { // during constant speed period
                targetSpeed = speed;
            } else {  // during ramp down time
                double remainTime = (tickCount - speedOffset * rampTime) / speed + rampTime - elapsedTime;
                targetSpeed = speedExcess * remainTime / rampTime + speedOffset;
            }
        }
        if (targetSpeed < speedOffset) {
            targetSpeed = speedOffset;
        }
        return targetSpeed;
    }

    public void setDrivePowers(double[] powers) {
        frontLeft.setPower(powers[0]);
        frontRight.setPower(powers[1]);
        rearLeft.setPower(powers[2]);
        rearRight.setPower(powers[3]);
    }
}
