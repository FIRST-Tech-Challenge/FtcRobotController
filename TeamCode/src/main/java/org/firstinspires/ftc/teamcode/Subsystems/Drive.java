
package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.Config.DriveConfig;
import org.firstinspires.ftc.teamcode.TrcPose2D;

import java.util.List;
import java.util.Locale;

/** Mecanum drivetrain subsystem */
public class Drive extends MinorSubsystem {
    // DC Motors
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;

    // use motor encoder for odometry
    public DcMotorEx odL;
    public DcMotorEx odB;
    public DcMotorEx odR;


    // Sensors
    private BNO055IMU imu;

    private double robotCurrentPosX;    // unit in mm
    private double robotCurrentPosY;    // unit in mm
    private double robotCurrentAngle;   // unit in degrees
    

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    private long startTime;

    public Drive(Robot robot, List<DcMotorEx> motors, DcMotorEx odL, DcMotorEx odB, DcMotorEx odR, BNO055IMU imu) {
        super(robot);
        this.frontLeft = motors.get(0);
        this.frontRight = motors.get(1);
        this.rearLeft = motors.get(2);
        this.rearRight = motors.get(3);
        this.odL = odL;
        this.odB = odB;
        this.odR = odR;
        this.imu = imu;
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public double getAngularVMaxNeverrest20(){
        return DriveConfig.ANGULAR_V_MAX_NEVERREST_20;
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
        frontLeft.setVelocity(DriveConfig.ANGULAR_V_MAX_NEVERREST_20);
        frontRight.setVelocity(DriveConfig.ANGULAR_V_MAX_NEVERREST_20);
        rearLeft.setVelocity(DriveConfig.ANGULAR_V_MAX_NEVERREST_20);
        rearRight.setVelocity(DriveConfig.ANGULAR_V_MAX_NEVERREST_20);
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
        DriveConfig.driveFullPower = fullPower;
    }

    public void setTargetPosition(int targetPosition) {
        frontLeft.setTargetPosition(targetPosition);
        frontRight.setTargetPosition(targetPosition);
        rearLeft.setTargetPosition(targetPosition);
        rearRight.setTargetPosition(targetPosition);
    }

    public int[] getCurrentPositions() {
        return new int[] {
                frontLeft.getCurrentPosition() - DriveConfig.encoderOffsetFL,
                frontRight.getCurrentPosition() - DriveConfig.encoderOffsetFR,
                rearLeft.getCurrentPosition() - DriveConfig.encoderOffsetRL,
                rearRight.getCurrentPosition() - DriveConfig.encoderOffsetRR
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
        }
        else {
            allMotorPIDControl((int) (-angle * DriveConfig.COUNTS_PER_DEGREE), DriveConfig.TURN_SPEED * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                    DriveConfig.motorRampTime, true, false, true, false, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
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
        if (DriveConfig.driveFullPower) {
            setDrivePower(1.0);
        }
        else {
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
        distanceCountX = targetPositionX * DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_X;
        distanceCountY = targetPositionY * DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_Y;
        if (DriveConfig.driveFullPower) {
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
        double power = motorPower * (-targetPositionX + targetPositionY) / angleScale;
        double lrPower = power;
        double lfPower = motorPower * (+ targetPositionX + targetPositionY) / angleScale;
        double rrPower = motorPower * (+ targetPositionX + targetPositionY) / angleScale;
        double rfPower = power;
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
        this.moveToPos2D(DriveConfig.DRIVE_SPEED, distanceCountX, distanceCountY);
        robotCurrentPosX = targetPositionX;
        robotCurrentPosY = targetPositionY;
        // Display it for the driver.
        opMode.telemetry.addData("moveToPosABS",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void moveToPosREL(double targetPositionX, double targetPositionY) {
        // move to (targetPositionX, targetPositionY) in relative robot coordinate
        this.moveToPos2D(DriveConfig.DRIVE_SPEED, targetPositionX, targetPositionY);
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
        moveForward_odometry(distance, DriveConfig.DRIVE_SPEED_Y);
    }

    public void moveForward_odometry(double distance, double motorSpeed) throws InterruptedException {
        resetOdometry();
//        this.moveToPos2D(motorSpeed, 0.0, distance);
        allMotorPIDControl( (int) (distance* DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_Y), motorSpeed * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                DriveConfig.motorRampTime, true, true, true, true, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
        robotCurrentPosX += distance * Math.cos(robotCurrentAngle*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin(robotCurrentAngle*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveForward",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        updateOdometry();
        opMode.telemetry.addData("odometry",  " L %7.2f R %7.2f B %7.2f", DriveConfig.odometryCountL* DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountR* DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountB* DriveConfig.ODOMETRY_mm_PER_COUNT);
        opMode.telemetry.update();
//        sleep(1000);
        double angleError = (((double) DriveConfig.odometryCountR) - ((double) DriveConfig.odometryCountL))*0.5* DriveConfig.ODOMETRY_mm_PER_COUNT*(180.0/3.14159265)/ DriveConfig.ODOMETRY_RADIUS_X;
        turnRobotByTick(-angleError);
        updateOdometry();
        opMode.telemetry.addData("correction angle",  " %7.2f", -angleError);
        opMode.telemetry.addData("odometry",  " L %7.2f R %7.2f B %7.2f", DriveConfig.odometryCountL* DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountR* DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountB* DriveConfig.ODOMETRY_mm_PER_COUNT);
        opMode.telemetry.update();
        if (DriveConfig.odometryCountB* DriveConfig.ODOMETRY_mm_PER_COUNT > 25.0) {
            moveLeft(DriveConfig.odometryCountB* DriveConfig.ODOMETRY_mm_PER_COUNT);
        }
        if (DriveConfig.odometryCountB* DriveConfig.ODOMETRY_mm_PER_COUNT < -25.0) {
            moveRight(-DriveConfig.odometryCountB* DriveConfig.ODOMETRY_mm_PER_COUNT);
        }
//        sleep(1000);
//        sleep(100);
    }

    public void moveForward(double distance) {
        moveForward(distance, DriveConfig.DRIVE_SPEED_Y);
    }

    public void moveForward(double distance, double motorSpeed) {
        //        this.moveToPos2D(motorSpeed, 0.0, distance);
        allMotorPIDControl( (int) (distance * DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_Y), motorSpeed * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                DriveConfig.motorRampTime, true, true, true, true, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
        robotCurrentPosX += distance * Math.cos(robotCurrentAngle*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin(robotCurrentAngle*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveForward",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void moveBackward_odometry(double distance) throws InterruptedException {
        moveBackward_odometry(distance, DriveConfig.DRIVE_SPEED_Y);
    }

    public void moveBackward_odometry(double distance, double motorSpeed) throws InterruptedException {
        resetOdometry();
//        this.moveToPos2D(motorSpeed, 0.0, -distance);
        allMotorPIDControl((int) (distance* DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_Y), motorSpeed * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                DriveConfig.motorRampTime, false, false, false, false, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+180.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+180.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveBackward",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        updateOdometry();
        opMode.telemetry.addData("odometry",  " L %7.2f R %7.2f B %7.2f", DriveConfig.odometryCountL* DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountR* DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountB* DriveConfig.ODOMETRY_mm_PER_COUNT);
        opMode.telemetry.update();
//        sleep(1000);
        double angleError = (((double) DriveConfig.odometryCountR) - ((double) DriveConfig.odometryCountL))*0.5* DriveConfig.ODOMETRY_mm_PER_COUNT*(180.0/3.14159265)/ DriveConfig.ODOMETRY_RADIUS_X;
        turnRobotByTick(-angleError);
        updateOdometry();
        opMode.telemetry.addData("correction angle",  " %7.2f", -angleError);
        opMode.telemetry.addData("odometry",  " L %7.2f R %7.2f B %7.2f", DriveConfig.odometryCountL* DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountR* DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountB* DriveConfig.ODOMETRY_mm_PER_COUNT);
        opMode.telemetry.update();
        if (DriveConfig.odometryCountB* DriveConfig.ODOMETRY_mm_PER_COUNT > 25.0) {
            moveLeft(DriveConfig.odometryCountB* DriveConfig.ODOMETRY_mm_PER_COUNT);
        }
        if (DriveConfig.odometryCountB* DriveConfig.ODOMETRY_mm_PER_COUNT < -25.0) {
            moveRight(-DriveConfig.odometryCountB* DriveConfig.ODOMETRY_mm_PER_COUNT);
        }
//        sleep(1000);
//        sleep(100);
    }

    public void moveBackward(double distance) {
        moveBackward(distance, DriveConfig.DRIVE_SPEED_Y);
    }

    public void moveBackward(double distance, double motorSpeed) {
//        this.moveToPos2D(motorSpeed, 0.0, -distance);
        allMotorPIDControl((int) (distance* DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_Y), motorSpeed * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                DriveConfig.motorRampTime, false, false, false, false, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+180.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+180.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveBackward",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void moveLeft_odometry(double distance) throws InterruptedException {
        moveLeft_odometry(distance, DriveConfig.DRIVE_SPEED_X);
    }

    public void moveLeft_odometry(double distance, double motorSpeed) throws InterruptedException {
        resetOdometry();
//        this.moveToPos2D(motorSpeed, -distance, 0.0);
        allMotorPIDControl((int) (distance* DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_X), motorSpeed * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                DriveConfig.motorRampTime, false, true, true, false, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+90.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+90.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveLeft",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        updateOdometry();
        opMode.telemetry.addData("odometry",  " L %7.2f R %7.2f B %7.2f", DriveConfig.odometryCountL* DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountR* DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountB* DriveConfig.ODOMETRY_mm_PER_COUNT);
        opMode.telemetry.update();
//        sleep(1000);
        double angleError = (((double) DriveConfig.odometryCountR) - ((double) DriveConfig.odometryCountL))*0.5* DriveConfig.ODOMETRY_mm_PER_COUNT*(180.0/3.14159265)/ DriveConfig.ODOMETRY_RADIUS_X;
        turnRobotByTick(-angleError);
        updateOdometry();
        opMode.telemetry.addData("correction angle",  " %7.2f", -angleError);
        opMode.telemetry.addData("odometry",  " L %7.2f R %7.2f B %7.2f", DriveConfig.odometryCountL* DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountR* DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountB* DriveConfig.ODOMETRY_mm_PER_COUNT);
        opMode.telemetry.update();
        double offsetY = (((double) DriveConfig.odometryCountR) + ((double) DriveConfig.odometryCountL))*0.5;
        if (offsetY* DriveConfig.ODOMETRY_mm_PER_COUNT > 25.0) {
            moveBackward(offsetY* DriveConfig.ODOMETRY_mm_PER_COUNT);
        }
        if (offsetY* DriveConfig.ODOMETRY_mm_PER_COUNT < -25.0) {
            moveForward(-offsetY* DriveConfig.ODOMETRY_mm_PER_COUNT);
        }
//        sleep(1000);
//        sleep(100);
    }

    public void moveLeft(double distance) {
        moveLeft(distance, DriveConfig.DRIVE_SPEED_X);
    }

    public void moveLeft(double distance, double motorSpeed) {
//        this.moveToPos2D(motorSpeed, -distance, 0.0);
        allMotorPIDControl((int) (distance* DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_X), motorSpeed * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                DriveConfig.motorRampTime, false, true, true, false, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+90.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+90.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveLeft",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void moveRight_odometry(double distance) throws InterruptedException {
        moveRight_odometry(distance, DriveConfig.DRIVE_SPEED_X);
    }

    public void moveRight_odometry(double distance, double motorSpeed) throws InterruptedException {
        resetOdometry();
//        this.moveToPos2D(motorSpeed, distance, 0.0);
        allMotorPIDControl((int) (distance* DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_X), motorSpeed * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                DriveConfig.motorRampTime, true, false, false, true, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveRight",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        updateOdometry();
        opMode.telemetry.addData("odometry",  " L %7.2f R %7.2f B %7.2f", DriveConfig.odometryCountL* DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountR* DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountB* DriveConfig.ODOMETRY_mm_PER_COUNT);
        opMode.telemetry.update();
//        sleep(1000);
        double angleError = (((double) DriveConfig.odometryCountR) - ((double) DriveConfig.odometryCountL))*0.5* DriveConfig.ODOMETRY_mm_PER_COUNT*(180.0/3.14159265)/ DriveConfig.ODOMETRY_RADIUS_X;
        turnRobotByTick(-angleError);
        updateOdometry();
        opMode.telemetry.addData("correction angle",  " %7.2f", -angleError);
        opMode.telemetry.addData("odometry",  " L %7.2f R %7.2f B %7.2f", DriveConfig.odometryCountL* DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountR* DriveConfig.ODOMETRY_mm_PER_COUNT, DriveConfig.odometryCountB* DriveConfig.ODOMETRY_mm_PER_COUNT);
        opMode.telemetry.update();
        double offsetY = (((double) DriveConfig.odometryCountR) + ((double) DriveConfig.odometryCountL))*0.5;
        if (offsetY* DriveConfig.ODOMETRY_mm_PER_COUNT > 25.0) {
            moveBackward(offsetY* DriveConfig.ODOMETRY_mm_PER_COUNT);
        }
        if (offsetY* DriveConfig.ODOMETRY_mm_PER_COUNT < -25.0) {
            moveForward(-offsetY* DriveConfig.ODOMETRY_mm_PER_COUNT);
        }
//        sleep(1000);
//        sleep(100);
    }

    public void moveRight(double distance) {
        moveRight(distance, DriveConfig.DRIVE_SPEED_X);
    }

    public void moveRight(double distance, double motorSpeed) {
//        this.moveToPos2D(motorSpeed, distance, 0.0);
        allMotorPIDControl((int) (distance* DriveConfig.COUNTS_PER_MM * DriveConfig.COUNTS_CORRECTION_X), motorSpeed * DriveConfig.ANGULAR_V_MAX_NEVERREST_20, DriveConfig.ANGULAR_V_MAX_NEVERREST_20,
                DriveConfig.motorRampTime, true, false, false, true, DriveConfig.motorKp, DriveConfig.motorKi, DriveConfig.motorKd);
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
        String output = String.format(Locale.US, "FL %.3f, %d, FR %.3f %d, RL %.3f %d, RR %.3f %d",
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
                    isMotorFLNotMoving = Math.abs(currentCount - prevCountFL) < timeOutThreshold;
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
                    isMotorFRNotMoving = Math.abs(currentCount - prevCountFR) < timeOutThreshold;
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
                    isMotorRLNotMoving = Math.abs(currentCount - prevCountRL) < timeOutThreshold;
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
                    isMotorRRNotMoving = Math.abs(currentCount - prevCountRR) < timeOutThreshold;
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
