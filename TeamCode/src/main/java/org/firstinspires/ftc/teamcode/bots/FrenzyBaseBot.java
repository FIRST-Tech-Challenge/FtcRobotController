package org.firstinspires.ftc.teamcode.bots;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.calibration.BotCalibConfig;
import org.firstinspires.ftc.teamcode.calibration.MotorReductionBot;
import org.firstinspires.ftc.teamcode.odometry.IBaseOdometry;
import org.firstinspires.ftc.teamcode.skills.Gyroscope;
import org.opencv.core.Mat;

import java.io.File;
import java.util.ArrayList;

import static java.lang.StrictMath.abs;

public class FrenzyBaseBot implements IOdoBot {

    protected DcMotorEx frontLeft = null;
    protected DcMotorEx frontRight = null;
    protected DcMotorEx backLeft = null;
    protected DcMotorEx backRight = null;

    protected HardwareMap hwMap = null;
    protected Telemetry telemetry;

    protected ElapsedTime runtime;

    private Gyroscope gyro = null;

    private int encoderDirection = 1;

    //    MaxVelocityTest: maxLF: 2440.00, maxRF: 2640.00, maxLB: 2520.00, maxRB: 2600.00
//    maxLF: 2760.00, maxRF: 2700.00, maxLB: 2680.00, maxRB: 2680.00

    protected static double MAX_VELOCITY_BACK_LEFT = 2680.00; //0.971
    protected static double MAX_VELOCITY_BACK_RIGHT = 2680.00; //0.971
    protected static double MAX_VELOCITY_FRONT_LEFT = 2760.00; //1
    protected static double MAX_VELOCITY_FRONT_RIGHT = 2700.00; //0.978

//    protected static double MAX_VELOCITY_VALUE = MAX_VELOCITY_FRONT_LEFT; //1

//    protected static double MAX_VELOCITY_BACK_LEFT = 2680.00;
//    protected static double MAX_VELOCITY_BACK_RIGHT = 2680.00;
//    protected static double MAX_VELOCITY_FRONT_LEFT = 2680.00;
//    protected static double MAX_VELOCITY_FRONT_RIGHT = 2680.00;

    protected LinearOpMode owner = null;

    static final double COUNTS_PER_MOTOR_GB = 537.7;
    static final double MOTOR_RPM_GB = 312;
    static final double MOTOR_RPS_GB = MOTOR_RPM_GB/60;
    public static final double MAX_VELOCITY_GB = MAX_VELOCITY_BACK_LEFT; //Lowest of max velocities
    public static final double MAX_VELOCITY_REV = 2140;


    protected static double positionPIDF = 20;
    protected static int positionToleration = 110;


    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP. was 2 in the sample
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH_GB = (COUNTS_PER_MOTOR_GB * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI); //42.8


    public static final double ROBOT_LENGTH_X = 17.25;
    public static final double ROBOT_LENGTH_Y = 17.5;

    public static final double ROBOT_CENTER_X = 8.25;
    public static final double ROBOT_CENTER_Y = 8.25;


    private BotCalibConfig botConfig;

    public static String LEFT_FRONT = "frontLeft";
    public static String RIGHT_FRONT = "frontRight";
    public static String LEFT_BACK = "backLeft";
    public static String RIGHT_BACK = "backRight";

    protected ArrayList<AutoDot> namedCoordinates = new ArrayList<>();

    private static final String TAG = "FrenzyBaseBot";


    public FrenzyBaseBot() {

    }

    public void init(LinearOpMode owner, HardwareMap hw, Telemetry t) throws Exception {
        this.owner = owner;
        this.hwMap = hw;
        this.telemetry = t;
        try {
            initCalibData();
            // Define and Initialize Motors
            frontLeft = hwMap.get(DcMotorEx.class, LEFT_FRONT);
            frontRight = hwMap.get(DcMotorEx.class, RIGHT_FRONT);
            backLeft = hwMap.get(DcMotorEx.class, LEFT_BACK);
            backRight = hwMap.get(DcMotorEx.class, RIGHT_BACK);

            resetEncoders();
            this.encoderDirection = 1;


            if (backLeft != null) {
                backLeft.setDirection(DcMotor.Direction.REVERSE);
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                setPIDF(backLeft, MAX_VELOCITY_BACK_LEFT, positionPIDF);
            }

            if (backRight != null) {
                backRight.setDirection(DcMotor.Direction.FORWARD);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                setPIDF(backRight, MAX_VELOCITY_BACK_RIGHT, positionPIDF);
            }

            if (frontLeft != null) {
                frontLeft.setDirection(DcMotor.Direction.REVERSE);
                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                setPIDF(frontLeft, MAX_VELOCITY_FRONT_LEFT, positionPIDF);
            }

            if (frontRight != null) {
                frontRight.setDirection(DcMotor.Direction.FORWARD);
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                setPIDF(frontRight, MAX_VELOCITY_FRONT_RIGHT, positionPIDF);
            }

            stop();
        } catch (Exception ex) {
            //issues accessing drive resources
            Log.e(TAG, "Init error", ex);
            throw new Exception("Issues accessing one of drive motors. Check the controller config", ex);
        }
    }

    protected void setPIDF(DcMotorEx motor, double maxVelocity, double positionPIDF) {
        double F = 32767 / maxVelocity;
        double P = 0.1 * F;
        double I = 0.1 * P;
        double D = 0;

        motor.setVelocityPIDFCoefficients(P, I, D, F);
//        double coeff = maxVelocity/MAX_VELOCITY_VALUE;
        Log.d(TAG, String.format("positionPIDF: %.2f", positionPIDF));
        Log.d(TAG, String.format("positionToleration: %d", positionToleration));
        motor.setPositionPIDFCoefficients(positionPIDF);
        motor.setTargetPositionTolerance(positionToleration);

    }
    public Telemetry getTelemetry() {
        return this.telemetry;
    }

    public String printInfo(){
        PIDFCoefficients flp = this.frontLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients fle = this.frontLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients frp = this.frontRight.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients fre = this.frontRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients blp = this.backLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients ble = this.backLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        PIDFCoefficients brp = this.backRight.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients bre = this.backRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        String info = "";
        info = String.format("LeftFront PIDF pos: %.2f, %.2f, %.2f, %.2f\n", flp.p, flp.i, flp.d, flp.f);
        info = String.format("%sLeftFront PIDF vel: %.2f, %.2f, %.2f, %.2f\n", info, fle.p, fle.i, fle.d, fle.f);

        info = String.format("%sRightFront PIDF pos: %.2f, %.2f, %.2f, %.2f\n", info, frp.p, frp.i, frp.d, frp.f);
        info = String.format("%sRightFront PIDF vel: %.2f, %.2f, %.2f, %.2f\n", info, fre.p, fle.i, fre.d, fre.f);

        info = String.format("%sLeftBack PIDF pos: %.2f, %.2f, %.2f, %.2f\n", info, blp.p, blp.i, blp.d, blp.f);
        info = String.format("%sLeftBack PIDF vel: %.2f, %.2f, %.2f, %.2f\n", info, ble.p, ble.i, ble.d, ble.f);

        info = String.format("%sRightBack PIDF pos: %.2f, %.2f, %.2f, %.2f\n", info, brp.p, brp.i, brp.d, brp.f);
        info = String.format("%sRightBack PIDF vel: %.2f, %.2f, %.2f, %.2f\n", info, bre.p, bre.i, bre.d, bre.f);

        info = String.format("%sTargetPositionTolerance Left Front: %d\n", info, frontLeft.getTargetPositionTolerance());
        info = String.format("%sTargetPositionTolerance Left Back: %d\n", info, backLeft.getTargetPositionTolerance());
        info = String.format("%sTargetPositionTolerance Right Front: %d\n", info, frontRight.getTargetPositionTolerance());
        info = String.format("%sTargetPositionTolerance Right Back: %d\n", info, backRight.getTargetPositionTolerance());

        return info;
    }


    protected void resetEncoders() {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void stop() {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            this.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }



    public double getLeftVelocity() {
        return frontLeft.getVelocity();
    }

    public double getRightVelocity() {
        return frontRight.getVelocity();
    }

    public double getLeftBackVelocity() {
        return backLeft.getVelocity();
    }

    public double getRightBackVelocity() {
        return backRight.getVelocity();
    }

    public void moveAtMaxSpeed(){
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null){
            this.frontLeft.setPower(1);
            this.frontRight.setPower(1);
            this.backLeft.setPower(1);
            this.backRight.setPower(1);
        }
    }

    public void move(double drive, double turn) {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            double rightPower = Range.clip(drive + turn, -1.0, 1.0);
            double leftPower = Range.clip(drive - turn, -1.0, 1.0);

            //create dead zone for bad joysticks
            if (drive > 0) {
                if (Math.abs(rightPower) < 0.02) {
                    rightPower = 0;
                }

                if (Math.abs(leftPower) < 0.02) {
                    leftPower = 0;
                }
            }

            //apply logarithmic adjustment
            rightPower = rightPower * 100 / 110;
            rightPower = rightPower * rightPower * rightPower;

            leftPower = leftPower * 100 / 110;
            leftPower = leftPower * leftPower * leftPower;

            this.frontLeft.setVelocity(MAX_VELOCITY_GB*leftPower);
            this.frontRight.setVelocity(MAX_VELOCITY_GB*rightPower);
            this.backLeft.setVelocity(MAX_VELOCITY_GB*leftPower);
            this.backRight.setVelocity(MAX_VELOCITY_GB*rightPower);
        }
    }

    public void moveAuto(double power) {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            double rightPower = Range.clip(power, -1.0, 1.0);
            double leftPower = Range.clip(power, -1.0, 1.0);


            this.frontLeft.setVelocity(MAX_VELOCITY_GB*leftPower);
            this.frontRight.setVelocity(MAX_VELOCITY_GB*rightPower);
            this.backLeft.setVelocity(MAX_VELOCITY_GB*leftPower);
            this.backRight.setVelocity(MAX_VELOCITY_GB*rightPower);
        }
    }

    protected boolean motorsBusy(){
        boolean busy = this.frontLeft.isBusy() && this.frontRight.isBusy() && this.backLeft.isBusy() && this.backRight.isBusy();
        return busy;
    }


    public void moveToPos(BotMoveProfile profile, IBaseOdometry locator){
        resetEncoders();
        this.frontLeft.setTargetPosition((int)profile.getLeftTarget());
        this.frontRight.setTargetPosition((int)profile.getRightTarget());
        this.backLeft.setTargetPosition((int)profile.getLeftTargetBack());
        this.backRight.setTargetPosition((int)profile.getRightTargetBack());


        this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MotorReductionBot mr = profile.getMotorReduction();

        double initSpeed = 0.01;
        double realSpeedLeft = initSpeed;
        double realSpeedRight = initSpeed;
        double speedIncrementLeft = 0.05;
        double speedIncrementRight = speedIncrementLeft;
        double originalRight = profile.getRealSpeedRight();
        double originalLeft = profile.getRealSpeedLeft();
        if (originalLeft > originalRight) {
            speedIncrementRight = speedIncrementLeft * profile.getSpeedRatio();
            realSpeedRight = realSpeedLeft * profile.getSpeedRatio();

        } else if (originalRight > originalLeft) {
            speedIncrementLeft = speedIncrementRight * profile.getSpeedRatio();
            realSpeedLeft = realSpeedRight * profile.getSpeedRatio();
        }


        this.frontLeft.setVelocity(MAX_VELOCITY_GB*realSpeedLeft*mr.getLF());
        this.frontRight.setVelocity(MAX_VELOCITY_GB*realSpeedRight*mr.getRF());
        this.backLeft.setVelocity(MAX_VELOCITY_GB*realSpeedLeft*mr.getLB());
        this.backRight.setVelocity(MAX_VELOCITY_GB*realSpeedRight*mr.getRB());


        while (motorsBusy()){
            //wait for at least one motor to stop
            if (!owner.opModeIsActive()){
                break;
            }
            //check for obstacles
            if (hitObstacle(locator)){
                break;
            }
            realSpeedLeft += speedIncrementLeft;
            realSpeedRight += speedIncrementRight;
            if (realSpeedLeft > originalLeft){
                realSpeedLeft = originalLeft;
            }
            if (realSpeedRight > originalRight){
                realSpeedRight = originalRight;
            }

            this.frontLeft.setVelocity(MAX_VELOCITY_GB*realSpeedLeft*mr.getLF());
            this.frontRight.setVelocity(MAX_VELOCITY_GB*realSpeedRight*mr.getRF());
            this.backLeft.setVelocity(MAX_VELOCITY_GB*realSpeedLeft*mr.getLB());
            this.backRight.setVelocity(MAX_VELOCITY_GB*realSpeedRight*mr.getRB());

        }
        stop();
    }

    protected boolean hitObstacle(IBaseOdometry locator){
        if (locator != null){
            if (runtime == null){
                runtime  = new ElapsedTime();
            }
            if (runtime.milliseconds() > 250) {
                double curX = locator.getCurrentX();
                double curY = locator.getCurrentY();
                double curHead = locator.getAdjustedCurrentHeading();
                if (Math.abs(locator.getPreviousX() - curX) < 1 &&
                        Math.abs(locator.getPreviousY() - curY) < 1 &&
                        Math.abs(locator.getPreviousAdjHeading() - curHead) < 1){
                    Log.d(TAG, String.format("Obstacle hit. X old: %.2f  X new: %.2f", locator.getPreviousX(), curX));
                    Log.d(TAG, String.format("Obstacle hit. Y old: %.2f  Y new: %.2f", locator.getPreviousY(), curY));
                    Log.d(TAG, String.format("Obstacle hit. Heading old: %.2f  new: %.2f", locator.getPreviousAdjHeading(), curHead));
                    return true;
                }
                Log.d(TAG, String.format("Obstacle check. X old: %.2f  X new: %.2f", locator.getPreviousX(), curX));
                Log.d(TAG, String.format("Obstacle check. Y old: %.2f  Y new: %.2f", locator.getPreviousY(), curY));
                Log.d(TAG, String.format("Obstacle check. Heading old: %.2f  new: %.2f", locator.getPreviousAdjHeading(), curHead));
                locator.setPreviousX(curX);
                locator.setPreviousY(curY);
                locator.setPreviousAdjHeading(curHead);
                runtime.reset();
            }
        }
        return false;
    }



    public void moveTo(BotMoveProfile profile) {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            double rightPower = profile.getRealSpeedRight();
            double leftPower = profile.getRealSpeedLeft();

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            MotorReductionBot mr = profile.getMotorReduction();

            boolean forward = profile.getDirection() == RobotDirection.Forward;


            double slowdownMark = profile.getSlowdownMarkLong();

            double leftTarget = profile.getLongTarget();


            double minSpeed = profile.getMinSpeed();

            double speedDropStep = profile.getSpeedDecrement();

            double originalRight = rightPower;


            double speedIncrement = 0.05;
            if (abs(leftPower) > 0.6 && abs(rightPower) > 0.6) {
                speedIncrement = 0.03;
            }

            if (forward) {
                speedIncrement = -speedIncrement;
            }
            leftPower = 0;
            rightPower = 0;

            double realSpeedLeft = leftPower;
            double realSpeedRight = rightPower;


            boolean stop = false;
            boolean slowDown = false;
            int step = 0;
            while (!stop && owner.opModeIsActive()) {
                double leftreading = this.getLeftOdometer();
                if ((forward && leftreading >= slowdownMark) ||
                        (forward == false && leftreading <= slowdownMark)) {

                    if (!slowDown) {
                        slowDown = true;
                    }
                    step++;
                    if (Math.abs(leftPower) <= Math.abs(minSpeed) || Math.abs(rightPower) <= Math.abs(minSpeed)) {
                        stop = (forward && leftreading >= leftTarget) ||
                                (forward == false && leftreading <= leftTarget);
                        if (stop) {
                            break;
                        }
                    }

                    if (forward) {
                        rightPower = realSpeedRight + speedDropStep * step;
                        leftPower = realSpeedLeft + speedDropStep * step;
                        if (rightPower >= -minSpeed || leftPower >= -minSpeed) {
                            leftPower = -minSpeed;
                            rightPower = -minSpeed;
                        }
                    } else {
                        rightPower = realSpeedRight - speedDropStep * step;
                        leftPower = realSpeedLeft - speedDropStep * step;
                        if (rightPower <= minSpeed || leftPower <= minSpeed) {
                            leftPower = minSpeed;
                            rightPower = minSpeed;
                        }
                    }
                } else {
                    //acceleration
                    if ((forward && rightPower + speedIncrement >= originalRight) ||
                            (!forward && rightPower + speedIncrement <= originalRight)) {
                        rightPower = rightPower + speedIncrement;
                        leftPower = rightPower;
                        realSpeedLeft = leftPower;
                        realSpeedRight = rightPower;
                    }
                }

                if (abs(leftPower) > 0.6 && abs(rightPower) > 0.6 && forward) {
                    this.frontLeft.setVelocity(MAX_VELOCITY_GB * leftPower * mr.getLF());
                    this.frontRight.setVelocity(MAX_VELOCITY_GB * rightPower * (mr.getRF() - 0.03));
                    this.backLeft.setVelocity(MAX_VELOCITY_GB * leftPower * mr.getLB());
                    this.backRight.setVelocity(MAX_VELOCITY_GB * rightPower * (mr.getRB() - 0.03));
                } else {
                    this.frontLeft.setVelocity(MAX_VELOCITY_GB * leftPower * mr.getLF());
                    this.frontRight.setVelocity(MAX_VELOCITY_GB * rightPower * mr.getRF());
                    this.backLeft.setVelocity(MAX_VELOCITY_GB * leftPower * mr.getLB());
                    this.backRight.setVelocity(MAX_VELOCITY_GB * rightPower * mr.getRB());
                }

            }

            this.stop();
        }
    }

    @Override
    public void diagTo(BotMoveProfile profile, IBaseOdometry locator) {
        if (backLeft != null && backRight != null && frontLeft != null && frontRight != null) {
            resetEncoders();

            MotorReductionBot calib = profile.getMotorReduction();


            boolean leftAxis = profile.getAngleChange() > 0;
            double power = profile.getTopSpeed();

            this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            Log.d(TAG, String.format("About to Diag at power %.2f Left Axis: %b Left pos: %.2f Right pos: %.2f", power, leftAxis, profile.getLeftTarget(), profile.getRightTarget()));

            if (!leftAxis) {
                this.frontLeft.setTargetPosition((int) profile.getLeftTarget());
                this.backRight.setTargetPosition((int)profile.getRightTargetBack());
                this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.frontLeft.setVelocity(MAX_VELOCITY_GB * power * calib.getLF());
                this.backRight.setVelocity(MAX_VELOCITY_GB * power * calib.getRB());

                this.backLeft.setPower(0);
                this.frontRight.setPower(0);

                while (this.frontLeft.isBusy() && this.backRight.isBusy()){
                    if (!owner.opModeIsActive()){
                        break;
                    }
                    //check for obstacles
                    if (hitObstacle(locator)){
                        break;
                    }
                }
            } else {
                this.backLeft.setTargetPosition((int)profile.getLeftTargetBack());
                this.frontRight.setTargetPosition((int)profile.getRightTarget());
                this.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.backLeft.setVelocity(MAX_VELOCITY_GB * power * calib.getLB());
                this.frontRight.setVelocity(MAX_VELOCITY_GB * power * calib.getRF());

                this.frontLeft.setPower(0);
                this.backRight.setPower(0);

                while (this.backLeft.isBusy() && this.frontRight.isBusy()){
                    if (!owner.opModeIsActive()){
                        break;
                    }
                    //check for obstacles
                    if (hitObstacle(locator)){
                        break;
                    }
                }
            }
            Log.d(TAG, String.format("Moved diag at power %.2f. Left axis: %b.  left pos %d : right pos %d", power, leftAxis, frontLeft.getCurrentPosition(), backRight.getCurrentPosition()));

            stop();
            Log.d(TAG, "Diag to stopped");
        }
        Log.d(TAG, "Diag to exited");
    }

    @Override
    public void initDetectorThread(String side, LinearOpMode caller) {

    }

    @Override
    public void stopDetection() {

    }

    @Override
    public double strafeToCalib(double speed, double inches, boolean left, MotorReductionBot calib, IBaseOdometry locator) {
        resetEncoders();

        int target = (int)(inches * this.getEncoderCountsPerInch()*this.getEncoderDirection());

        if (left){
            this.backLeft.setTargetPosition(target);
            this.backRight.setTargetPosition(-target);
            this.frontLeft.setTargetPosition(-target);
            this.frontRight.setTargetPosition(target);
        }
        else{
            this.backLeft.setTargetPosition(-target);
            this.backRight.setTargetPosition(target);
            this.frontLeft.setTargetPosition(target);
            this.frontRight.setTargetPosition(-target);
        }

        this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.backLeft.setVelocity(MAX_VELOCITY_GB*speed);
        this.backRight.setVelocity(MAX_VELOCITY_GB*speed);
        this.frontLeft.setVelocity(MAX_VELOCITY_GB*speed);
        this.frontRight.setVelocity(MAX_VELOCITY_GB*speed);
        while (motorsBusy()){
            if (!owner.opModeIsActive()){
                break;
            }
            //check for obstacles
            if (hitObstacle(locator)){
                break;
            }
        }
        stop();

        return 0;
    }


    public RobotMovementStats moveToCalib(double leftspeed, double rightspeed, double inches, MotorReductionBot mr, double breakPoint, IBaseOdometry locator) {
        RobotMovementStats stats = new RobotMovementStats();
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            RobotDirection direction = RobotDirection.Forward;
            if (inches < 0){
                direction = RobotDirection.Backward;
            }
            BotMoveProfile profile =
                    BotMoveProfile.buildMoveProfile(this, Math.abs(inches), leftspeed, 0, 0, false, direction, null, -1, -1, locator );
            Log.d(TAG, String.format("Built profile to calib. Distance: %.2f, Direction %s", inches, direction));
            curveTo(profile, locator);
        }
        return stats;
    }


    public void curveTo(BotMoveProfile profile, IBaseOdometry locator) {
        moveToPos(profile, locator);
    }


    public void spin(BotMoveProfile profile, IBaseOdometry locator) {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            spinToPos(profile, locator);
        }
    }

    public void spinToPos(BotMoveProfile profile, IBaseOdometry locator){
        moveToPos(profile, locator);
    }


    public void spinCalib(double degrees, double speed, IBaseOdometry locator) {

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        speed = Math.abs(speed);
        boolean spinLeft = false;
        if (degrees > 0) {
            spinLeft = true;
        }

        double leftDesiredSpeed = speed;
        double rightDesiredSpeed = speed;

        double startHead = locator.getOrientation();


        if (spinLeft) {
            rightDesiredSpeed = -rightDesiredSpeed;
        } else {
            leftDesiredSpeed = -leftDesiredSpeed;
        }

        double slowdownMark = Math.abs(degrees) * 075;


        double leftPower = 0;
        double rightPower = 0;

        double realSpeedLeft = leftPower;
        double realSpeedRight = rightPower;

        double speedIncrement = 0.05;

        boolean stop = false;
        int step = 0;
        double minSpeed = 0.1;

        double speedDropStep = 0.1;
        while (!stop && this.owner.opModeIsActive()) {
            double currentHead = locator.getOrientation();
            double change = Math.abs(currentHead - startHead);
            if (change >= Math.abs(degrees)) {
                stop = true;
            }
            if (!stop) {
                //slow down
                if (change >= slowdownMark) {
                    step++;

                    if (spinLeft) {
                        rightPower = realSpeedRight + speedDropStep * step;
                        leftPower = realSpeedLeft - speedDropStep * step;
                        if (rightPower >= -minSpeed || leftPower <= minSpeed) {
                            leftPower = minSpeed;
                            rightPower = -minSpeed;
                        }
                    } else {
                        rightPower = realSpeedRight - speedDropStep * step;
                        leftPower = realSpeedLeft + speedDropStep * step;
                        if (rightPower <= minSpeed || leftPower >= -minSpeed) {
                            leftPower = -minSpeed;
                            rightPower = minSpeed;
                        }
                    }
                } else {
                    //accelerate
                    if ((spinLeft && leftPower + speedIncrement <= leftDesiredSpeed) ||
                            (!spinLeft && rightPower + speedIncrement <= rightDesiredSpeed)) {
                        if (spinLeft) {
                            leftPower = leftPower + speedIncrement;
                            rightPower = -leftPower;
                        } else {
                            rightPower = rightPower + speedIncrement;
                            leftPower = -rightPower;
                        }
                        realSpeedLeft = leftPower;
                        realSpeedRight = rightPower;
                    }
                }
            }
            this.frontLeft.setVelocity(MAX_VELOCITY_GB *leftPower);
            this.frontRight.setVelocity(MAX_VELOCITY_GB *rightPower);
            this.backLeft.setVelocity(MAX_VELOCITY_GB *leftPower);
            this.backRight.setVelocity(MAX_VELOCITY_GB *rightPower);
        }

        this.stop();
    }

    @Override
    public double getEncoderCountsPerInch() {
        return COUNTS_PER_INCH_GB;
    }

    @Override
    public double getHorizontalOdometer() {
        return 0;
    }

    @Override
    public double getRobotCenterX() {
        return ROBOT_CENTER_X;
    }

    @Override
    public double getRobotCenterY() {
        return ROBOT_CENTER_Y;
    }

    public double getLeftTarget(double inches) {
        return this.getLeftOdometer() + inches * COUNTS_PER_INCH_GB;
    }

    public double getRightTarget(double inches) {
        return this.getRightOdometer() + inches * COUNTS_PER_INCH_GB;
    }


    public void spinLeft(double speed, boolean forward) {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            if (forward) {
                power = -power;
            }

            this.frontLeft.setVelocity(MAX_VELOCITY_GB*-power);
            this.frontRight.setVelocity(MAX_VELOCITY_GB*power);
            this.backLeft.setVelocity(MAX_VELOCITY_GB*-power);
            this.backRight.setVelocity(MAX_VELOCITY_GB*power);
            telemetry.addData("Odo", "Left from %7d", frontLeft.getCurrentPosition());
            telemetry.addData("Odo", "Right from %7d", frontRight.getCurrentPosition());
        }
    }

    public void spinRight(double speed, boolean forward) {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            if (forward) {
                power = -power;
            }

            this.frontLeft.setVelocity(MAX_VELOCITY_GB*power);
            this.frontRight.setVelocity(MAX_VELOCITY_GB*-power);
            this.backLeft.setVelocity(MAX_VELOCITY_GB*power);
            this.backRight.setVelocity(MAX_VELOCITY_GB*-power);
            telemetry.addData("Odo", "Left from %7d", frontLeft.getCurrentPosition());
            telemetry.addData("Odo", "Right from %7d", frontRight.getCurrentPosition());
        }
    }

    @Override
    public void diagToCalib(double speed, double lowSpeed, double diagInches, boolean leftAxis, MotorReductionBot calib, IBaseOdometry locator) {
        if (!leftAxis) {
            this.frontLeft.setVelocity(MAX_VELOCITY_GB * -speed * calib.getLF());
            this.backRight.setVelocity(MAX_VELOCITY_GB * -speed * calib.getRB());

            this.backLeft.setPower(0);
            this.frontRight.setPower(0);

            while (locator.getCurrentX() > 0){

            }
        } else {
            this.backLeft.setVelocity(MAX_VELOCITY_GB * -speed * calib.getLB());
            this.frontRight.setVelocity(MAX_VELOCITY_GB * -speed * calib.getRF());

            this.frontLeft.setPower(0);
            this.backRight.setPower(0);

            while (locator.getCurrentX() < diagInches*2){

            }
        }
        stop();
    }

    public void turnLeft(double speed, boolean forward) {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            if (forward) {
                power = -power;
            }

            this.frontRight.setVelocity(MAX_VELOCITY_GB*power);
            this.backRight.setVelocity(MAX_VELOCITY_GB*power);
            telemetry.addData("Odo", "Left from %7d", frontLeft.getCurrentPosition());
            telemetry.addData("Odo", "Right from %7d", frontRight.getCurrentPosition());
        }
    }

    public void turnRight(double speed, boolean forward) {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            if (forward == true) {
                power = -power;
            }

            this.frontLeft.setVelocity(MAX_VELOCITY_GB*power);
            this.backLeft.setVelocity(MAX_VELOCITY_GB*power);
            telemetry.addData("Odo", "Left from %7d", frontLeft.getCurrentPosition());
            telemetry.addData("Odo", "Right from %7d", frontRight.getCurrentPosition());
        }
    }

    public void strafeLeft(double speed) {
        if (backLeft != null && backRight != null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            MotorReductionBot calib = null;
            calib = getCalibConfig().getStrafeRightReduction();
            power = power * power * power;
            if (calib != null) {
                this.backLeft.setVelocity(MAX_VELOCITY_GB * power * calib.getLB());
                this.backRight.setVelocity(MAX_VELOCITY_GB * -power * calib.getRB());
                this.frontLeft.setVelocity(MAX_VELOCITY_GB * -power * calib.getLF());
                this.frontRight.setVelocity(MAX_VELOCITY_GB * power * calib.getRF());
            }
            else{
                this.backLeft.setVelocity(MAX_VELOCITY_GB*power);
                this.backRight.setVelocity(MAX_VELOCITY_GB*-power);
                this.frontLeft.setVelocity(MAX_VELOCITY_GB*-power);
                this.frontRight.setVelocity(MAX_VELOCITY_GB*power);
            }
            telemetry.addData("Motors", "Front: %.0f", power);
            telemetry.addData("Motors", "Back: %.0f", power);
        }
    }

    public void strafeRight(double speed) {
        if (backLeft != null && backRight != null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            MotorReductionBot calib = null;
            calib = getCalibConfig().getStrafeRightReduction();
            power = power * power * power;
            if (calib != null) {
                this.backLeft.setVelocity(MAX_VELOCITY_GB * -power * calib.getLB());
                this.backRight.setVelocity(MAX_VELOCITY_GB * power * calib.getRB());
                this.frontLeft.setVelocity(MAX_VELOCITY_GB * power * calib.getLF());
                this.frontRight.setVelocity(MAX_VELOCITY_GB * -power * calib.getRF());
            }
            else{
                this.backLeft.setVelocity(MAX_VELOCITY_GB * -power);
                this.backRight.setVelocity(MAX_VELOCITY_GB * power);
                this.frontLeft.setVelocity(MAX_VELOCITY_GB * power);
                this.frontRight.setVelocity(MAX_VELOCITY_GB * -power);
            }
            telemetry.addData("Motors", "Front: %.0f", power);
            telemetry.addData("Motors", "Back: %.0f", power);
        }
    }



    public void diagLeft(double speed) {
        if (backLeft != null && backRight != null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            this.frontLeft.setVelocity(MAX_VELOCITY_GB*power);
            this.backRight.setVelocity(MAX_VELOCITY_GB*power);

            this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.backLeft.setVelocity(0);
            this.frontRight.setVelocity(0);
        }
    }

    public void diagRight(double speed) {
        if (backLeft != null && backRight != null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            this.backLeft.setVelocity(MAX_VELOCITY_GB*power);
            this.frontRight.setVelocity(MAX_VELOCITY_GB*power);

            this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.frontLeft.setVelocity(0);
            this.backRight.setVelocity(0);
        }
    }


    public void initCalibData() throws Exception {
        if (botConfig != null){
            return;
        }
        File calibFile = AppUtil.getInstance().getSettingsFile(BotCalibConfig.BOT_CALIB_CONFIG);
        if (calibFile.exists()) {
            String data = ReadWriteFile.readFile(calibFile);
            botConfig = BotCalibConfig.deserialize(data);
            if (botConfig == null) {
                throw new Exception("Calibration data does not exist. Run calibration first");
            }
            if (botConfig.getPositionPIDF() > 0){
                positionPIDF = botConfig.getPositionPIDF();
                Log.d(TAG, String.format("Setting positionPIDF: %.2f", positionPIDF));

            }

            if (botConfig.getPositionToleration() > 0){
                positionToleration = botConfig.getPositionToleration();
                Log.d(TAG, String.format("Setting positionToleration: %d", positionToleration));
            }

            if (botConfig.getMaxVelocityLF() > 0){
                MAX_VELOCITY_FRONT_LEFT = botConfig.getMaxVelocityLF();
            }

            if (botConfig.getMaxVelocityLB() > 0){
                MAX_VELOCITY_BACK_LEFT = botConfig.getMaxVelocityLB();
            }

            if (botConfig.getMaxVelocityRF() > 0){
                MAX_VELOCITY_FRONT_RIGHT = botConfig.getMaxVelocityRF();
            }

            if (botConfig.getMaxVelocityRB() > 0){
                MAX_VELOCITY_BACK_RIGHT = botConfig.getMaxVelocityRB();
            }

            telemetry.addData("Bot Config", "Initialized");
            telemetry.update();
        } else {
            throw new Exception("Calibration data does not exist. Run calibration first");
        }
    }

    public BotCalibConfig getCalibConfig() {
        if (botConfig == null) {
            File calibFile = AppUtil.getInstance().getSettingsFile(BotCalibConfig.BOT_CALIB_CONFIG);
            if (calibFile.exists()) {
                String data = ReadWriteFile.readFile(calibFile);
                botConfig = BotCalibConfig.deserialize(data);
            }
        }
        return botConfig;
    }

    @Override
    public double getLeftOdometer() {
        return this.frontLeft.getCurrentPosition();
    }

    @Override
    public double getRightOdometer() {
        return this.frontRight.getCurrentPosition();
    }

    @Override
    public double getLeftBackOdometer() {
        return this.backLeft.getCurrentPosition();
    }

    @Override
    public double getRightBackOdometer() {
        return this.backRight.getCurrentPosition();
    }

    public File getCalibConfigFile() {
        return AppUtil.getInstance().getSettingsFile(BotCalibConfig.BOT_CALIB_CONFIG);
    }


    ///gyroscope
    public void initGyro() {
        if (this.gyro == null) {
            this.gyro = new Gyroscope();
        }
        File calibFile = AppUtil.getInstance().getSettingsFile(gyro.CALIB_FILE);
        if (calibFile.exists()) {
            this.gyro.init(this.hwMap, this.telemetry, false);
        } else {
            this.gyro.init(this.hwMap, this.telemetry, true);
            this.gyro.calibrate();
        }
    }

    public void calibrateGyro() {
        if (this.gyro == null) {
            this.gyro = new Gyroscope();
        }

        this.gyro.init(this.hwMap, this.telemetry, true);
        this.gyro.calibrate();

    }

    public double getGyroHeading() {
        if (this.gyro != null) {
            return this.gyro.getHeading();
        }

        return -666;
    }


    public void addNamedCoordinate(AutoDot dot){
        this.namedCoordinates.add(dot);
    }

    public int getEncoderDirection() {
        return encoderDirection;
    }

      public AutoDot getDetectionResult(){
        return null;
      }

    @Override
    public AutoDot getCurrentDetectionResult() {
        return null;
    }


    public void reverseEncoderDirection() {
        if (this.encoderDirection == 1){
            this.encoderDirection = -1;
        }
        else{
            this.encoderDirection = 1;
        }

    }
}
