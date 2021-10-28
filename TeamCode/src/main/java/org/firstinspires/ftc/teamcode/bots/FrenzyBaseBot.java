package org.firstinspires.ftc.teamcode.bots;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
import org.firstinspires.ftc.teamcode.skills.Led;

import java.io.File;
import java.util.ArrayList;

import static java.lang.StrictMath.abs;

public class FrenzyBaseBot implements OdoBot {

    protected DcMotorEx frontLeft = null;
    protected DcMotorEx frontRight = null;
    protected DcMotorEx backLeft = null;
    protected DcMotorEx backRight = null;

    protected HardwareMap hwMap = null;
    protected Telemetry telemetry;

    protected ElapsedTime runtime;

    private Gyroscope gyro = null;

    private int encoderDirection = 1;


    protected LinearOpMode owner = null;

    static final double COUNTS_PER_MOTOR_GB = 537.7;
    static final double MOTOR_RPM_GB = 312;
    static final double MOTOR_RPS_GB = MOTOR_RPM_GB/60;
    public static final double MAX_VELOCITY_GB = COUNTS_PER_MOTOR_GB * MOTOR_RPS_GB;
    public static final double MAX_VELOCITY_REV = 2140;

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
            // Define and Initialize Motors
            frontLeft = hwMap.get(DcMotorEx.class, LEFT_FRONT);
            frontRight = hwMap.get(DcMotorEx.class, RIGHT_FRONT);
            backLeft = hwMap.get(DcMotorEx.class, LEFT_BACK);
            backRight = hwMap.get(DcMotorEx.class, RIGHT_BACK);

            resetEncoders();
            this.encoderDirection = -1;


            if (backLeft != null) {
                backLeft.setDirection(DcMotor.Direction.FORWARD);
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (backRight != null) {
                backRight.setDirection(DcMotor.Direction.REVERSE);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (frontLeft != null) {
                frontLeft.setDirection(DcMotor.Direction.REVERSE);
                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (frontRight != null) {
                frontRight.setDirection(DcMotor.Direction.FORWARD);
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            stop();
        } catch (Exception ex) {
            //issues accessing drive resources
            throw new Exception("Issues accessing one of drive motors. Check the controller config", ex);
        }
    }

    public Telemetry getTelemetry() {
        return this.telemetry;
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
        }
    }



    public double getLeftVelocity() {
        return frontLeft.getVelocity();
    }

    public double getRightVelocity() {
        return frontRight.getVelocity();
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

    public void moveToPos2(BotMoveProfile profile, IBaseOdometry locator){
        resetEncoders();
        this.frontLeft.setTargetPosition((int)profile.getLeftTarget());
        this.frontRight.setTargetPosition((int)profile.getRightTarget());


        this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MotorReductionBot mr = profile.getMotorReduction();

        this.frontRight.setVelocity(MAX_VELOCITY_GB*profile.getRealSpeedLeft()*mr.getLF());
        this.frontRight.setVelocity(MAX_VELOCITY_GB*profile.getRealSpeedRight()*mr.getRF());


        runtime.reset();
        while ( this.frontLeft.isBusy() && this.frontRight.isBusy()){
            this.backLeft.setVelocity(this.frontLeft.getVelocity());
            this.backRight.setVelocity(this.frontRight.getVelocity());
            if (runtime.milliseconds() > 500){
                locator.setTarget(profile.getTarget());
                this.frontLeft.setTargetPosition((int)profile.getLeftTarget());
                this.frontRight.setTargetPosition((int)profile.getRightTarget());
                this.frontLeft.setVelocity(MAX_VELOCITY_GB*profile.getRealSpeedLeft()*mr.getLF());
                this.frontRight.setVelocity(MAX_VELOCITY_GB*profile.getRealSpeedRight()*mr.getRF());
                runtime.reset();
            }
        }
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

        this.frontRight.setVelocity(MAX_VELOCITY_GB*profile.getRealSpeedLeft()*mr.getLF());
        this.backLeft.setVelocity(MAX_VELOCITY_GB*profile.getRealSpeedLeft()*mr.getLF());
        this.frontRight.setVelocity(MAX_VELOCITY_GB*profile.getRealSpeedRight()*mr.getRF());
        this.backRight.setVelocity(MAX_VELOCITY_GB*profile.getRealSpeedRight()*mr.getRF());
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
    public void diagTo(BotMoveProfile profile) {
        if (backLeft != null && backRight != null && frontLeft != null && frontRight != null) {

            MotorReductionBot calib = profile.getMotorReduction();

            boolean leftAxis = profile.getAngleChange() > 0;
            double power = profile.getTopSpeed();

            this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            if (!leftAxis) {

                this.frontLeft.setTargetPosition((int) profile.getLeftTarget());
                this.backRight.setTargetPosition((int)profile.getRightTargetBack());
                this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.frontLeft.setVelocity(MAX_VELOCITY_GB * power * calib.getLF());
                this.backRight.setVelocity(MAX_VELOCITY_GB * power * calib.getRB());

                this.backLeft.setVelocity(0);
                this.frontRight.setVelocity(0);
            } else {
                this.backLeft.setTargetPosition((int)profile.getLeftTargetBack());
                this.frontRight.setTargetPosition((int)profile.getRightTarget());
                this.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.backLeft.setVelocity(MAX_VELOCITY_GB * power * calib.getLB());
                this.frontRight.setVelocity(MAX_VELOCITY_GB * power * calib.getRF());

                this.frontLeft.setVelocity(0);
                this.backRight.setVelocity(0);
            }

        }

    }

    @Override
    public void initDetectorThread(String side, LinearOpMode caller) {

    }

    @Override
    public void stopDetection() {

    }

    @Override
    public double strafeToCalib(double speed, double inches, boolean left, MotorReductionBot calib) {
        return 0;
    }


    public RobotMovementStats moveToCalib(double leftspeed, double rightspeed, double inches, MotorReductionBot mr, double breakPoint) {
        RobotMovementStats stats = new RobotMovementStats();
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            double rightPower = rightspeed;
            double leftPower = leftspeed;
            stats.setMotorPower(Math.abs(leftPower));
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            boolean forward = inches > 0;

            //reverse speed
            if (forward) {
                rightPower = -rightPower;
                leftPower = -leftPower;
            } else {
                breakPoint = -breakPoint;
            }

            double distance = inches * COUNTS_PER_INCH_GB;

            double startingPoint = this.getLeftOdometer();


            double slowdownMark = startingPoint + (distance - breakPoint);

            double leftTarget = startingPoint + distance;


            double minSpeed = 0.1;

            double speedDropStep = 0.05;

            double originalRight = rightPower;
            double originalLeft = leftPower;


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

            boolean fullSpeedReached = false;

            stats.startAccelerateTimer(startingPoint);

            boolean stop = false;
            boolean slowDown = false;
            int step = 0;
            while (!stop && owner.opModeIsActive()) {
                double leftreading = this.getLeftOdometer();
                if ((forward && leftreading >= slowdownMark) ||
                        (forward == false && leftreading <= slowdownMark)) {

                    if (!slowDown) {
                        if (fullSpeedReached) {
                            fullSpeedReached = false;
                            stats.stopFullSpeedTimer(leftreading);
                        } else {
                            stats.stopAccelerateTimer(leftreading);
                        }
                        stats.startSlowDownTimer(leftreading, slowdownMark);
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
                    } else {
                        //full speed
                        if (!fullSpeedReached) {
                            fullSpeedReached = true;
                            stats.stopAccelerateTimer(leftreading);
                            stats.startFullSpeedTimer(leftreading);
                        }
                    }
                }

                if (abs(leftPower) > 0.6 && abs(rightPower) > 0.6 && forward) {
                    this.frontLeft.setVelocity(MAX_VELOCITY_GB * leftPower * mr.getLF());
                    this.frontRight.setVelocity(MAX_VELOCITY_GB * rightPower * (mr.getRF() - 0.04));
                    this.backLeft.setVelocity(MAX_VELOCITY_GB * leftPower * mr.getLB());
                    this.backRight.setVelocity(MAX_VELOCITY_GB * rightPower * (mr.getRB() - 0.04));
                } else {
                    this.frontLeft.setVelocity(MAX_VELOCITY_GB * leftPower * mr.getLF());
                    this.frontRight.setVelocity(MAX_VELOCITY_GB * rightPower * mr.getRF());
                    this.backLeft.setVelocity(MAX_VELOCITY_GB * leftPower * mr.getLB());
                    this.backRight.setVelocity(MAX_VELOCITY_GB * rightPower * mr.getRB());
                }

                stats.updateVelocity(this.getLeftVelocity(), this.getRightVelocity());
            }
            stats.stopSlowdownTimer(this.getLeftOdometer());

            stats.computeTotals(this.getLeftOdometer());

            this.stop();
        }
        return stats;
    }


    public void curveTo(BotMoveProfile profile, IBaseOdometry locator) {
        moveToPos(profile, locator);
    }


    public void spin(BotMoveProfile profile, IBaseOdometry locator) {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            double speed = Math.abs(profile.getTopSpeed());
            double degrees = profile.getAngleChange();
            boolean spinLeft = false;
            if (degrees > 0) {
                spinLeft = true;
            }

            double leftDesiredSpeed = speed;
            double rightDesiredSpeed = speed;

            double startHead = locator.getOrientation();
            Log.d(TAG, String.format("Bot Spin. Orientation: %.2f. Degrees Raw: %.2f", startHead, degrees));


            MotorReductionBot reduction;
            if (spinLeft) {
                rightDesiredSpeed = -rightDesiredSpeed;
                reduction = botConfig.getSpinLeftConfig();
            } else {
                leftDesiredSpeed = -leftDesiredSpeed;
                reduction = botConfig.getSpinRightConfig();
            }

            double desired = Math.abs(degrees);
            double slowdownMark = Math.abs(degrees) * reduction.getBreakPoint(profile.getTopSpeed());
            //adjust slow down by 100ms to account for processing time
            double ticksAdjustment = MAX_VELOCITY_GB*profile.getTopSpeed();
            double ticksAdjustmentDegrees = ticksAdjustment/botConfig.getHorizontalTicksDegree();
            slowdownMark = slowdownMark - ticksAdjustmentDegrees;
            desired = desired - ticksAdjustmentDegrees;


            if (!profile.shouldStop()) {
                 desired = slowdownMark;
            }


            double leftPower = 0;
            double rightPower = 0;

            double realSpeedLeft = leftPower;
            double realSpeedRight = rightPower;

            double speedIncrement = profile.getSpeedIncrement();

            boolean stop = false;
            int step = 0;
            double minSpeed = profile.getMinSpeedSpin();

            double speedDropStep = profile.getSpeedDecrement();
            while (!stop && this.owner.opModeIsActive()) {
                double currentHead = locator.getOrientation();
                double change = Math.abs(currentHead - startHead);
                if (change >= desired) {
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

            if (profile.shouldStop()) {
                this.stop();
            }
        }
    }

    public void spinToPos(BotMoveProfile profile){
        moveToPos(profile, null);
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
    public void diagToCalib(double speed, double lowSpeed, double diagInches, boolean leftAxis, MotorReductionBot calib) {

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
        File calibFile = AppUtil.getInstance().getSettingsFile(BotCalibConfig.BOT_CALIB_CONFIG);
        if (calibFile.exists()) {
            String data = ReadWriteFile.readFile(calibFile);
            botConfig = BotCalibConfig.deserialize(data);
            if (botConfig == null) {
                throw new Exception("Calibration data does not exist. Run calibration first");
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

    public void reverseEncoderDirection() {
        if (this.encoderDirection == 1){
            this.encoderDirection = -1;
        }
        else{
            this.encoderDirection = 1;
        }

    }
}
