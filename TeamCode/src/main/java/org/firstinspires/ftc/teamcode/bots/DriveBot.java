package org.firstinspires.ftc.teamcode.bots;

import static java.lang.StrictMath.abs;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

public class DriveBot {
    protected DcMotorEx frontLeft = null;
    protected DcMotorEx frontRight = null;
    protected DcMotorEx backLeft = null;
    protected DcMotorEx backRight = null;

    protected HardwareMap hwMap = null;
    protected Telemetry telemetry;

    private Gyroscope gyro = null;

    private Led led = null;

    protected LinearOpMode owner = null;

    public static final double ROBOT_CENTER_X = 8.25;
    public static final double ROBOT_CENTER_Y = 8.25;

    public double ANTI_GRAVITY_POWER = -0.5;

    public double DRIVE_SPEED = 0.99;

    static final double     STRAFE_COUNT_INCH    =  20;    // Rev Core Hex motor
    public static final double     PIVOT_SPEED = 0.2;

    private static final double MAX_RANGE = 200;

    public static final double ROBOT_LENGTH = 14.8;
    public static final double ROBOT_WIDTH = 15;

    static final double calibration = 1.07;

    static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // Rev Core Hex motor
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP. was 2 in the sample
    static final double     WHEEL_DIAMETER_INCHES   = 2.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH_REV     = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI) * calibration;  //22.64

    private ElapsedTime period  = new ElapsedTime();

    private ElapsedTime runtime = new ElapsedTime();


    private BotCalibConfig botConfig;

    public static String LEFT_FRONT = "frontLeft";
    public static String RIGHT_FRONT = "frontRight";
    public static String LEFT_BACK = "backLeft";
    public static String RIGHT_BACK = "backRight";

    protected ArrayList<AutoDot> namedCoordinates = new ArrayList<>();

    private static final String TAG = "DriveBot";


    public DriveBot() {

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


            if (backLeft != null) {
                backLeft.setDirection(DcMotor.Direction.FORWARD);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (backRight != null) {
                backRight.setDirection(DcMotor.Direction.REVERSE);
                backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (frontLeft != null) {
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
                frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (frontRight != null) {
                frontRight.setDirection(DcMotor.Direction.REVERSE);
                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

            // Set all motors to zero power
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }


    public void move(double drive, double turn){
        if (backLeft != null && backRight!= null && frontLeft != null && frontRight != null) {
            double rightPower = Range.clip(drive + (turn * 0.85), -1.0, 1.0);
            double leftPower = Range.clip(drive - (turn * 0.85), -1.0, 1.0);
            if ((drive > 0 && drive <= 4 )|| (turn > 0 && turn <= 4)){
                rightPower = rightPower * rightPower * rightPower;
                leftPower = leftPower * leftPower * leftPower;
            }
            //use cubic modifier
//        rightPower = rightPower*rightPower*rightPower;
//        leftPower = leftPower*leftPower*leftPower;

            this.backLeft.setPower(leftPower);
            this.backRight.setPower(rightPower);
            this.frontLeft.setPower(leftPower);
            this.frontRight.setPower(rightPower);
            telemetry.addData("Motors", "Left: %.0f", leftPower);
            telemetry.addData("Motors", "Right: %.0f", rightPower);
            telemetry.addData("Motors", "Turn: %.0f", turn);
            telemetry.addData("Motors", "LeftFront from %7d", frontLeft.getCurrentPosition());
            telemetry.addData("Motors", "LeftBack from %7d", backLeft.getCurrentPosition());
            telemetry.addData("Motors", "RightFront from %7d", frontRight.getCurrentPosition());
            telemetry.addData("Motors", "RightBack from %7d", backRight.getCurrentPosition());
        }
    }


    public void strafeLeft(double speed){
        if (backLeft != null && backRight!= null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            power = power * power * power;
            this.backLeft.setPower(power);
            this.backRight.setPower(-power);
            this.frontLeft.setPower(-power);
            this.frontRight.setPower(power);
            telemetry.addData("Motors", "Front: %.0f", power);
            telemetry.addData("Motors", "Back: %.0f", power);
        }
    }

    public void strafeRight(double speed){
        if (backLeft != null && backRight!= null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            power = power * power * power;
            this.backLeft.setPower(-power);
            this.backRight.setPower(power);
            this.frontLeft.setPower(power);
            this.frontRight.setPower(-power);
            telemetry.addData("Motors", "Front: %.0f", power);
            telemetry.addData("Motors", "Back: %.0f", power);
        }
    }

    public void diagLeft(double speed){
        if (backLeft != null && backRight!= null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            this.frontLeft.setPower(power);
            this.backRight.setPower(power);

            this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.backLeft.setPower(0);
            this.frontRight.setPower(0);
        }
    }

    public void diagRight(double speed){
        if (backLeft != null && backRight!= null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            this.backLeft.setPower(power);
            this.frontRight.setPower(power);

            this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.frontLeft.setPower(0);
            this.backRight.setPower(0);
        }
    }

    public void pivotLeft(double speed){
        if (backLeft != null && backRight!= null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            this.backLeft.setPower(-power);
            this.backRight.setPower(power);
            this.frontLeft.setPower(-power);
            this.frontRight.setPower(power);
            telemetry.addData("Motors", "Left: %7d Right: %7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telemetry.update();
        }
    }

    public void pivotRight(double speed){
        if (backLeft != null && backRight!= null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            this.backLeft.setPower(power);
            this.backRight.setPower(-power);
            this.frontLeft.setPower(power);
            this.frontRight.setPower(-power);
            telemetry.addData("Motors", "Lef: %7d Right: %7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telemetry.update();
        }
    }

    public void turnLeft(double speed){
        if (backLeft != null && backRight!= null && frontLeft != null && frontRight != null) {
            this.backLeft.setPower(0);
            this.backRight.setPower(speed);
            this.frontLeft.setPower(0);
            this.frontRight.setPower(speed);
        }
    }

    public void turnRight(double speed){
        if (backLeft != null && backRight!= null && frontLeft != null && frontRight != null) {
            this.backLeft.setPower(speed);
            this.backRight.setPower(0);
            this.frontLeft.setPower(speed);
            this.frontRight.setPower(0);
        }
    }

//    ///gyroscope
//    public Gyro getGyro() {
//        return gyro;
//    }
//
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
//
//    public void encoderDrive(double speed,
//                             double leftInches, double rightInches,
//                             double timeoutMS, LinearOpMode caller) {
//
//        try {
//            // Determine new target position, and pass to motor controller
//            int newLeftTarget = this.backLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
//            int newRightTarget = this.backRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);
//            int newLeftFrontTarget = this.frontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
//            int newRightFrontTarget = this.frontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);
//
//            this.backLeft.setTargetPosition(newLeftTarget);
//            this.backRight.setTargetPosition(newRightTarget);
//            this.frontLeft.setTargetPosition(newLeftFrontTarget);
//            this.frontRight.setTargetPosition(newRightFrontTarget);
//
//
//            // Turn On RUN_TO_POSITION
//            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            ((DcMotorEx)(this.backLeft)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
//            ((DcMotorEx)(this.backRight)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
//            ((DcMotorEx)(this.frontLeft)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
//            ((DcMotorEx)(this.frontRight)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            this.backLeft.setPower(Math.abs(speed));
//            this.backRight.setPower(Math.abs(speed));
//            this.frontLeft.setPower(Math.abs(speed));
//            this.frontRight.setPower(Math.abs(speed));
//
//            boolean stop = false;
//            boolean leftMove = leftInches != 0;
//            boolean rightMove = rightInches != 0;
//            while (!stop) {
//                boolean timeUp = timeoutMS > 0 && runtime.milliseconds() >= timeoutMS;
//                stop = !caller.opModeIsActive() || timeUp || ((leftMove && !this.backLeft.isBusy()) || (rightMove && !this.backRight.isBusy())
//                        || (leftMove && !this.frontLeft.isBusy()) || (rightMove && !this.frontRight.isBusy()));
//
//            }
//
//            this.stop();
//
//
//            // Turn off RUN_TO_POSITION
//            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//        catch (Exception ex){
//            telemetry.addData("Issues running with encoders to position", ex);
//            telemetry.update();
//        }
//    }
//
//    public void encoderStartMove(double speed,
//                                 double leftInches, double rightInches,
//                                 double timeoutMS, LinearOpMode caller) {
//
//        try {
//            // Determine new target position, and pass to motor controller
//            int newLeftTarget = this.backLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
//            int newRightTarget = this.backRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);
//            int newLeftFrontTarget = this.frontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
//            int newRightFrontTarget = this.frontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);
//
//            this.backLeft.setTargetPosition(newLeftTarget);
//            this.backRight.setTargetPosition(newRightTarget);
//            this.frontLeft.setTargetPosition(newLeftFrontTarget);
//            this.frontRight.setTargetPosition(newRightFrontTarget);
//
//
//            // Turn On RUN_TO_POSITION
//            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            ((DcMotorEx)(this.backLeft)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
//            ((DcMotorEx)(this.backRight)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
//            ((DcMotorEx)(this.frontLeft)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
//            ((DcMotorEx)(this.frontRight)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            this.backLeft.setPower(Math.abs(speed));
//            this.backRight.setPower(Math.abs(speed));
//            this.frontLeft.setPower(Math.abs(speed));
//            this.frontRight.setPower(Math.abs(speed));
//        }
//        catch (Exception ex){
//            telemetry.addData("Issues running with encoders to position", ex);
//            telemetry.update();
//        }
//    }
//
//    public void encoderStopMove(){
//
//        this.stop();
//
//
//        // Turn off RUN_TO_POSITION
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public void encoderDriveGyro(double speed,
//                                 double leftInches, double rightInches,
//                                 double timeoutS, LinearOpMode caller) {
//
//        try {
//            // Determine new target position, and pass to motor controller
//            int newLeftTarget = this.backLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
//            int newRightTarget = this.backRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);
//            int newLeftFrontTarget = this.frontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
//            int newRightFrontTarget = this.frontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);
//
//            this.backLeft.setTargetPosition(newLeftTarget);
//            this.backRight.setTargetPosition(newRightTarget);
//            this.frontLeft.setTargetPosition(newLeftFrontTarget);
//            this.frontRight.setTargetPosition(newRightFrontTarget);
//
//
//            // Turn On RUN_TO_POSITION
//            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            ((DcMotorEx)(this.backLeft)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
//            ((DcMotorEx)(this.backRight)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
//            ((DcMotorEx)(this.frontLeft)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
//            ((DcMotorEx)(this.frontRight)).setTargetPositionTolerance((int)COUNTS_PER_INCH_REV);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            this.backLeft.setPower(Math.abs(speed));
//            this.backRight.setPower(Math.abs(speed));
//            this.frontLeft.setPower(Math.abs(speed));
//            this.frontRight.setPower(Math.abs(speed));
//
//            boolean stop = false;
//            boolean leftMove = leftInches != 0;
//            boolean rightMove = rightInches != 0;
//            boolean resume = false;
//            while (!stop) {
//                boolean timeUp = timeoutS > 0 && runtime.seconds() >= timeoutS;
//                stop = !caller.opModeIsActive() || timeUp || ((leftMove && !this.backLeft.isBusy()) || (rightMove && !this.backRight.isBusy())
//                        || (leftMove && !this.frontLeft.isBusy()) || (rightMove && !this.frontRight.isBusy()));
//
//                if(getGyro().isOffCourse()){
//                    backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    getGyro().fixHeading(0.3, caller);
//                    resume = true;
//                }else if (resume){
//                    this.backLeft.setTargetPosition(newLeftTarget);
//                    this.backRight.setTargetPosition(newRightTarget);
//                    this.frontLeft.setTargetPosition(newLeftFrontTarget);
//                    this.frontRight.setTargetPosition(newRightFrontTarget);
//
//
//                    // Turn On RUN_TO_POSITION
//                    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    this.backLeft.setPower(Math.abs(speed));
//                    this.backRight.setPower(Math.abs(speed));
//                    this.frontLeft.setPower(Math.abs(speed));
//                    this.frontRight.setPower(Math.abs(speed));
//                    resume = false;
//                }
//            }
//
//            telemetry.addData("Motors", "Going to stop");
//            telemetry.update();
//            this.stop();
//            telemetry.addData("Motors", "Stopped");
//            telemetry.update();
//
//            // Turn off RUN_TO_POSITION
//            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//        catch (Exception ex){
//            telemetry.addData("Issues running with encoders to position", ex);
//            telemetry.update();
//        }
//    }
//
//
//    public void encoderTurn(double turn, double leftInches, double rightInches, double timeoutS) {
//
//        try {
//            // Determine new target position, and pass to motor controller
//            int newLeftTarget = 0;
//            int newRightTarget = 0;
//            int newLeftFrontTarget = 0;
//            int newRightFrontTarget = 0;
//            if (turn <=0 ) {
//                newLeftTarget = this.backLeft.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH_REV);
//                newRightTarget = this.backRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);
//                newLeftFrontTarget = this.frontLeft.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH_REV);
//                newRightFrontTarget = this.frontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);
//            }
//            else{
//                newLeftTarget = this.backLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
//                newRightTarget = this.backRight.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH_REV);
//                newLeftFrontTarget = this.frontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
//                newRightFrontTarget = this.frontRight.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH_REV);
//            }
//
//            this.backLeft.setTargetPosition(newLeftTarget);
//            this.backRight.setTargetPosition(newRightTarget);
//            this.frontLeft.setTargetPosition(newLeftFrontTarget);
//            this.frontRight.setTargetPosition(newRightFrontTarget);
//
//
//            // Turn On RUN_TO_POSITION
//            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            this.backLeft.setPower(Math.abs(turn));
//            this.backRight.setPower(Math.abs(turn));
//            this.frontLeft.setPower(Math.abs(turn));
//            this.frontRight.setPower(Math.abs(turn));
//
//            boolean stop = false;
//            boolean leftMove = leftInches != 0;
//            boolean rightMove = rightInches != 0;
//            while (!stop) {
//                boolean timeUp = timeoutS > 0 && runtime.seconds() >= timeoutS;
//                stop = timeUp || ((leftMove && !this.backLeft.isBusy()) || (rightMove && !this.backRight.isBusy())
//                        || (leftMove && !this.frontLeft.isBusy()) || (rightMove && !this.frontRight.isBusy()));
//
//                telemetry.addData("Motors", "Starting encoder drive. Left: %.2f, Right:%.2f", leftInches, rightInches);
//                telemetry.addData("Motors", "LeftFront from %7d to %7d", frontLeft.getCurrentPosition(), newLeftFrontTarget);
//                telemetry.addData("Motors", "LeftBack from %7d to %7d", backLeft.getCurrentPosition(), newLeftTarget);
//                telemetry.addData("Motors", "RightFront from %7d to %7d", frontRight.getCurrentPosition(), newRightFrontTarget);
//                telemetry.addData("Motors", "RightBack from %7d to %7d", backRight.getCurrentPosition(), newRightTarget);
//                telemetry.update();
//            }
//
//            telemetry.addData("Motors", "Going to stop");
//            telemetry.update();
//            this.stop();
//            telemetry.addData("Motors", "Stopped");
//            telemetry.update();
//
//            // Turn off RUN_TO_POSITION
//            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//        catch (Exception ex){
//            telemetry.addData("Issues running with encoders to position", ex);
//            telemetry.update();
//        }
//    }
//
//    public int getStrafeIncrement(double moveTo){
//        int increment = (int) (moveTo * STRAFE_COUNT_INCH);
//        return increment;
//    }
//
//    public int getDriveIncrement(double moveTo){
//        return  (int) (moveTo * this.COUNTS_PER_INCH_REV);
//    }
//
//    public double getPositionDiffInches(int start, int finish){
//        return  Math.abs(finish - start)/COUNTS_PER_INCH_REV;
//    }
//    public void encoderStrafe(double speed,
//                              double distanceInches,
//                              double timeoutS) {
//
//        try {
//            double val = Math.abs(distanceInches);
//            int newLeftTarget = 0 ;
//            int newRightTarget = 0;
//            int newLeftFrontTarget = 0;
//            int newRightFrontTarget = 0;
//
//            int increment = (int) (val * STRAFE_COUNT_INCH);
//            if (distanceInches < 0){
//                //going left
//                newLeftTarget = this.backLeft.getCurrentPosition() + increment;
//                newRightTarget = this.backRight.getCurrentPosition() - increment;
//                newLeftFrontTarget = this.frontLeft.getCurrentPosition() - increment;
//                newRightFrontTarget = this.frontRight.getCurrentPosition() + increment;
//            }
//            else{
//                //going right
//                newLeftTarget = this.backLeft.getCurrentPosition() - increment;
//                newRightTarget = this.backRight.getCurrentPosition() + increment;
//                newLeftFrontTarget = this.frontLeft.getCurrentPosition() + increment;
//                newRightFrontTarget = this.frontRight.getCurrentPosition() - increment;
//            }
//
//
//            this.backLeft.setTargetPosition(newLeftTarget);
//            this.backRight.setTargetPosition(newRightTarget);
//            this.frontLeft.setTargetPosition(newLeftFrontTarget);
//            this.frontRight.setTargetPosition(newRightFrontTarget);
//
//
//            // Turn On RUN_TO_POSITION
//            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            this.backLeft.setPower(Math.abs(speed));
//            this.backRight.setPower(Math.abs(speed));
//            this.frontLeft.setPower(Math.abs(speed));
//            this.frontRight.setPower(Math.abs(speed));
//
//            boolean stop = false;
//            while (!stop) {
//                boolean timeUp = timeoutS > 0 && runtime.seconds() >= timeoutS;
//                stop = timeUp || (!this.backLeft.isBusy() || !this.backRight.isBusy()
//                        || !this.frontLeft.isBusy() || !this.frontRight.isBusy());
//                // Display it for the driver.
//                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//                telemetry.addData("Path2", "Back: %7d :%7d front: %7d :%7d",
//                        this.backLeft.getCurrentPosition(),
//                        this.backRight.getCurrentPosition(),
//                        this.frontLeft.getCurrentPosition(),
//                        this.frontRight.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            this.stop();
//
//            // Turn off RUN_TO_POSITION
//            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//        catch (Exception ex){
//            telemetry.addData("Issues running with encoders to position", ex);
//            telemetry.update();
//        }
//    }
//
//    public void encoderPivot(double speed, double inches, double timeoutS) {
//
//        try {
//            double val = Math.abs(inches);
//            int newLeftTarget = 0 ;
//            int newRightTarget = 0;
//            int newLeftFrontTarget = 0;
//            int newRightFrontTarget = 0;
//            if (inches < 0){
//                //pivot left
//                newLeftTarget = this.backLeft.getCurrentPosition() - (int) (val * COUNTS_PER_INCH_REV);
//                newRightTarget = this.backRight.getCurrentPosition() + (int) (val * COUNTS_PER_INCH_REV);
//                newLeftFrontTarget = this.frontLeft.getCurrentPosition() - (int) (val * COUNTS_PER_INCH_REV);
//                newRightFrontTarget = this.frontRight.getCurrentPosition() + (int) (val * COUNTS_PER_INCH_REV);
//            }
//            else{
//                //pivot right
//                newLeftTarget = this.backLeft.getCurrentPosition() + (int) (val * COUNTS_PER_INCH_REV);
//                newRightTarget = this.backRight.getCurrentPosition() - (int) (val * COUNTS_PER_INCH_REV);
//                newLeftFrontTarget = this.frontLeft.getCurrentPosition() + (int) (val * COUNTS_PER_INCH_REV);
//                newRightFrontTarget = this.frontRight.getCurrentPosition() - (int) (val * COUNTS_PER_INCH_REV);
//            }
//
//
//            this.backLeft.setTargetPosition(newLeftTarget);
//            this.backRight.setTargetPosition(newRightTarget);
//            this.frontLeft.setTargetPosition(newLeftFrontTarget);
//            this.frontRight.setTargetPosition(newRightFrontTarget);
//
//
//            // Turn On RUN_TO_POSITION
//            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            this.backLeft.setPower(Math.abs(speed));
//            this.backRight.setPower(Math.abs(speed));
//            this.frontLeft.setPower(Math.abs(speed));
//            this.frontRight.setPower(Math.abs(speed));
//
//            boolean stop = false;
//            while (!stop) {
//                boolean timeUp = timeoutS > 0 && runtime.seconds() >= timeoutS;
//                stop = timeUp || (!this.backLeft.isBusy() || !this.backRight.isBusy()
//                        || !this.frontLeft.isBusy() || !this.frontRight.isBusy());
//                // Display it for the driver.
//                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//                telemetry.addData("Path2", "Back: %7d :%7d front: %7d :%7d",
//                        this.backLeft.getCurrentPosition(),
//                        this.backRight.getCurrentPosition(),
//                        this.frontLeft.getCurrentPosition(),
//                        this.frontRight.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            this.stop();
//
//            // Turn off RUN_TO_POSITION
//            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//        catch (Exception ex){
//            telemetry.addData("Issues running with encoders to position", ex);
//            telemetry.update();
//        }
//    }

}
