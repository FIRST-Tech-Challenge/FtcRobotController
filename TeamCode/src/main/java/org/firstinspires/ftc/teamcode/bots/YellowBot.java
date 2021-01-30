package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Point;

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
import org.firstinspires.ftc.teamcode.calibration.DiagCalibConfig;
import org.firstinspires.ftc.teamcode.calibration.MotorReductionBot;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;
import org.firstinspires.ftc.teamcode.skills.Geometry;
import org.firstinspires.ftc.teamcode.skills.Gyroscope;
import org.firstinspires.ftc.teamcode.skills.Led;
//import org.openftc.revextensions2.ExpansionHubEx;
//import org.openftc.revextensions2.ExpansionHubMotor;

import java.io.File;
import java.util.ArrayList;
import java.util.Random;

public class YellowBot implements OdoBot {
    public static double CALIB_SPEED = 0.5;
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    protected DcMotorEx leftOdo = null; //leftodo port 1 ext hub    forward positive
    protected DcMotorEx rightOdo = null; // rightodo - intake port 0 control hub     forw negative
    protected DcMotorEx horizontalOdo = null; // horizontal port 0 ext hub    right neg
    private DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;

    protected HardwareMap hwMap = null;
    protected Telemetry telemetry;

    private Gyroscope gyro = null;

    private Led led = null;

    protected LinearOpMode owner = null;

    static final double COUNTS_PER_MOTOR_HD = 560;    // Rev HD motor 537.6
    static final double REV_TBORE = 8192;    // Rev HD motor
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP. was 2 in the sample
    static final double WHEEL_DIAMETER_INCHES = 2.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH_REV = (REV_TBORE * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);  //1303.79729
    public static final double MAX_VELOCITY = 2140;
    public static final  double PROC_DELAY = 7; // 142ms. todo: infer from calibration
    public static final double MAX_VELOCITY_PER_PROC_DELAY = MAX_VELOCITY / PROC_DELAY;

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


    public YellowBot() {

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

            horizontalOdo = hwMap.get(DcMotorEx.class, "horizontal");
            leftOdo = hwMap.get(DcMotorEx.class, "leftodo");
            rightOdo = hwMap.get(DcMotorEx.class, "rightodo");

            resetEncoders();


            if (backLeft != null) {
                backLeft.setDirection(DcMotor.Direction.FORWARD);
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (backRight != null) {
                backRight.setDirection(DcMotor.Direction.REVERSE);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (frontLeft != null) {
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (frontRight != null) {
                frontRight.setDirection(DcMotor.Direction.REVERSE);
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (horizontalOdo != null){
                horizontalOdo.setDirection(DcMotor.Direction.REVERSE);
            }

            if (leftOdo != null){
                horizontalOdo.setDirection(DcMotor.Direction.FORWARD);
            }

            if (rightOdo != null){
                horizontalOdo.setDirection(DcMotor.Direction.FORWARD);
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

    public void setDriveToPowerMode(){
        if (backLeft != null) {
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (backRight != null) {
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (frontLeft != null) {
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (frontRight != null) {
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


    protected void resetEncoders() {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (leftOdo != null && rightOdo != null && horizontalOdo != null){
            leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    public double getOdometerReading(boolean left) {
        if (left) {
            return leftOdo.getCurrentPosition();
        } else {
            return rightOdo.getCurrentPosition();
        }
    }

    public double getLeftOdometer() {
        return leftOdo.getCurrentPosition();

    }

    public double getRightOdometer() {
        return -rightOdo.getCurrentPosition();
    }

    public double getHorizontalOdometer() {
        return  horizontalOdo.getCurrentPosition();

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

            this.frontLeft.setPower(leftPower);
            this.frontRight.setPower(rightPower);
            this.backLeft.setPower(leftPower);
            this.backRight.setPower(rightPower);
        }
    }


    public MotorReductionBot getSuggestedMR(MotorReductionBot amperage) {

        MotorReductionBot recommended = new MotorReductionBot();

        double minVal = 100;
        if (minVal > amperage.getRB()) {
            minVal = amperage.getRB();
        }
        if (minVal > amperage.getRF()) {
            minVal = amperage.getRF();
        }
        if (minVal > amperage.getLB()) {
            minVal = amperage.getLB();
        }

        if (minVal > amperage.getLF()) {
            minVal = amperage.getLF();
        }

        recommended.setRB(minVal / amperage.getRB());
        recommended.setRF(minVal / amperage.getRF());
        recommended.setLB(minVal / amperage.getLB());
        recommended.setLF(minVal / amperage.getLF());

        return recommended;
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


            double minSpeed = 0.1;

            double speedDropStep = 0.05;

            double originalRight = rightPower;


            double speedIncrement = 0.05;
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

                this.frontLeft.setVelocity(MAX_VELOCITY * leftPower * mr.getLF());
                this.frontRight.setVelocity(MAX_VELOCITY * rightPower * mr.getRF());
                this.backLeft.setVelocity(MAX_VELOCITY * leftPower * mr.getLB());
                this.backRight.setVelocity(MAX_VELOCITY * rightPower * mr.getRB());

            }

            this.stop();
        }
    }


    public RobotMovementStats moveToCalib(double leftspeed, double rightspeed, double inches, MotorReductionBot mr, double breakPoint, Led led) {
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

            double distance = inches * COUNTS_PER_INCH_REV;

            double startingPoint = this.getLeftOdometer();


            double slowdownMark = startingPoint + (distance - breakPoint);

            double leftTarget = startingPoint + distance;


            double minSpeed = 0.1;

            double speedDropStep = 0.05;

            double originalRight = rightPower;
            double originalLeft = leftPower;


            double speedIncrement = 0.05;
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

                this.frontLeft.setVelocity(MAX_VELOCITY * leftPower * mr.getLF());
                this.frontRight.setVelocity(MAX_VELOCITY * rightPower * mr.getRF());
                this.backLeft.setVelocity(MAX_VELOCITY * leftPower * mr.getLB());
                this.backRight.setVelocity(MAX_VELOCITY * rightPower * mr.getRB());

                stats.updateVelocity(this.getLeftVelocity(), this.getRightVelocity());
            }
            stats.stopSlowdownTimer(this.getLeftOdometer());

            stats.computeTotals(this.getLeftOdometer());

            this.stop();
        }
        return stats;
    }


    public void curveTo(BotMoveProfile profile, RobotCoordinatePosition locator) {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {

            MotorReductionBot mr = profile.getMotorReduction();

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            boolean forward = profile.getDirection() == RobotDirection.Forward;
            boolean leftLong = profile.isLeftLong();


            double startPower = 0;
            double speedIncrementLeft = 0.05;
            double speedIncrementRight = speedIncrementLeft;

            double slowdownMarkLong = profile.getSlowdownMarkLong();
            double slowdownMarkShort = profile.getSlowdownMarkShort();

            double longTarget = profile.getLongTarget();


            double minSpeed = 0.1;

            double lastMileSpeed = 0.2;

            double speedDropStep = 0.05;

            double originalRight = profile.getRealSpeedRight();
            double originalLeft = profile.getRealSpeedLeft();

            if (originalLeft > originalRight) {
                speedIncrementRight = speedIncrementLeft * profile.getSpeedRatio();
                startPower = originalRight / 2;
            } else if (originalRight > originalLeft) {
                leftLong = false;
                speedIncrementLeft = speedIncrementRight * profile.getSpeedRatio();
                startPower = originalLeft / 2;
            }


            if (forward) {
                speedIncrementLeft = -speedIncrementLeft;
                speedIncrementRight = -speedIncrementRight;
            }


            double realSpeedLF = startPower;
            double realSpeedLB = startPower;
            double realSpeedRF = startPower;
            double realSpeedRB = startPower;

            if (profile.shouldStop() == false) {
                //will terminate earlier, at the slowdown point, to start the next move
                longTarget = slowdownMarkLong;
            }


            boolean accelerating = true;
            boolean slowingDown = false;
            boolean cruising = false;

            boolean stop = false;
            while (!stop && owner.opModeIsActive()) {
                double longReading = leftLong == true ? this.getLeftOdometer() : this.getRightOdometer();
                double shortReading = leftLong == true ? this.getRightOdometer() : this.getLeftOdometer();

                slowingDown = (forward && (longReading >= slowdownMarkLong || shortReading >= slowdownMarkShort)) ||
                        (forward == false && (longReading <= slowdownMarkLong || shortReading <= slowdownMarkShort));
                if (slowingDown) {
                    //slowing down
//                    telemetry.addData("longReading", longReading);
//                    telemetry.addData("shortReading", shortReading);
//                    telemetry.addData("leftLong", leftLong);
//                    telemetry.addData("longTarget", longTarget);
//                    telemetry.addData("realSpeedRF", realSpeedRF);
//                    telemetry.addData("realSpeedRB", realSpeedRB);
//                    telemetry.addData("realSpeedLF", realSpeedLF);
//                    telemetry.addData("realSpeedLB", realSpeedLB);
//                    telemetry.update();

                    cruising = false;
                    //stop course correction
                    locator.setTarget(null);
                    if (forward) {
                        realSpeedRF = realSpeedRF + speedDropStep;
                        realSpeedRB = realSpeedRB + speedDropStep;
                        realSpeedLF = realSpeedLF + speedDropStep;
                        realSpeedLB = realSpeedLB + speedDropStep;
                        if (realSpeedRF >= -minSpeed || realSpeedRB >= -minSpeed
                                || realSpeedLF >= -minSpeed || realSpeedLB >= -minSpeed) {
                            realSpeedRF = -lastMileSpeed;
                            realSpeedRB = -lastMileSpeed;
                            realSpeedLF = -lastMileSpeed;
                            realSpeedLB = -lastMileSpeed;
                            stop = longReading >= longTarget;
                            if (stop) {
                                break;
                            }
                        }
                    } else {
                        realSpeedRF = realSpeedRF - speedDropStep;
                        realSpeedRB = realSpeedRB - speedDropStep;
                        realSpeedLF = realSpeedLF - speedDropStep;
                        realSpeedLB = realSpeedLB - speedDropStep;
                        if (realSpeedRF <= minSpeed || realSpeedRB <= minSpeed
                                || realSpeedLF <= minSpeed || realSpeedLB <= minSpeed) {
                            realSpeedRF = lastMileSpeed;
                            realSpeedRB = lastMileSpeed;
                            realSpeedLF = lastMileSpeed;
                            realSpeedLB = lastMileSpeed;
                            stop = longReading <= longTarget;
                            if (stop) {
                                break;
                            }
                        }
                    }

                } else if (accelerating) {
                    if ((forward && realSpeedRF + speedIncrementRight >= originalRight && realSpeedLF + speedIncrementLeft >= originalLeft) ||
                            (!forward && realSpeedRF + speedIncrementRight <= originalRight && realSpeedLF + speedIncrementLeft <= originalLeft)) {
                        accelerating = true;
                        realSpeedRF += speedIncrementRight;
                        realSpeedRB += speedIncrementRight;
                        realSpeedLF += speedIncrementLeft;
                        realSpeedLB += speedIncrementLeft;

                    } else {
                        if (!cruising) {
                            accelerating = false;
                            cruising = true;
                            realSpeedRF = originalRight;
                            realSpeedRB = originalRight;
                            realSpeedLF = originalLeft;
                            realSpeedLB = originalLeft;
                            //start course adjustment
                            locator.setTarget(profile.getTarget());
                        }
                    }
                }
                if (cruising) {
                    //adjust left and right speeds based on the locator
                    double adjustedLeft = locator.getRealSpeedLeft();
                    double adjustedRight = locator.getRealSpeedRight();
                    if (Math.abs(adjustedLeft) > 0 && Math.abs(adjustedRight) > 0) {
                        realSpeedRF = adjustedRight;
                        realSpeedRB = adjustedRight;
                        realSpeedLF = adjustedLeft;
                        realSpeedLB = adjustedLeft;
                        telemetry.addData("Adj Left", adjustedLeft);
                        telemetry.addData("Adj Right", adjustedRight);
                        leftLong = locator.isLeftLong();
//                        slowdownMarkLong = locator.getSlowdownMarkLong();
//                        slowdownMarkShort = locator.getSlowdownMarkShort();
//                        longTarget = locator.getLongTarget();
                        telemetry.addData("LeftLong", leftLong);
//                        telemetry.addData("SlowdownMarkLong", slowdownMarkLong);
//                        telemetry.addData("SlowdownMarkShort", slowdownMarkShort);
//                        telemetry.addData("longTarget", longTarget);
                    }
                }

                this.frontLeft.setVelocity(MAX_VELOCITY *realSpeedLF * mr.getLF());
                this.frontRight.setVelocity(MAX_VELOCITY *realSpeedRF * mr.getRF());
                this.backLeft.setVelocity(MAX_VELOCITY *realSpeedLB * mr.getLB());
                this.backRight.setVelocity(MAX_VELOCITY *realSpeedRB * mr.getRB());
            }

            if (profile.shouldStop()) {
                this.stop();
            }
        }
    }

    public void spin(BotMoveProfile profile, RobotCoordinatePosition locator) {
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
            double ticksAdjustment = MAX_VELOCITY_PER_PROC_DELAY*profile.getTopSpeed();
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
            double minSpeed = profile.getMinSpeed();

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
                this.frontLeft.setVelocity(MAX_VELOCITY *leftPower);
                this.frontRight.setVelocity(MAX_VELOCITY *rightPower);
                this.backLeft.setVelocity(MAX_VELOCITY *leftPower);
                this.backRight.setVelocity(MAX_VELOCITY *rightPower);
            }

            if (profile.shouldStop()) {
                this.stop();
            }
        }
    }


    public void spinCalib(double degrees, double speed, RobotCoordinatePosition locator) {

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
            this.frontLeft.setVelocity(MAX_VELOCITY *leftPower);
            this.frontRight.setVelocity(MAX_VELOCITY *rightPower);
            this.backLeft.setVelocity(MAX_VELOCITY *leftPower);
            this.backRight.setVelocity(MAX_VELOCITY *rightPower);
        }

        this.stop();
    }

    public double getLeftTarget(double inches) {
        return this.getLeftOdometer() + inches * COUNTS_PER_INCH_REV;
    }

    public double getRightTarget(double inches) {
        return this.getRightOdometer() + inches * COUNTS_PER_INCH_REV;
    }


    public void spinLeft(double speed, boolean forward) {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            if (forward) {
                power = -power;
            }

            this.frontLeft.setPower(-power);
            this.frontRight.setPower(power);
            this.backLeft.setPower(-power);
            this.backRight.setPower(power);
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

            this.frontLeft.setPower(power);
            this.frontRight.setPower(-power);
            this.backLeft.setPower(power);
            this.backRight.setPower(-power);
            telemetry.addData("Odo", "Left from %7d", frontLeft.getCurrentPosition());
            telemetry.addData("Odo", "Right from %7d", frontRight.getCurrentPosition());
        }
    }

    public void spinLeftDegrees(double speed, double degrees, boolean forward) {
        File leftSpinPerDegFile = AppUtil.getInstance().getSettingsFile("leftSpinPerDeg.txt");
        File rightSpinPerDegFile = AppUtil.getInstance().getSettingsFile("rightSpinPerDeg.txt");

        double leftPerDegree = Double.parseDouble(ReadWriteFile.readFile(leftSpinPerDegFile).trim());
        double rightPerDegree = Double.parseDouble(ReadWriteFile.readFile(rightSpinPerDegFile).trim());

        double targetPos = this.getLeftOdometer() + leftPerDegree * degrees;

        double power = Range.clip(speed, -1.0, 1.0);

        if (forward) {
            power = -power;
        }

        this.frontLeft.setPower(-power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(-power);
        this.backRight.setPower(power);
        while (this.getLeftOdometer() > targetPos && owner.opModeIsActive()) {

        }

        this.stop();
    }


    public void turnLeft(double speed, boolean forward) {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            if (forward) {
                power = -power;
            }

            this.frontRight.setPower(power);
            this.backRight.setPower(power);
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

            this.frontLeft.setPower(power);
            this.backLeft.setPower(power);
            telemetry.addData("Odo", "Left from %7d", frontLeft.getCurrentPosition());
            telemetry.addData("Odo", "Right from %7d", frontRight.getCurrentPosition());
        }
    }

    public void strafeLeft(double speed) {
        if (backLeft != null && backRight != null && frontLeft != null && frontRight != null) {
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

    public void strafeRight(double speed) {
        if (backLeft != null && backRight != null && frontLeft != null && frontRight != null) {
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

    public double strafeTo(double speed, double inches, boolean left) {
        double currentPos = this.getHorizontalOdometer();
        double distance = inches * COUNTS_PER_INCH_REV;

        MotorReductionBot calib = null;
        if (left) {
            calib = getCalibConfig().getStrafeLeftReduction();
        } else {
            calib = getCalibConfig().getStrafeRightReduction();
        }


        double overage = 0;

        if (left == false) {
            distance = -distance;
        }

        double target = currentPos + distance;

        boolean stop = false;

        while (!stop && this.owner.opModeIsActive()) {
            currentPos = this.getHorizontalOdometer();
            if ((left && currentPos >= target) || (left == false && currentPos <= target)) {
                stop = true;
            }

            if (left) {
                this.backLeft.setPower(-speed * calib.getLB());
                this.backRight.setPower(speed * calib.getRB());
                this.frontLeft.setPower(speed * calib.getLF());
                this.frontRight.setPower(-speed * calib.getRF());
            } else {
                this.backLeft.setPower(speed * calib.getLB());
                this.backRight.setPower(-speed * calib.getRB());
                this.frontLeft.setPower(-speed * calib.getLF());
                this.frontRight.setPower(speed * calib.getRF());
            }
        }

        stop();
        double newPos = this.getHorizontalOdometer();
        double diff = Math.abs(newPos - target);
        overage = diff / distance * 100;
        return overage;
    }

    public double strafeToCalib(double speed, double inches, boolean left, MotorReductionBot calib) {
        double currentPos = this.getHorizontalOdometer();
        double distance = inches * COUNTS_PER_INCH_REV;


        double overage = 0;

        if (left == false) {
            distance = -distance;
        }

        double target = currentPos + distance;

        boolean stop = false;

        while (!stop && this.owner.opModeIsActive()) {
            currentPos = this.getHorizontalOdometer();
            if ((left && currentPos >= target) || (left == false && currentPos <= target)) {
                stop = true;
            }

            if (left) {
                this.backLeft.setVelocity(MAX_VELOCITY *-speed * calib.getLB());
                this.backRight.setVelocity(MAX_VELOCITY *speed * calib.getRB());
                this.frontLeft.setVelocity(MAX_VELOCITY *speed * calib.getLF());
                this.frontRight.setVelocity(MAX_VELOCITY *-speed * calib.getRF());
            } else {
                this.backLeft.setVelocity(MAX_VELOCITY *speed * calib.getLB());
                this.backRight.setVelocity(MAX_VELOCITY *-speed * calib.getRB());
                this.frontLeft.setVelocity(MAX_VELOCITY *-speed * calib.getLF());
                this.frontRight.setVelocity(MAX_VELOCITY *speed * calib.getRF());
            }
        }

        stop();
        double newPos = this.getHorizontalOdometer();
        double diff = Math.abs(newPos - target);
        overage = diff / distance * 100;
        return overage;
    }


    public void diagLeft(double speed) {
        if (backLeft != null && backRight != null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            this.frontLeft.setPower(power);
            this.backRight.setPower(power);

            this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.backLeft.setPower(0);
            this.frontRight.setPower(0);
        }
    }

    public void diagRight(double speed) {
        if (backLeft != null && backRight != null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);

            this.backLeft.setPower(power);
            this.frontRight.setPower(power);

            this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.frontLeft.setPower(0);
            this.backRight.setPower(0);
        }
    }

    public void diagToCalib(double speed, double lowSpeed, double diagInches, boolean leftAxis, MotorReductionBot calib) {
        if (backLeft != null && backRight != null && frontLeft != null && frontRight != null) {


            double leftOdoStart = getLeftOdometer();
            double rightOdoStart = getRightOdometer();
            double horOdoStart = getHorizontalOdometer();

            double distance = Math.abs(diagInches * COUNTS_PER_INCH_REV);

//            double horDistance = distance * Math.sin(Math.toRadians(targetAngle));
//            double verDistance = distance * Math.cos(Math.toRadians(targetAngle));
//
//
//
//            boolean angleChange = lowSpeed > 0;

            double power = speed;

            if (diagInches > 0) {
                power = -power;
                lowSpeed = -lowSpeed;
            }

            boolean stop = false;


            while (!stop && owner.opModeIsActive()) {
                double leftOdo = getLeftOdometer();
                double rightOdo = getRightOdometer();
                double horOdo = getHorizontalOdometer();
                double leftDistActual = Math.abs(leftOdo - leftOdoStart);
                double rightDistActual = Math.abs(rightOdo - rightOdoStart);
                double horDistActual = Math.abs(horOdo - horOdoStart);

                double catet = leftDistActual;
                if (rightDistActual > catet) {
                    catet = rightDistActual;
                }

                double hyp = Math.sqrt(catet * catet + horDistActual * horDistActual);

                if (hyp >= distance) {
                    break;
                }

//                if (angleChange) {
//                    if (hyp >= distance) {
//                        break;
//                    }
//                }else {
//                    if (leftDistActual >= verDistance || rightDistActual >= verDistance || horDistActual >= horDistance) {
//                        break;
//                    }
//                }

                this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


                if (!leftAxis) {
                    this.frontLeft.setVelocity(MAX_VELOCITY *power * calib.getLF());
                    this.backRight.setVelocity(MAX_VELOCITY *power * calib.getRB());

                    this.backLeft.setVelocity(MAX_VELOCITY *lowSpeed * calib.getLB());
                    this.frontRight.setVelocity(MAX_VELOCITY *lowSpeed * calib.getRF());
                } else {
                    this.backLeft.setVelocity(MAX_VELOCITY *power * calib.getLB());
                    this.frontRight.setVelocity(MAX_VELOCITY *power * calib.getRF());

                    this.frontLeft.setVelocity(MAX_VELOCITY *lowSpeed * calib.getLF());
                    this.backRight.setVelocity(MAX_VELOCITY *lowSpeed * calib.getRB());
                }
            }
        }
        this.stop();
    }

    public void diagTo(BotMoveProfile profile) {
        if (backLeft != null && backRight != null && frontLeft != null && frontRight != null) {

            MotorReductionBot calib = profile.getMotorReduction();

            double leftOdoStart = getLeftOdometer();
            double rightOdoStart = getRightOdometer();
            double horOdoStart = getHorizontalOdometer();

            double distance = profile.getLongTarget();


            boolean leftAxis = profile.getAngleChange() > 0;
            double power = profile.getTopSpeed();
            double lowSpeed = profile.getLowSpeed();

            if (profile.getDirection() == RobotDirection.Forward) {
                power = -power;
                lowSpeed = -lowSpeed;
            }

            boolean stop = false;


            while (!stop && owner.opModeIsActive()) {
                double leftOdo = getLeftOdometer();
                double rightOdo = getRightOdometer();
                double horOdo = getHorizontalOdometer();
                double leftDistActual = Math.abs(leftOdo - leftOdoStart);
                double rightDistActual = Math.abs(rightOdo - rightOdoStart);
                double horDistActual = Math.abs(horOdo - horOdoStart);

                double catet = leftDistActual;
                if (rightDistActual > catet) {
                    catet = rightDistActual;
                }

                double hyp = Math.sqrt(catet * catet + horDistActual * horDistActual);

                if (hyp >= distance) {
                    break;
                }


                this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


                if (!leftAxis) {
                    this.frontLeft.setVelocity(MAX_VELOCITY *power * calib.getLF());
                    this.backRight.setVelocity(MAX_VELOCITY *power * calib.getRB());

                    this.backLeft.setVelocity(MAX_VELOCITY *lowSpeed * calib.getLB());
                    this.frontRight.setVelocity(MAX_VELOCITY *lowSpeed * calib.getRF());
                } else {
                    this.backLeft.setVelocity(MAX_VELOCITY *power * calib.getLB());
                    this.frontRight.setVelocity(MAX_VELOCITY *power * calib.getRF());

                    this.frontLeft.setVelocity(MAX_VELOCITY *lowSpeed * calib.getLF());
                    this.backRight.setVelocity(MAX_VELOCITY *lowSpeed * calib.getRB());
                }
            }
        }
        this.stop();
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


    //leds

    public Led getLights() {
        if (led == null) {
            led = new Led();
            led.init(hwMap, telemetry);
        }
        return led;
    }

    public void addNamedCoordinate(AutoDot dot){
        this.namedCoordinates.add(dot);
    }

}
