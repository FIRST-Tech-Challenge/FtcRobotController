package org.firstinspires.ftc.masters.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.masters.FreightFrenzyComputerVisionFindDuck;
import org.firstinspires.ftc.masters.FreightFrenzyComputerVisionShippingElementReversion;
import org.firstinspires.ftc.masters.FreightFrenzyConstants;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.masters.util.AxesSigns;
import org.firstinspires.ftc.masters.util.BNO055IMUUtil;
import org.firstinspires.ftc.masters.util.LynxModuleUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;

import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.masters.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.masters.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.masters.drive.DriveConstants.TICKS_PER_REV;
import static org.firstinspires.ftc.masters.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.masters.drive.DriveConstants.kA;
import static org.firstinspires.ftc.masters.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.masters.drive.DriveConstants.kV;
import static org.firstinspires.ftc.masters.drive.DriveConstants.encoderTicksToInches;
import static java.lang.Math.abs;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(7, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(7, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.14;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private final TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private final TrajectoryFollower follower;

    private final DcMotorEx frontLeft;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final DcMotorEx frontRight;
    public DcMotor carousel, intakeMotor, linearSlideMotor;
   // public DistanceSensor distanceSensorLeft, distanceSensorRight;
   public DistanceSensor distanceSensorIntake, distanceSensorTop;
    public Servo linearSlideServo;
    public DigitalChannel redLED, greenLED, redLED2, greenLED2;

    private final List<DcMotorEx> motors;

    private final BNO055IMU imu;
    private final VoltageSensor batteryVoltageSensor;
    protected LinearOpMode opmode;
    FreightFrenzyComputerVisionShippingElementReversion CV;
    FreightFrenzyComputerVisionFindDuck duckCV;

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        this(hardwareMap, null, null);
    }

    public SampleMecanumDrive(HardwareMap hardwareMap, LinearOpMode opmode, Telemetry telemetry) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        this.opmode = opmode;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:

        BNO055IMUUtil.remapAxes(imu, AxesOrder.XZY, AxesSigns.NPN);


        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        carousel = hardwareMap.get(DcMotor.class, "carouselMotor");
        intakeMotor = hardwareMap.dcMotor.get("intake");
        linearSlideMotor = hardwareMap.dcMotor.get("linearSlide");
        linearSlideServo = hardwareMap.servo.get("dumpServo");
        linearSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        distanceSensorLeft = (DistanceSensor) hardwareMap.get("distanceSensorLeft");
//        distanceSensorRight = (DistanceSensor) hardwareMap.get("distanceSensorRight");
        distanceSensorIntake = (DistanceSensor) hardwareMap.get("intakeSensor");
        distanceSensorTop = (DistanceSensor) hardwareMap.get("topDistanceSensor");

        redLED = (DigitalChannel) hardwareMap.get("red");
        greenLED = (DigitalChannel) hardwareMap.get("green");
        redLED2 = (DigitalChannel) hardwareMap.get("red2");
        greenLED2 = (DigitalChannel) hardwareMap.get("green2");

        motors = Arrays.asList(frontLeft, backLeft, backRight, frontRight);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }



    public void jevilTurnCarousel(double speed, double seconds) {
        carousel.setPower(speed);
        pauseButInSecondsForThePlebeians(seconds);
        carousel.setPower(0);
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public boolean getDuck(){

        int frontLeftInit= frontLeft.getCurrentPosition();
        int backLeftInit = backLeft.getCurrentPosition();
        int frontRightInit = frontRight.getCurrentPosition();
        int backRightInit = backRight.getCurrentPosition();

        while (distanceSensorIntake.getDistance(DistanceUnit.CM)<7 && DriveConstants.encoderTicksToInches(Math.abs(frontLeftInit-frontLeft.getCurrentPosition()))<3
                && DriveConstants.encoderTicksToInches(Math.abs(frontRightInit-frontRight.getCurrentPosition()))<3
                && DriveConstants.encoderTicksToInches(Math.abs(backLeftInit-backLeft.getCurrentPosition()))<3
                && DriveConstants.encoderTicksToInches(Math.abs(backRightInit-backRight.getCurrentPosition()))<3
        ){
            intakeMotor.setPower(0.8);
            frontLeft.setPower(-0.2);
            frontRight.setPower(-0.2);
            backRight.setPower(-0.2);
            backLeft.setPower(-0.2);
        }

        return DriveConstants.encoderTicksToInches(Math.abs(frontLeftInit - frontLeft.getCurrentPosition())) < 3 || DriveConstants.encoderTicksToInches(Math.abs(frontRightInit - frontRight.getCurrentPosition())) < 3
                || DriveConstants.encoderTicksToInches(Math.abs(backLeftInit - backLeft.getCurrentPosition())) < 3 || DriveConstants.encoderTicksToInches(Math.abs(backRightInit - backRight.getCurrentPosition())) < 3;
    }

    public void aquireDuckRed(double speed, double secondaryStopConditionSeconds) {
        long startTime = new Date().getTime();
        long time = 0;
        double distanceSensorValue = distanceSensorIntake.getDistance(DistanceUnit.CM);

        boolean foundFreight = false;

        intakeMotor.setPower(.8);
        setMotorPowers(-speed, -speed, -speed, -speed);


        while (distanceSensorValue > 3  && time < secondaryStopConditionSeconds * 1000) {
            //pause(50);
            time = new Date().getTime() - startTime;
            distanceSensorValue = distanceSensorIntake.getDistance(DistanceUnit.CM);
            if (distanceSensorValue > 7) {
                foundFreight = true;
            }
        }
        stopMotors();
        intakeMotor.setPower(0);
    }

    public void lightSet () {
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED2.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED2.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public boolean getCube () {
        ElapsedTime elapsedTime = new ElapsedTime();
        lightSet();
        frontLeft.setPower(-.3);
        frontRight.setPower(-.3);
        backLeft.setPower(-.3);
        backRight.setPower(-.3);
        intakeMotor.setPower(.8);
        double intakeDistance = distanceSensorIntake.getDistance(DistanceUnit.CM);
        double bucketdistance = distanceSensorTop.getDistance(DistanceUnit.CM);
        redLED.setState(true);

        while ((intakeDistance>10 && bucketdistance>14) && this.opmode.opModeIsActive() && elapsedTime.milliseconds()<1600) {
            intakeDistance = distanceSensorIntake.getDistance(DistanceUnit.CM);
            bucketdistance = distanceSensorTop.getDistance(DistanceUnit.CM);
            telemetry.addData("top distance", bucketdistance);
            telemetry.addData("intake", intakeDistance);
            telemetry.update();
        }
        stopMotors();
        telemetry.addData("top distance", distanceSensorTop.getDistance(DistanceUnit.CM));
        telemetry.update();

        if (intakeDistance<10 || distanceSensorTop.getDistance(DistanceUnit.CM)<13.5){
            pauseButInSecondsForThePlebeians(.3);
            intakeMotor.setPower(-.8);
            pauseButInSecondsForThePlebeians(.5);
            intakeMotor.setPower(0);
            redLED.setState(false);
            redLED2.setState(false);
            greenLED.setState(true);
            greenLED2.setState(true);
            linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
            return true;
        } else
            return false;
    }


//    public void distanceSensorStrafeLeft(double speed) {
//
//        double leftDistance = distanceSensorLeft.getDistance(DistanceUnit.CM);
//        double rightDistance = distanceSensorRight.getDistance(DistanceUnit.CM);
//
//        frontLeft.setPower(-speed);
//        frontRight.setPower(speed);
//        backLeft.setPower(speed);
//        backRight.setPower(-speed);
//
//        while (rightDistance - leftDistance > 1) {
//            leftDistance = distanceSensorLeft.getDistance(DistanceUnit.CM);
//            rightDistance = distanceSensorRight.getDistance(DistanceUnit.CM);
//        }
//        stopMotors();
//    }
//
//    public void distanceSensorStrafeRight(double speed) {
//        double leftDistance = distanceSensorLeft.getDistance(DistanceUnit.CM);
//        double rightDistance = distanceSensorRight.getDistance(DistanceUnit.CM);
//
//        frontLeft.setPower(speed);
//        frontRight.setPower(-speed);
//        backLeft.setPower(-speed);
//        backRight.setPower(speed);
//
//        while (leftDistance - rightDistance > 1) {
//            leftDistance = distanceSensorLeft.getDistance(DistanceUnit.CM);
//            rightDistance = distanceSensorRight.getDistance(DistanceUnit.CM);
//        }
//        stopMotors();
//    }
//
//    public void distanceSensorForward(double speed) {
//        double leftDistance = distanceSensorLeft.getDistance(DistanceUnit.CM);
//        double rightDistance = distanceSensorRight.getDistance(DistanceUnit.CM);
//
//        frontLeft.setPower(-speed);
//        frontRight.setPower(-speed);
//        backLeft.setPower(-speed);
//        backRight.setPower(-speed);
//
//        while (leftDistance > 15) {
//            leftDistance = distanceSensorLeft.getDistance(DistanceUnit.CM);
//        }
//        stopMotors();
//    }
//
//    public void distanceSensorStuff() {
//
//        double leftDistance = distanceSensorLeft.getDistance(DistanceUnit.CM);
//        double rightDistance = distanceSensorRight.getDistance(DistanceUnit.CM);
//
//
//        if (leftDistance - rightDistance > 1) {
//            distanceSensorStrafeRight(.3);
//        } else if (rightDistance - leftDistance > 1) {
//            distanceSensorStrafeLeft(.3);
//        }
//
//        if (leftDistance > 15) {
//            distanceSensorForward(.3);
//        }
//    }

    public void dumpFreightBottom() {
        linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_LOW);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setPower(.8);
        while (linearSlideMotor.isBusy() && this.opmode.opModeIsActive()) {

        }
        pause(150);
        linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_DROP);
        pause(1200);
        linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
        linearSlideMotor.setTargetPosition(0);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setPower(-.4);
        while (linearSlideMotor.isBusy() && this.opmode.opModeIsActive()) {

        }
        linearSlideMotor.setPower(0);
    }

    public void dumpFreightMiddle() {
        linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_MIDDLE);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setPower(.9);
        while (linearSlideMotor.isBusy() && this.opmode.opModeIsActive()) {
        }
        pause(150);
        linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_DROP);//1.5
        pause(1200);
        //forward(0.3, -0.4);
        linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
        linearSlideMotor.setTargetPosition(0);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setPower(-.4);
        while (linearSlideMotor.isBusy() && this.opmode.opModeIsActive()) {

        }
        linearSlideMotor.setPower(0);
    }

    public void dumpFreightTop() {
        linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_TOP);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setPower(.9);
        while (linearSlideMotor.isBusy() && this.opmode.opModeIsActive()) {

        }
        pause(200);

        linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_DROP);
        pause(800);
        //forward(0.3, -0.2);
        linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
        linearSlideMotor.setTargetPosition(0);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setPower(-.4);
        while (linearSlideMotor.isBusy() && this.opmode.opModeIsActive()) {

        }
        linearSlideMotor.setPower(0);
    }

    protected void motorSetMode(DcMotor.RunMode runMode) {
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

    public void forward(double speed, double rotations) {
        int leftCurrent = frontLeft.getCurrentPosition();
        int rightCurrent = frontRight.getCurrentPosition();
        int backLeftCurrent = backLeft.getCurrentPosition();
        int backRightCurrent = backRight.getCurrentPosition();

        double toPositionLeft = leftCurrent + rotations * TICKS_PER_REV;
        double toPositionRight = rightCurrent + rotations * TICKS_PER_REV;
        double toPositionbackLeft = backLeftCurrent + rotations * TICKS_PER_REV;
        double toPositionbackRight = backRightCurrent + rotations * TICKS_PER_REV;

        frontLeft.setTargetPosition((int) toPositionLeft);
        frontRight.setTargetPosition((int) toPositionRight);
        backLeft.setTargetPosition((int) toPositionbackLeft);
        backRight.setTargetPosition((int) toPositionbackRight);

        motorSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(abs(speed));
        frontRight.setPower(abs(speed));
        backLeft.setPower(abs(speed));
        backRight.setPower(abs(speed));

        while (this.opmode.opModeIsActive() &&
                (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

        }
        stopMotors();

        motorSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void strafeLeft(double speed, double rotations) {

        int leftCurrent = frontLeft.getCurrentPosition();
        int rightCurrent = frontRight.getCurrentPosition();
        int backLeftCurrent = backLeft.getCurrentPosition();
        int backRightCurrent = backRight.getCurrentPosition();

        double toPositionLeft = leftCurrent - rotations * TICKS_PER_REV;
        double toPositionRight = rightCurrent + rotations * TICKS_PER_REV;
        double toPositionbackLeft = backLeftCurrent + rotations * TICKS_PER_REV;
        double toPositionbackRight = backRightCurrent - rotations * TICKS_PER_REV;

        frontLeft.setTargetPosition((int) toPositionLeft);
        frontRight.setTargetPosition((int) toPositionRight);
        backLeft.setTargetPosition((int) toPositionbackLeft);
        backRight.setTargetPosition((int) toPositionbackRight);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(abs(-speed));
        frontRight.setPower(abs(speed));
        backLeft.setPower(abs(speed));
        backRight.setPower(abs(-speed));
        while (this.opmode.opModeIsActive() &&
                (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

        }
        stopMotors();

        motorSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void strafeRight(double speed, double rotations) {

        int leftCurrent = frontLeft.getCurrentPosition();
        int rightCurrent = frontRight.getCurrentPosition();
        int backLeftCurrent = backLeft.getCurrentPosition();
        int backRightCurrent = backRight.getCurrentPosition();

        double toPositionLeft = leftCurrent + rotations * TICKS_PER_REV;
        double toPositionRight = rightCurrent - rotations * TICKS_PER_REV;
        double toPositionbackLeft = backLeftCurrent - rotations * TICKS_PER_REV;
        double toPositionbackRight = backRightCurrent + rotations * TICKS_PER_REV;

        frontLeft.setTargetPosition((int) toPositionLeft);
        frontRight.setTargetPosition((int) toPositionRight);
        backLeft.setTargetPosition((int) toPositionbackLeft);
        backRight.setTargetPosition((int) toPositionbackRight);

        motorSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(abs(speed));
        frontRight.setPower(abs(-speed));
        backLeft.setPower(abs(-speed));
        backRight.setPower(abs(speed));
        while (this.opmode.opModeIsActive() &&
                (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

        }
        stopMotors();

        motorSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void findDuckRed(){

        //strafe right
        double speed = 0.35;

        while (this.opmode.opModeIsActive() && analyzeDuck()!= FreightFrenzyComputerVisionFindDuck.DuckDeterminationPipeline.DuckPosition.LEFT2 &&
                analyzeDuck()!= FreightFrenzyComputerVisionFindDuck.DuckDeterminationPipeline.DuckPosition.LEFT1
                && analyzeDuck()!= FreightFrenzyComputerVisionFindDuck.DuckDeterminationPipeline.DuckPosition.CENTER){
            frontLeft.setPower(speed);
            frontRight.setPower(-speed);
            backLeft.setPower(-speed);
            backRight.setPower(speed);
        }

        intakeMotor.setPower(0.8);
        forward(0.4, -1.5);

        stopMotors();
    }

    public void findDuckBlue(){

        //strafe right
        double speed = 0.35;

        while (this.opmode.opModeIsActive() && analyzeDuck()!= FreightFrenzyComputerVisionFindDuck.DuckDeterminationPipeline.DuckPosition.RIGHT2 &&
                analyzeDuck()!= FreightFrenzyComputerVisionFindDuck.DuckDeterminationPipeline.DuckPosition.RIGHT1
                && analyzeDuck()!= FreightFrenzyComputerVisionFindDuck.DuckDeterminationPipeline.DuckPosition.CENTER){
            frontLeft.setPower(-speed);
            frontRight.setPower(speed);
            backLeft.setPower(speed);
            backRight.setPower(-speed);
        }

        intakeMotor.setPower(0.8);
        forward(0.4, -1.5);

        stopMotors();
    }

    public void openCVInnitShenanigans(String color) {
        CV = new FreightFrenzyComputerVisionShippingElementReversion(hardwareMap, telemetry);

    }

    public void openDuckCVInit(){
        duckCV = new FreightFrenzyComputerVisionFindDuck(hardwareMap, telemetry);
    }

    public FreightFrenzyComputerVisionShippingElementReversion.SkystoneDeterminationPipeline.FreightPosition analyze() {
        return CV.pipeline.position;
    }

    public FreightFrenzyComputerVisionFindDuck.DuckDeterminationPipeline.DuckPosition analyzeDuck() {
        return duckCV.pipeline.position;
    }


    public FreightFrenzyComputerVisionShippingElementReversion.SkystoneDeterminationPipeline.HubPosition analyze_hub_blue() {
        return CV.pipeline.hub_position;
    }

    public FreightFrenzyComputerVisionShippingElementReversion.SkystoneDeterminationPipeline.HubPosition analyze_hub_red() {
        return CV.pipeline.hub_position;
    }


    public void pause(int millis) {
        long startTime = new Date().getTime();
        long time = 0;

        while (time < millis && opmode.opModeIsActive()) {
            time = new Date().getTime() - startTime;
        }
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        frontLeft.setPower(v);
        backLeft.setPower(v1);
        backRight.setPower(v2);
        frontRight.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface

        // To work around an SDK bug, use -zRotationRate in place of xRotationRate 
        // and -xRotationRate in place of zRotationRate (yRotationRate behaves as 
        // expected). This bug does NOT affect orientation. 
        //
        // See https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251 for details.
        return (double) -imu.getAngularVelocity().xRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public void pauseButInSecondsForThePlebeians(double seconds) {
        long startTime = new Date().getTime();
        long time = 0;

        while (time < seconds * 1000 && this.opmode.opModeIsActive()) {
            time = new Date().getTime() - startTime;
        }
    }

    public void retract(){
        linearSlideMotor.setTargetPosition(0);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setPower(.4);
    }
}
