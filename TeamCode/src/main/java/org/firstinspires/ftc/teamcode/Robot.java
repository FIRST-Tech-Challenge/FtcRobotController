package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import android.util.Log;
import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Robot {
    public final Outtake outtake;
    public final Intake intake;
    public final DroneLauncher droneLauncher;
    public final Drivetrain drivetrain;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    LinearOpMode opMode;
    DcMotor rightEncoder;
    DcMotor leftEncoder;
    DcMotor backEncoder;
    public DcMotor fLeft;
    DcMotor fRight;
    public DcMotor bLeft;
    DcMotor bRight;
    DcMotor intakeMotor;
    DcMotor lsBack;
    DcMotor lsFront;
    Servo tray;
    Servo clamp;
    Servo hook;
    DcMotor planeLauncher;
    Servo planeLauncherServo;
    double planeServoDisengage = 0.73;
    Servo spikeServo;
    Servo trayAngle;
    double trayAngleDefault = 0.5;
    IMU imu;
    double prevError = 0;
    double prevTime = 0;
    Servo stackAttachment;
    double botHeading = 0;
    ElapsedTime elapsedTime = new ElapsedTime();
    public MarkerDetector.MARKER_POSITION markerPos;
    int wantedAprTagId;
    int secondWantedTagId;
    public MarkerProcessor markerProcessor;
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;
    boolean isRedAlliance;
    boolean testingOnBert = false;
    boolean allowTrayAngle = false;
    boolean allowTrayAngleOverride = false;
    double hardStopTrayAngleBig;
    double hardStopTrayAngleSmall;

    public PIDController straightController;
    public PIDController fLeftMecanumController;
    public PIDController bRightMecanumController;
    PIDController setHeadingController;
    double robotX = 0;
    double robotY = 0;
    private boolean previousHookButtonValue;
    private boolean hookShouldClose;
    private boolean stackShouldBeDown;
    private boolean previousStackButtonValue;
    private double trayAngleSlope;
    private double teleOpTuneValueTrayAngle;
    double slideStartingPosition;
    boolean isLong;


    public enum MARKER_LOCATION {
        INNER, CENTER, OUTER
    }
    public enum PARKING_POSITION {
        FREEWAY, TRUSS, BOARD
    }
    MARKER_LOCATION markerLocation;

    boolean lowOuttake;
    int autoDelayInSeconds;
    PARKING_POSITION parkingPosition;

    double DEAD_WHEEL_RADIUS = 24;
    double TICKS_PER_REV = 2000;
    double CM_PER_TICK = 2.0 * Math.PI * DEAD_WHEEL_RADIUS / TICKS_PER_REV;



    //CONSTRUCTOR
    public Robot(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry, boolean isLong, boolean red, boolean isAutonomous) {
        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
        this.telemetry = telemetry;
        AprilTagProcessor aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        setUpDrivetrainMotors();
        setUpImu(isAutonomous);
        isRedAlliance = red;
        if (isLong) {
            robotX = 35;
            robotY = 17;
            isLong = true;

        } else if (!isLong) {
            robotX = 83;
            robotY = 17;
            isLong = false;
        }

        if (!testingOnBert) {
            intakeMotor = hardwareMap.dcMotor.get("intake");
            lsFront = hardwareMap.dcMotor.get("lsFront");
            lsFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lsBack = hardwareMap.dcMotor.get("lsBack");
            lsBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // get servos from hardware map
            tray = hardwareMap.servo.get("arm");
            clamp = hardwareMap.servo.get("holderClamp");
            hook = hardwareMap.servo.get("linearLocker");
            spikeServo = hardwareMap.servo.get("spikeServo");
            stackAttachment = hardwareMap.servo.get("stackAttachment");
            trayAngle = hardwareMap.servo.get("trayAngle");

            // initialize controllers
            //straightController = new PIDController("straight", 0.007, 0.0000005, 0.4, false);
            straightController = new PIDController("straight", 0.005, 0.0000015, 0.8, false);
            fLeftMecanumController = new PIDController("fl mecanum", 0.005, 0.0000005, 0.4, true); //0.01 - 0.0001
            bRightMecanumController = new PIDController("br mecanum", 0.005, 0.0000005, 0.4, true);
            setHeadingController = new PIDController("set heading", 0.06, 0, 2_500_000, false);

            outtake = new Outtake(this);
            intake = new Intake(this);
            droneLauncher = new DroneLauncher(this);
            drivetrain = new Drivetrain(this);
        } else {
            outtake = null;
            intake = null;
            droneLauncher = null;
            drivetrain = null;
        }
    }


    public void moveLinearSlideByTicksBlocking(double targetDistanceInTicks) {

        //target distance negative when going up
        double remainingDistanceLow = targetDistanceInTicks - lsFront.getCurrentPosition();

        while (opMode.opModeIsActive() && Math.abs(targetDistanceInTicks - lsFront.getCurrentPosition()) > 100) {
            remainingDistanceLow = targetDistanceInTicks - lsFront.getCurrentPosition();

            double power = remainingDistanceLow * 0.002;

            double minPower = 0.4;
            if (power > 0 && power < minPower) {
                power = minPower;
            } else if (power < 0 && power > -1 * minPower) {
                power = -1 * minPower;
            }

            lsFront.setPower(power);
            lsBack.setPower(power);

            telemetry.update();

            // opMode.sleep(100); TIME OPTIMIZATION

//            Log.d("vision ls", "moveLinearSlideByTicksBlocking: lsFront position " + lsFront.getCurrentPosition());
//            Log.d("vision ls", "moving linear slide: remaining distance " + remainingDistanceLow);
//            Log.d("vision ls", "moving linear slide: power " + power);
        }

        lsFront.setPower(0);
        lsBack.setPower(0);

        telemetry.update();
    }

    public void setServoPos(Servo servo, double targetServoPos) {

        servo.setPosition(targetServoPos);
    }

    public void trayToOuttakeSplitEditionOne() {
        setServoPos(tray, 0.15);
    }

    public void trayToOuttakeSplitEditionTwo() {
        setServoPos(tray, 0.0);
        opMode.sleep(400);
    }

    public void autoOuttake(boolean lowOuttake) {

        // move linear slide up
        if (lowOuttake) {
            outtake.trayToOuttakePos(false); // pivot tray to outtake position
            moveLinearSlideByTicksBlocking(slideStartingPosition + 1550); //1700
            opMode.sleep(100);
            outtake.openClamp(true, true, false); // drop pixel
            opMode.sleep(200);
            moveLinearSlideByTicksBlocking(slideStartingPosition + 1950);
        } else {
            trayToOuttakeSplitEditionOne();
            moveLinearSlideByTicksBlocking(slideStartingPosition + 1750);//1900
            trayToOuttakeSplitEditionTwo();
            outtake.openClamp(true, true, false); // drop pixel
            opMode.sleep(200);
            moveLinearSlideByTicksBlocking(slideStartingPosition + 2350);
        }

        straightBlocking(4, true, 0.7); //move back 2

        setServoPos(trayAngle, 0.5);
        outtake.trayToIntakePos(true); //intake pos
    }

    public void setMarkerPos(MarkerDetector.MARKER_POSITION position) {
        markerPos = position;
    }

    public void setMarkerLocation (boolean isRedAlliance, boolean longPath, MarkerDetector.MARKER_POSITION markerPosition) {
        if (longPath) {
            if ((markerPos == MarkerDetector.MARKER_POSITION.RIGHT && isRedAlliance)
                    || (markerPos == MarkerDetector.MARKER_POSITION.LEFT && !isRedAlliance)) {

                markerLocation = MARKER_LOCATION.INNER;

            } else if ((markerPos == MarkerDetector.MARKER_POSITION.LEFT && isRedAlliance)
                    || (markerPos == MarkerDetector.MARKER_POSITION.RIGHT && !isRedAlliance)) {

                markerLocation = MARKER_LOCATION.OUTER;

            } else {

                markerLocation = MARKER_LOCATION.CENTER;

            }
        } else {
            if ((markerPos == MarkerDetector.MARKER_POSITION.RIGHT && isRedAlliance)
                    || (markerPos == MarkerDetector.MARKER_POSITION.LEFT && !isRedAlliance)) {

                markerLocation = MARKER_LOCATION.OUTER;

            } else if ((markerPos == MarkerDetector.MARKER_POSITION.LEFT && isRedAlliance)
                    || (markerPos == MarkerDetector.MARKER_POSITION.RIGHT && !isRedAlliance)) {

                markerLocation = MARKER_LOCATION.INNER;

            } else {

                markerLocation = MARKER_LOCATION.CENTER;

            }
        }
    }

    public void setWantedAprTagId(MarkerDetector.MARKER_POSITION position, MarkerDetector.ALLIANCE_COLOR allianceColor) {

        if (allianceColor == MarkerDetector.ALLIANCE_COLOR.RED) {
            switch (position) {
                case CENTER:
                    Log.d("vision", "setWantedAprTagId: tag = " + 5);
                    wantedAprTagId = 5;
                    break;
                case RIGHT:
                    Log.d("vision", "setWantedAprTagId: tag = " + 6);
                    wantedAprTagId = 6;
                    break;
                case LEFT:
                    Log.d("vision", "setWantedAprTagId: tag = " + 4);
                    wantedAprTagId = 4;
                    break;
                default:
                    Log.d("vision", "setWantedAprTagId: enter default. tag = " + 5);
                    wantedAprTagId = 5;
            }
        } else {
            switch (position) {
                case CENTER:
                    Log.d("vision", "setWantedAprTagId: tag = " + 2);
                    wantedAprTagId = 2;
                    break;
                case RIGHT:
                    Log.d("vision", "setWantedAprTagId: tag = " + 3);
                    wantedAprTagId = 3;
                    break;
                case LEFT:
                    Log.d("vision", "setWantedAprTagId: tag = " + 1);
                    wantedAprTagId = 1;
                    break;
                default:
                    Log.d("vision", "setWantedAprTagId: enter default. tag = " + 2);
                    wantedAprTagId = 2;
            }
        }
    }

    public void initVisionProcessing() {
        // Initializing marker and apriltag processors and setting them with visionportal
        markerProcessor = new MarkerProcessor(telemetry, isRedAlliance ? MarkerDetector.ALLIANCE_COLOR.RED : MarkerDetector.ALLIANCE_COLOR.BLUE);
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setLiveViewContainerId(VisionPortal.DEFAULT_VIEW_CONTAINER_ID)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(false)
                .setAutoStopLiveView(true)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .addProcessor(markerProcessor)
                .build();
    }

    public void setUpImu(boolean isAutonomous) {

        this.imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        if (isAutonomous) {
            imu.resetYaw();
        }

    }

    public void setUpDrivetrainMotors() {
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightEncoder = bRight;
        leftEncoder = bLeft;
        backEncoder = fRight;

        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void resetDrivetrainEncoders () {
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */
    }

    public void setUpIntakeOuttake() {
        intakeMotor = hardwareMap.dcMotor.get("intake");
        lsBack = hardwareMap.dcMotor.get("lsBack");
        lsFront = hardwareMap.dcMotor.get("lsFront");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lsFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lsBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lsFront.setDirection(DcMotorSimple.Direction.REVERSE);
        lsBack.setDirection(DcMotorSimple.Direction.REVERSE);

        lsFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lsFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getCurrentHeading() {
        double currentYaw;
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        // Loop here if IMU is bad (usually because battery power is low)
        while ((robotOrientation.getAcquisitionTime() == 0) && opMode.opModeIsActive()) {
            Log.d("imu crash", "getCurrentHeading: acquisition time is 0");
            opMode.sleep(100);
            robotOrientation = imu.getRobotYawPitchRollAngles();
        }

        currentYaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        return currentYaw;
    }
    public double getCurrentHeadingTeleOp() {
        double currentYaw;
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        if (robotOrientation.getAcquisitionTime() == 0) {
            Log.d("imu crash", "getCurrentHeading: acquisition time is 0");
            return 500;
        }

        currentYaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        return currentYaw;
    }

    public void setHeading(double targetAbsDegrees, double maxPower) {
        YawPitchRollAngles robotOrientation;
        double KP = 0.06;
        double KD = 2_500_000;
        double ERROR_TOLERANCE = 0.5; //degrees
        double currentHeading = getCurrentHeading();
        double error = angleWrap(targetAbsDegrees - currentHeading);
        double errorDer;
        double power;
        double currentTime;
        double minPower = 0.15;

        //while start
        while (Math.abs(error) > ERROR_TOLERANCE && opMode.opModeIsActive()) {
            currentHeading = getCurrentHeading();

            currentTime = SystemClock.elapsedRealtimeNanos();
            error = targetAbsDegrees - currentHeading; //error is degrees to goal
            errorDer = (error - prevError) / (currentTime - prevTime);
            power = (KP * error) + (KD * errorDer);

                /*
                Log.d("pid", "setHeading: current heading is " + currentHeading);
                Log.d("pid", "setHeading: Target heading is " + targetAbsDegrees);
                Log.d("pid", "setHeading: time is " + currentTime);
                Log.d("pid", "setHeading: heading error is " + error);
                Log.d("pid", "setHeading: errorDer is " + errorDer);
                Log.d("pid", "setHeading: calculated power is " + power);
                */

            if (power > 0 && power < minPower) {
                power = minPower;
                // Log.d("pid", "setHeading: adjusted power is " + power);
            } else if (power < 0 && power > -1 * minPower) {
                power = minPower * -1;
                // Log.d("pid", "setHeading: adjusted power is " + power);
            }

            //cap power
            power = Range.clip(power, -1 * maxPower, maxPower);
            // Log.d("pid", "straightBlockingFixHeading: power after clipping is " + power);

            drivetrain.setMotorPower(-1 * power, power, -1 * power, power);
            prevError = error;
            prevTime = currentTime;
        }
        drivetrain.setMotorPower(0, 0, 0, 0);
        opMode.sleep(100);
        currentHeading = getCurrentHeading();
        botHeading = targetAbsDegrees;
        Log.d("pid", "setHeading: final heading is " + currentHeading);
    }

    public void mecanumBlocking(double inches, boolean right, double maxPower) {

        double ERROR_TOLERANCE = 10;
        double power;
        double targetTick;
        final double KP_MECANUM = 0.002;
        final double minPower = 0.4;
        final double IN_TO_TICK = 56.3;

        assert maxPower >= minPower;

        double currentTick = fLeft.getCurrentPosition();

        if (right) {
            targetTick = currentTick + inches * IN_TO_TICK;
        } else {
            targetTick = currentTick - inches * IN_TO_TICK;
        }

        double error = targetTick - currentTick;

        telemetry.addLine("current ticks: " + currentTick);
        telemetry.addLine("target ticks: " + targetTick);
        telemetry.addLine("delta: " + error);
        telemetry.update();

        while (Math.abs(error) >= ERROR_TOLERANCE && opMode.opModeIsActive()) {
            power = KP_MECANUM * error;

            if (power > 0 && power < minPower) {
                power = minPower;
            } else if (power < 0 && power > -1 * minPower) {
                power = -1 * minPower;
            }

            //cap power
            power = Range.clip(power, -1 * maxPower, maxPower);

            drivetrain.setMotorPower(power, -1 * power, -1 * power, power);

            error = targetTick - fLeft.getCurrentPosition();
        }
        drivetrain.setMotorPower(0, 0, 0, 0);
        opMode.sleep(100);
    }

    public void straightBlocking(double inches, boolean forward, double maxPower) {
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double ERROR_TOLERANCE = 10;
        double power;
        double endTick;
        final double KP = 0.01;
        final double KD = 500_000;
        final double minPower = 0.2;
        double currentTick = fLeft.getCurrentPosition();
        double errorDer;
        double currentTime;

        //inch to tick
        final double wheelDiaMm = 96;
        final double PI = Math.PI;
        final double wheelCircIn = wheelDiaMm * PI / 25.4; //~11.87
        final double IN_TO_TICK = 537 / wheelCircIn;

        if (forward) {
            endTick = currentTick + inches * IN_TO_TICK;
        } else {
            endTick = currentTick - inches * IN_TO_TICK;
        }

        double error = endTick - currentTick;

        while (Math.abs(error) >= ERROR_TOLERANCE && opMode.opModeIsActive()) {

            currentTime = SystemClock.elapsedRealtimeNanos();
            error = endTick - currentTick;
            errorDer = (error - prevError) / (currentTime - prevTime);
            power = (KP * error) + (KD * errorDer);

            if (power > 0 && power < minPower) {
                power += minPower;
            } else if (power < 0 && power > -1 * minPower) {
                power -= minPower;
            }

            //cap power
            power = Range.clip(power, -1 * maxPower, maxPower);

            drivetrain.setMotorPower(power, power, power, power);

            currentTick = fLeft.getCurrentPosition();
            prevTime = currentTime;
            prevError = error;
        }
        drivetrain.setMotorPower(0, 0, 0, 0);
        opMode.sleep(100);
    }

    public void straightBlockingWithTimer(double inches, boolean forward, double maxPower, double seconds) {
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double ERROR_TOLERANCE = 10;
        double power;
        double endTick;
        final double KP = 0.01;
        final double KD = 500_000;
        final double minPower = 0.2;
        double currentTick = fLeft.getCurrentPosition();
        double errorDer;
        double currentTime;

        //inch to tick
        final double wheelDiaMm = 96;
        final double PI = Math.PI;
        final double wheelCircIn = wheelDiaMm * PI / 25.4; //~11.87
        final double IN_TO_TICK = 537 / wheelCircIn;

        if (forward) {
            endTick = currentTick + inches * IN_TO_TICK;
        } else {
            endTick = currentTick - inches * IN_TO_TICK;
        }

        double error = endTick - currentTick;
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while ((Math.abs(error) >= ERROR_TOLERANCE && elapsedTime.milliseconds() < seconds * 1000) && opMode.opModeIsActive()) {

            currentTime = SystemClock.elapsedRealtimeNanos();
            error = endTick - currentTick;
            errorDer = (error - prevError) / (currentTime - prevTime);
            power = (KP * error) + (KD * errorDer);

            if (power > 0 && power < minPower) {
                power += minPower;
            } else if (power < 0 && power > -1 * minPower) {
                power -= minPower;
            }

            //cap power
            power = Range.clip(power, -1 * maxPower, maxPower);

            drivetrain.setMotorPower(power, power, power, power);

            currentTick = fLeft.getCurrentPosition();
            prevTime = currentTime;
            prevError = error;
        }
        drivetrain.setMotorPower(0, 0, 0, 0);
        opMode.sleep(100);
    }

    public void detectMarkerPosition() {

        //boolean isTesting = false;
        int visionTimeout = 500; // timeout detection after 2 seconds
        double time;
        MarkerDetector.MARKER_POSITION position;

        elapsedTime.reset();
        time = elapsedTime.milliseconds();

        //detect marker position
        position = MarkerDetector.MARKER_POSITION.UNDETECTED;
        int i =  0;
        while (position == MarkerDetector.MARKER_POSITION.UNDETECTED && opMode.opModeIsActive()) {
            i++;
            Log.d("vision", i + " undetected marker, keep looking" + visionPortal.getCameraState());
            position = markerProcessor.getPosition();
            Log.d("color detection", String.valueOf(markerProcessor.avgCenterCb));
            //Log.d("color detection", String.valueOf(markerProcessor.avgRightCb));
            Log.d("color detection", String.valueOf(markerProcessor.avgCenterCr));
            Log.d("elapsed time", String.valueOf(elapsedTime.milliseconds()));
            if (elapsedTime.milliseconds() > time + visionTimeout && position == MarkerDetector.MARKER_POSITION.UNDETECTED) {
                position = MarkerDetector.MARKER_POSITION.CENTER;
                Log.d("vision", "detected time out. Picking CENTER");
                break;
            }
        }

        Log.d("done process", "detected at" + String.valueOf(elapsedTime.milliseconds()));

        //save marker position and apriltag position in robot class
        setMarkerPos(position);
        setWantedAprTagId(position, isRedAlliance ? MarkerDetector.ALLIANCE_COLOR.RED : MarkerDetector.ALLIANCE_COLOR.BLUE);
        setSecondWantedTagId();

        //print position
        Log.d("done process", "detected position: " + position);
        telemetry.addData("position ", markerPos);
        telemetry.update();
    }

    public void shortMoveToBoard2 () {
        int polarity = isRedAlliance ? -1 : 1;

        while (opMode.opModeIsActive()) {

            Log.d("vision", "path: Pos " + markerPos);
            Log.d("vision", "path: Tag " + wantedAprTagId);

            if (markerLocation == MARKER_LOCATION.INNER) {
                Log.d("vision", "path: Inner Spike");

                // P1: (35, 17)

                straightBlocking2(-29);

                // P2: (35, 44)

                setHeading(-90 * polarity, 0.7);

                // P3: (43.5, 35.5)
                if (isRedAlliance) {
                    straightBlocking2(-2);
                } else  {
                    straightBlocking2(-3);
                }
                setHeading(-90 * polarity, 0.7);

                if (!testingOnBert) {
                    moveFingerUp();
                    opMode.sleep(200);
                }

                setHeading(-90 * polarity, 0.7);

                visionPortal.setProcessorEnabled(aprilTagProcessor, true);
                straightBlocking2(29.5);
                setHeading(90 * polarity, 0.7);

                break;

            } else if (markerLocation == MARKER_LOCATION.OUTER) {
                Log.d("vision","path: Outer Spike");

                // P1: (35, 17)

                //straightBlocking2(-2);

                if (isRedAlliance) {
                    mecanumBlocking2(-23);
                } else {
                    mecanumBlocking2(19);
                }
                setHeading(0, 0.7);

                // P2: (14, 17)

                straightBlocking2(-29);

                // P3: (14, 48)

                setHeading(-90 * polarity, 0.7);

                straightBlocking2(-1.5);

                // P4: (22.5, 39.5)

                if (!testingOnBert) {
                    moveFingerUp();
                    opMode.sleep(200);
                }
                setHeading(-90 * polarity, 0.7);

                straightBlocking2(9.5);
                setHeading(90 * polarity, 0.7);

                visionPortal.setProcessorEnabled(aprilTagProcessor, true);

                mecanumBlocking2(10 * polarity);
                setHeading(90 * polarity, 0.7);

                break;

            } else { //center, default
                Log.d("vision", "path: Center Spike");

                // P1: (36, 17)

                straightBlocking2(-2);

                if (isRedAlliance) {
                    mecanumBlocking2(-15);
                } else {
                    mecanumBlocking2(12);
                }

                setHeading(0, 0.7);

                // P2: (23, 17)

                straightBlocking2(-34);

                // P3: (23, 52)
                setHeading(-90 * polarity, 0.7);

                // P4: (31.5, 44)

                if (!testingOnBert) {
                    moveFingerUp();
                    opMode.sleep(200);
                }

                straightBlocking(6, true, 0.7);

                setHeading(90 * polarity, 0.7);

                straightBlocking2(-10);

                setHeading(90 * polarity, 0.7);

                visionPortal.setProcessorEnabled(aprilTagProcessor, true);

                mecanumBlocking2(10 * polarity);

                break;
            }
        }
    }

    public void alignToBoardFast(int tagId) {
        boolean aligned = false;
        List<AprilTagDetection> myAprilTagDetections;
        double distanceToBoard = 12;
        double PIXEL_SIZE = 6;
        int numberOfDetectionsProcessed = 0;
        double distanceToMove;
        double distanceBetweenId = 6;
        boolean movedToDesired = false;

        if (isLong) {
            opMode.sleep(250);
        } else {
            opMode.sleep(500);
        }

        while (!aligned && opMode.opModeIsActive()) {
            Log.d("apriltag", "alignToBoardFast: passed tagid is " + tagId);

            if (numberOfDetectionsProcessed > 200) {
                Log.d("apriltag", "alignToBoardFast: didn't align, distanceToBoard is 12");
                break;
            } else {
                numberOfDetectionsProcessed++;
            }

            myAprilTagDetections = aprilTagProcessor.getDetections();
            if (myAprilTagDetections.size() == 0) {
                Log.d("apriltag", "Houston: No tags seen");
            }

            for (AprilTagDetection detection : myAprilTagDetections) {
                if (detection.metadata == null) {
                    Log.d("apriltag", "Houston: No meta data for this detection");
                }

                if (detection.metadata != null && !movedToDesired) {
                    Log.d("apriltag", "found tag" + detection.id);
                    distanceToMove = ((tagId - detection.id) * distanceBetweenId) + detection.ftcPose.x;
                    Log.d("apriltag", "calculated, distance to move: " + distanceToMove);
                    mecanumBlocking(distanceToMove, false, 0.7); //-1 to fix too much movement
                    Log.d("apriltag", "moved");
                    movedToDesired = true;
                }

                if (detection.metadata != null && movedToDesired) {
                    Log.d("apriltag", "runOpMode: aligned");
                    Log.d("apriltag", "alignToBoard: Range is " + detection.ftcPose.range);
                    distanceToBoard = Math.abs(detection.ftcPose.range) - PIXEL_SIZE;
                    aligned = true;
                    break;
                }
            }
        }
        straightBlockingWithTimer(distanceToBoard, false, 0.4, 1);
    }

    public void longMoveToBoard(boolean isJuice) {
        int polarity = (isRedAlliance) ? -1 : 1;

        while (opMode.opModeIsActive()) {

            Log.d("vision", "path: Pos " + markerPos);
            Log.d("vision", "path: Tag " + wantedAprTagId);

            if (markerLocation == MARKER_LOCATION.INNER) {
                Log.d("vision", "path: Inner Spike");

                // P1: (35, 17)

                straightBlocking2(-27);

                setHeading(90 * polarity, 0.7);

                if (isRedAlliance) {
                    straightBlocking2(-3);
                } else {
                    straightBlocking2(-2);
                }
                setHeading(90 * polarity, 0.7);

                if (!testingOnBert) {
                    moveFingerUp();
                    opMode.sleep(200);
                }

                straightBlocking2(6);

                // P3: (43.5, 35.5)

                if (isRedAlliance) {
                    mecanumBlocking2(24.5);
                } else {
                    mecanumBlocking2(-26.5);
                }
                setHeading(90 * polarity, 0.7);

                // P4: (43.5, 60)

                straightBlocking2FixHeading(-78.5);
                setHeading(90 * polarity, 0.7);

                // P5: (120, 60)

                visionPortal.setProcessorEnabled(aprilTagProcessor, true);

                if (isRedAlliance) {
                    mecanumBlocking2(-30);
                } else {
                    mecanumBlocking2(30);
                }
                setHeading(90 * polarity, 0.7);

                // P6: (120, 30)

                break;

            } else if (markerLocation == MARKER_LOCATION.OUTER) {
                Log.d("vision", "path: Outer Spike");

                if (isRedAlliance) {
                    mecanumBlocking2(20);
                } else {
                    mecanumBlocking2(-23);
                }

                setHeading(0, 0.7);

                straightBlocking2(-31);

                setHeading(90 * polarity, 0.7);

                //spike
                if (!testingOnBert) {
                    moveFingerUp();
                    opMode.sleep(200);
                }

                if(isRedAlliance){
                    straightBlocking(2, false, 0.7);
                    setHeading(90 * polarity, 0.7);
                    straightBlocking(2, true, 0.7);
                    setHeading(90 * polarity, 0.7);
                } else {
//                    straightBlocking(2, false, 0.7);
//                    setHeading(90 * polarity, 0.7);
                    straightBlocking(2, true, 0.7);
//                    setHeading(90 * polarity, 0.7);
                }

                if (isRedAlliance) {
                    mecanumBlocking2(18);
                } else {
                    mecanumBlocking2(-21);
                }
                setHeading(90 * polarity, 0.7);

                // P6: (19.5, 57.5)

                straightBlocking2FixHeading(-93);
                setHeading(90 * polarity, 0.7);

                // P7: (117.5, 57.5)

                visionPortal.setProcessorEnabled(aprilTagProcessor, true);

                if (isRedAlliance) {
                    mecanumBlocking2(-21);
                } else {
                    mecanumBlocking2(17);
                }
                setHeading(90 * polarity, 0.7);

                // P8: (117.5, 36.5)

                break;

            } else { //center, default
                Log.d("vision", "path: Center Spike");

                if (isRedAlliance) {
                    mecanumBlocking2(13);
                } else {
                    mecanumBlocking2(-13);
                }
                setHeading(0, 0.7);

                if (isRedAlliance) {
                    straightBlocking2(-36);
                } else {
                    straightBlocking2(-35);
                }

                setHeading(90 * polarity, 0.7);

                if (!testingOnBert) {
                    moveFingerUp();
                    opMode.sleep(200);
                }

                straightBlocking(3, true, 0.7);

                // P5: (27.5, 44)
                // actually ending up at around 48 here

                if (isRedAlliance) {
                    mecanumBlocking2(12);
                } else {
                    mecanumBlocking2(-15);
                }

                setHeading(90 * polarity, 0.7);

                // P6: (27.5, 60)

                straightBlocking2FixHeading(-89.5); // subtracted 4 here
                setHeading(90 * polarity, 0.7);

                // P7: (120, 60)

                visionPortal.setProcessorEnabled(aprilTagProcessor, true);

                if (isRedAlliance) {
                    mecanumBlocking2(-26);
                } else {
                    mecanumBlocking2(26);
                }
                setHeading(90 * polarity, 0.7);

                // P8: (120, 35)

                break;

            }
        }
    }

    public void openHook() {
        hook.setPosition(0.37); //started 0.49
    }

    public void closeHook() {
        hook.setPosition(0.27);
    }

    //0 is aligned to board
    public void setHeadingRelativeToBoard(double relativeHeading, double maxPower) {
        double absoluteHeading;

        if (isRedAlliance) {
            absoluteHeading = relativeHeading - 90;
        } else {
            absoluteHeading = relativeHeading + 90;
        }

        setHeading(absoluteHeading, maxPower);
    }

    public double getHeadingRelativeToBoard() {
        double absoluteHeading = getCurrentHeadingTeleOp();
        double relativeHeading;
        if (isRedAlliance) {
            relativeHeading = absoluteHeading + 90;
        } else {
            relativeHeading = absoluteHeading - 90;
        }
        return relativeHeading;
    }

    public void initForTeleOp() {
        // initialize robot class
        setUpDrivetrainMotors();
        setUpIntakeOuttake();
        planeLauncherServo = hardwareMap.servo.get("planeLauncherServo");
        planeLauncher = hardwareMap.dcMotor.get("planeLauncher");
        planeLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        planeLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stackAttachment = hardwareMap.servo.get("stackAttachment");

        openHook();
        outtake.closeClamp(false);
        outtake.trayToIntakePos(true);
        moveFingerUp();
        opMode.sleep(100);
        planeLauncher.setPower(0);
        planeLauncherServo.setPosition(planeServoDisengage);
        //planeLauncher.setPosition(0.6);
        moveLinearSlideByTicksBlocking(0);
    }

    public void teleOpWhileLoop(Gamepad gamepad1, Gamepad gamepad2) {

        boolean hangingMode = false;
        int TRIGGER_PRESSED = 0; // TODO: test
        int frontFacing = 1;
        boolean slowMode = false;

        //doubles for amount of input for straight, turning, and mecanuming variables
        double straight;
        double turning;
        double mecanuming;

        double fLeftPower;
        double fRightPower;
        double bLeftPower;
        double bRightPower;
        double maxPower;
        double scale;

        double fLeftPowerPrev = 0;
        double fRightPowerPrev = 0;
        double bLeftPowerPrev = 0;
        double bRightPowerPrev = 0;
        boolean willOpenClamp = false;

        double lsPowerSlow = 0.3;
        double lsPowerFast = 1;
        double lsStayUpAddPower = 0.1;
        trayAngleSlope = -0.004;
        teleOpTuneValueTrayAngle = 0;
        double relativeHeadingToBoard = getCurrentHeadingTeleOp();
        double trayAngleServoPos = trayAngleDefault;
        boolean dpadDownPreviousValue = false;
        boolean stackAttachmentOut = false;

        final long KONSTANT_TIME_GAP_MILLISECONDS = 100;
        double launcherSpeedTarget = 0.5;
        boolean droneShouldHaveLaunched = false;
        //max power = 2800ticks/sec
        //6000rpm rotations per second = 6000/60 rotation per millisecond = (6000/60)/1000 ticks per millisecond = 0.1*28
        final double targetVelocity = ((0.028*KONSTANT_TIME_GAP_MILLISECONDS)*launcherSpeedTarget);
        double currentVelocity;
        double initialPlaneLauncherPower = 0.2;
        double planeLauncherTicksBeforeSleep = planeLauncher.getCurrentPosition();
        double planeLauncherTicksAfterSleep = planeLauncher.getCurrentPosition();
        ElapsedTime timeSincePressed = new ElapsedTime();
        double timeSincePressedRounded = Math.round(timeSincePressed.time());
        timeSincePressed.reset();

        while (opMode.opModeIsActive()) {

            if (lsFront.getCurrentPosition() > 857) {
                lsStayUpAddPower = 0.1;
            } else if (lsFront.getCurrentPosition() < 856) {
                lsStayUpAddPower = 0;
            }

            // GAMEPAD 1: DRIVER CONTROLS

            /*
            // b aligns bot to board - disabled because setheading loops imu
            if (gamepad1.b) {
                if (isRedAlliance) {
                    setHeading(-90, 0.7);
                } else {
                    setHeading(90, 0.7);
                }
            }
            */


            //a and y switch which side is front
            if (gamepad1.a) {
                frontFacing = 1;
            } else if (gamepad1.y) {
                frontFacing = -1;
            }

            //hold right bumper for slow mode
            if (gamepad1.right_bumper) {
                slowMode = true;
            } else {
                slowMode = false;
            }

            // both bumper launches drone
            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                while (!droneShouldHaveLaunched) {
                    planeLauncher.setPower(initialPlaneLauncherPower);

                    planeLauncherTicksBeforeSleep = planeLauncher.getCurrentPosition();

                    opMode.sleep(KONSTANT_TIME_GAP_MILLISECONDS);

                    planeLauncherTicksAfterSleep = planeLauncher.getCurrentPosition();
                    planeLauncherTicksAfterSleep = planeLauncherTicksBeforeSleep - planeLauncherTicksAfterSleep;
                    currentVelocity = Math.abs(planeLauncherTicksAfterSleep / KONSTANT_TIME_GAP_MILLISECONDS);

                    if ((currentVelocity >= targetVelocity) || (initialPlaneLauncherPower >= 1)) {
                        //launch
                        planeLauncherServo.setPosition(0.45);
                        droneShouldHaveLaunched = true;
                        Log.d("drone", "reach velocity: " + (currentVelocity >= targetVelocity));
                        opMode.sleep(1000);
                        break;
                    } else {
                        initialPlaneLauncherPower += 0.05;

                        Log.d("drone", "current velocity: " + currentVelocity + "target velocity: " + targetVelocity + "current power" + initialPlaneLauncherPower);
                        telemetry.addData("currentVelocity: ", currentVelocity);
                        telemetry.addData("targetVelocity: ", targetVelocity);

                    }
                }
            } else {
                planeLauncher.setPower(0);
                //not launch
                planeLauncherServo.setPosition(planeServoDisengage);
            }


            //dpad right - toggle stack attachment

            if (!gamepad1.dpad_right && previousStackButtonValue) {
                stackShouldBeDown = !stackShouldBeDown;
            }
            previousStackButtonValue = gamepad1.dpad_right;

            if (stackShouldBeDown) {
                stackAttachmentOut();
            } else {
                stackAttachmentIn();
            }

            //setting forward and mecanum based on where the front is
            straight = (gamepad1.left_stick_y)*(gamepad1.left_stick_y)*(gamepad1.left_stick_y) * frontFacing * -1;
            mecanuming = (gamepad1.left_stick_x)*(gamepad1.left_stick_x)*(gamepad1.left_stick_x) * frontFacing;

            //turning stays the same
            turning = (gamepad1.right_stick_x) * (gamepad1.right_stick_x) * (gamepad1.right_stick_x);

            //Pure Mecanum overrides straight and turn
            if (gamepad1.right_trigger != 0) {
                straight = 0;
                turning = 0;
                mecanuming = 0.7;
            } else if (gamepad1.left_trigger != 0) {
                straight = 0;
                turning = 0;
                mecanuming = -0.7;
            }

            //set powers using this input
            fLeftPower = straight + turning + mecanuming;
            fRightPower = straight - turning - mecanuming;
            bLeftPower = straight + turning - mecanuming;
            bRightPower = straight - turning + mecanuming;


            //scale powers
            maxPower = maxAbsValueDouble(fLeftPower, bLeftPower, fRightPower, bRightPower);

            if (Math.abs(maxPower) > 1) {
                scale = Math.abs(maxPower);
                fLeftPower /= scale;
                bLeftPower /= scale;
                fRightPower /= scale;
                bRightPower /= scale;
            }

            //uses different powers based on which bumper was pressed last
            if (slowMode) {
                fLeftPower *= 0.7;
                bLeftPower *= 0.7;
                fRightPower *= 0.7;
                bRightPower *= 0.7;
            }

            //set motor power ONLY if a value has changed. else, use previous value.
            if (fLeftPowerPrev != fLeftPower || fRightPowerPrev != fRightPower
                    || bLeftPowerPrev != bLeftPower || bRightPowerPrev != bRightPower) {
                drivetrain.setMotorPower(fLeftPower, fRightPower, bLeftPower, bRightPower);

                fLeftPowerPrev = fLeftPower;
                fRightPowerPrev = fRightPower;
                bLeftPowerPrev = bLeftPower;
                bRightPowerPrev = bRightPower;
            }


            // GAMEPAD 2: ARM CONTROLS

            // dpad controlling lock
            if (!gamepad2.dpad_up && previousHookButtonValue) { // up - close
                hookShouldClose = !hookShouldClose;
            }
            previousHookButtonValue = gamepad2.dpad_up;

            if (hookShouldClose) {
                closeHook();
            } else {
                openHook();
            }

            // pivoting tray
            if (gamepad2.a && gamepad2.y) { // both - stay at current
                // do nothing
            } else if (gamepad2.a) { // a - intake position
                allowTrayAngle = false;
                trayAngle.setPosition(trayAngleDefault);
                outtake.trayToIntakePos(false);
            } else if (gamepad2.y) { // y - outtake position
                allowTrayAngle = true;
                outtake.trayToOuttakePos(false);
            }

            hardStopTrayAngleBig = trayAngleDefault + 0.18;
            hardStopTrayAngleSmall = trayAngleDefault - 0.18;

            //override tray angle toggle
            if (!gamepad2.dpad_down && dpadDownPreviousValue) {

                allowTrayAngleOverride = !allowTrayAngleOverride;

                trayAngle.setPosition(trayAngleDefault);
                //Log.d("pigeon head override", Boolean.toString(allowTrayAngleOverride));
                telemetry.addData("pigeon head on", allowTrayAngle);
            } else if (allowTrayAngle && !allowTrayAngleOverride) {
                relativeHeadingToBoard = getHeadingRelativeToBoard();
                //Log.d("pigeon head override", "pigeon head on");
                //checking imu in correct range
                if ((relativeHeadingToBoard <= 60 && relativeHeadingToBoard >-60)) {
                    //-0.004
                    trayAngleServoPos = Math.min(trayAngleSlope*(relativeHeadingToBoard + teleOpTuneValueTrayAngle) + trayAngleDefault, hardStopTrayAngleBig);
                    trayAngleServoPos = Math.max(trayAngleServoPos, hardStopTrayAngleSmall);
                } else {

                    trayAngleServoPos = trayAngleDefault;
                }

                trayAngle.setPosition(trayAngleServoPos);
                Log.d("pigeon head override", String.valueOf(trayAngleServoPos));
                telemetry.addData("pigeon head on", allowTrayAngle);
            }
            dpadDownPreviousValue = gamepad2.dpad_down;

            //adjustable tray in teleop
            if (gamepad2.right_stick_x < -0.9) {
                teleOpTuneValueTrayAngle += 0.25;
            } else if (gamepad2.right_stick_x  > 0.9){
                teleOpTuneValueTrayAngle -= 0.25;
            } else if (gamepad2.right_stick_button) {
                teleOpTuneValueTrayAngle = 0;
            }

            // clamp controls
            if (gamepad2.right_trigger > TRIGGER_PRESSED) { // right trigger - close clamp
                willOpenClamp = false;
            } else if (gamepad2.right_bumper) {
                willOpenClamp = true;
            } else if (gamepad2.left_trigger > TRIGGER_PRESSED) {
                willOpenClamp = true;
            } else {
                willOpenClamp = false;
            }

            if (willOpenClamp) {
                outtake.openClamp(true, false, false);
            } else {
                outtake.closeClamp(false);
            }

            // intake regurgitate
            if (gamepad2.left_trigger > TRIGGER_PRESSED && gamepad2.left_bumper) { // both - nothing
                // do nothing
            } else if (gamepad2.left_trigger > TRIGGER_PRESSED) { // left trigger - intake
                intakeMotor.setPower(-1);
            } else if (gamepad2.left_bumper) { // left bumper - regurgitate
                intakeMotor.setPower(1);
            } else { // neither - stop
                intakeMotor.setPower(0);
            }

            //b to use slow linear slide
            if (gamepad2.b) {
                //if b is held linear slide is slow
                if (-gamepad2.left_stick_y > 0) {
                    //only if the linear slides aren't at upper the limit
                    if (lsFront.getCurrentPosition() < 3100) {
                        lsBack.setPower(lsPowerSlow);
                        lsFront.setPower(lsPowerSlow);
                    }
                } else if (-gamepad2.left_stick_y < 0) {
                    //only if the linear slides aren't at the lower limit
                    if (lsFront.getCurrentPosition() > 50 || gamepad2.x) {
                        lsBack.setPower(-lsPowerSlow);
                        lsFront.setPower(-lsPowerSlow);
                    }
                } else {
                    lsBack.setPower(0 + lsStayUpAddPower);
                    lsFront.setPower(0 + lsStayUpAddPower);
                }
            } else {
                //if b is not held linear slide is fast
                if (-gamepad2.left_stick_y > 0) {
                    //only if the linear slides aren't at upper the limit
                    if (lsFront.getCurrentPosition() < 3100) {
                        lsBack.setPower(-gamepad2.left_stick_y);
                        lsFront.setPower(-gamepad2.left_stick_y);
                    }
                } else if (-gamepad2.left_stick_y < 0) {
                    //only if the linear slides aren't at the lower limit
                    if (lsFront.getCurrentPosition() > 50 || gamepad2.x) {
                        lsBack.setPower(-gamepad2.left_stick_y);
                        lsFront.setPower(-gamepad2.left_stick_y);
                    }
                } else {
                    lsBack.setPower(0 + lsStayUpAddPower);
                    lsFront.setPower(0 + lsStayUpAddPower);
                }
            }

            if (gamepad2.x) {
                lsFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //override 0
                lsFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            /*
            //one button outtake - disabled this because it uses setheading, which loops imu
            if(gamepad2.dpad_left) {
                oneButtonOuttake(gamepad1, gamepad2);
            }

            */

        /*
            if (gamepad2.x) {
                linearSlideFlag = true;
                targetLinearSlideTicks = 1000 + getCurrentLinearSlideTicks();
            }

            if (linearSlideFlag = true) {
                boolean done = moveLinearSlidesByTicksParallel(targetLinearSlideTicks);
                linearSlideFlag = !done;
            }
            */

//            Log.d("vision ls", "teleOpWhileLoop: lsFront position " + lsFront.getCurrentPosition());
//            telemetry.addData("tray angle position servo", trayAngleServoPos);
//            telemetry.addData("imu", getCurrentHeading());
//            Log.d("trayAngle", trayAngle + "tray" + getCurrentHeading() + "heading");
            telemetry.update();
        }
    }

    public double maxAbsValueDouble(double a, double... others) {
        double max = a;

        for (double next : others) {
            if (Math.abs(next) > Math.abs(max)) {
                max = next;
            }
        }

        return max;
    }

    public void stackAttachmentOut() {
        stackAttachment.setPosition(0.7);
    }

    public void stackAttachmentIn() {
        stackAttachment.setPosition(0.01);
    }

    // intake is at the front of the bot, and camera is at the back of the bot
    // positive inches - go forward
    // negative inches - go backward
    public void straightBlocking2(double inches) {
        resetDrivetrainEncoders();
        straightController.state.integral = 0;
        straightController.state.lastPos = 0;
        straightController.state.lastError = 0;
        straightController.state.lastTime = 0;
        double currentPos = fLeft.getCurrentPosition();
        double targetPos = currentPos + straightController.convertInchesToTicks(inches);
        double power;
        double ERROR_TOLERANCE_IN_TICKS = 20;
        int counter = 0;
        double finalError;
        double velocity;

        while (opMode.opModeIsActive() && counter < 3) {

            telemetry.addData("currentPos", currentPos);
            telemetry.addData("targetPos", targetPos);
            telemetry.update();

            currentPos = fLeft.getCurrentPosition();
            power = straightController.calculatePID(currentPos, targetPos);

            velocity = straightController.getVelocity(currentPos);
            Log.d("new pid", "straightBlocking2: velocity is  " + velocity);

            if (Math.abs(straightController.state.lastError) < ERROR_TOLERANCE_IN_TICKS && velocity < 0.1) {
                counter++;
            } else { //todo: test this
                counter = 0;
            }

            drivetrain.setMotorPower(power, power, power, power);
        }

        currentPos = fLeft.getCurrentPosition();
        finalError = targetPos - currentPos;
        Log.d("new pid", "straightBlocking2: final error is " + finalError);
        drivetrain.setMotorPower(0, 0, 0, 0); // stop, to be safe
        opMode.sleep(100);
    }
    public void straightBlocking2FixHeading (double inches) {
        resetDrivetrainEncoders();
        straightController.state.integral = 0;
        straightController.state.lastPos = 0;
        straightController.state.lastError = 0;
        straightController.state.lastTime = 0;
        double currentPos = fLeft.getCurrentPosition();
        double targetPos = currentPos + straightController.convertInchesToTicks(inches);
        double power;
        double ERROR_TOLERANCE_IN_TICKS = 50;
        int counter = 0;
        double finalError;
        double velocity;
        double currentHeading;
        double headingError;
        double targetHeading = botHeading;
        double leftPower = 0;
        double rightPower = 0;
        double headingCorrectionBigginator = 1.5;

        while (opMode.opModeIsActive() && counter < 3) {

            telemetry.addData("currentPos", currentPos);
            telemetry.addData("targetPos", targetPos);
            telemetry.update();

            currentPos = fLeft.getCurrentPosition();
            power = straightController.calculatePID(currentPos, targetPos);
            //get heading & heading error

            velocity = straightController.getVelocity(currentPos);
            Log.d("new pid", "straightBlocking2: velocity is  " + velocity);

            if (Math.abs(straightController.state.lastError) < ERROR_TOLERANCE_IN_TICKS && velocity < 0.1) {
                counter++;
            } else {
                counter = 0;
            }

            currentHeading = getCurrentHeading();
            headingError = currentHeading - targetHeading;

            //correction based on the current heading
            if (Math.abs(headingError) > 0.5) {
                if (headingError < 0) {
                    //turn left
                    if (currentPos < targetPos) {
                        leftPower = 0.2 * headingCorrectionBigginator;
                        rightPower = 0.4 * headingCorrectionBigginator;
                    } else {
                        leftPower = -0.4 * headingCorrectionBigginator;
                        rightPower = -0.2 * headingCorrectionBigginator;
                    }
                } else if (headingError > 0) {
                    //turn right
                    if (currentPos < targetPos) {
                        leftPower = 0.4 * headingCorrectionBigginator;
                        rightPower = 0.2 * headingCorrectionBigginator;
                    } else {
                        leftPower = -0.2 * headingCorrectionBigginator;
                        rightPower = -0.4 * headingCorrectionBigginator;
                    }
                }
                Log.d("fixHeading", "straightBlocking2FixHeading: power is " + power);
                Log.d("fixHeading", "straightBlocking2FixHeading: leftpower is " + leftPower);
                Log.d("fixHeading", "straightBlocking2FixHeading: rightpower is " + rightPower);
                Log.d("fixHeading", "straightBlocking2FixHeading: headingerror is " + headingError);
                drivetrain.setMotorPower(leftPower, rightPower, leftPower, rightPower);
            } else {
                drivetrain.setMotorPower(power, power, power, power);
            }
        }

        currentPos = fLeft.getCurrentPosition();
        finalError = targetPos - currentPos;
        Log.d("new pid", "straightBlocking2: final error is " + finalError);
        drivetrain.setMotorPower(0, 0, 0, 0); // stop, to be safe
        opMode.sleep(100);
    }

    // intake is at the front of the bot, and camera is at the back of the bot
    // positive inches - right
    // negative inches - left
    public void mecanumBlocking2(double inches) {
        resetDrivetrainEncoders();
        fLeftMecanumController.state.integral = 0;
        fLeftMecanumController.state.lastError = 0;
        fLeftMecanumController.state.lastTime = 0;
        double currentPos = fLeft.getCurrentPosition();
        double targetPos = currentPos + (bRightMecanumController.convertInchesToTicks(inches));
        double power;
        double ERROR_TOLERANCE_IN_TICKS = 15;
        int counter = 0;

        while (opMode.opModeIsActive() && counter < 3) {
            if ((Math.abs(fLeftMecanumController.state.lastError) < ERROR_TOLERANCE_IN_TICKS)) {
                counter++;
            } else { //todo: test this
                counter = 0;
            }

            currentPos = fLeft.getCurrentPosition();
            power = fLeftMecanumController.calculatePID(currentPos, targetPos);
            drivetrain.setMotorPower(power, -1 * power, -1 * power, power);

        }

        drivetrain.setMotorPower(0, 0, 0, 0); // stop, to be safe
        opMode.sleep(100);
    }

    public void oneButtonOuttake(Gamepad gamepad1, Gamepad gamepad2) {
        double distanceToMove = -lsFront.getCurrentPosition();

        outtake.openClamp(true, false, true);
        opMode.sleep(500);
        trayAngle.setPosition(trayAngleDefault);
        allowTrayAngle = false;
        opMode.sleep(50);
        outtake.trayToIntakePos(false);
        opMode.sleep(50);

        Log.d("linear slides", "moving linear slides now");

        while(opMode.opModeIsActive() && distanceToMove < -10) {
            distanceToMove = 0 - lsFront.getCurrentPosition();
            linearSlidesMoveToZeroParallel(0);

            boolean hangingMode = false;
            int TRIGGER_PRESSED = 0; // TODO: test
            int frontFacing = 1;
            boolean slowMode = false;

            //doubles for amount of input for straight, turning, and mecanuming variables
            double straight;
            double turning;
            double mecanuming;

            double fLeftPower;
            double fRightPower;
            double bLeftPower;
            double bRightPower;
            double maxPower;
            double scale;

            double fLeftPowerPrev = 0;
            double fRightPowerPrev = 0;
            double bLeftPowerPrev = 0;
            double bRightPowerPrev = 0;

                // GAMEPAD 1: DRIVER CONTROLS

            /*
                // b aligns bot to board - disabled this because setheading loops imu
                if (gamepad1.b) {
                    if (isRedAlliance) {
                        setHeading(-90, 0.7);
                    } else {
                        setHeading(90, 0.7);
                    }
                }
             */

                //a and y switch which side is front
                if (gamepad1.a) {
                    frontFacing = 1;
                } else if (gamepad1.y) {
                    frontFacing = -1;
                }

                //hold right bumper for slow mode
                if (gamepad1.right_bumper) {
                    slowMode = true;
                } else {
                    slowMode = false;
                }




                //setting forward and mecanum based on where the front is
                straight = gamepad1.left_stick_y * frontFacing * -1;
                mecanuming = gamepad1.left_stick_x * frontFacing;

                //turning stays the same
                turning = gamepad1.right_stick_x;

                //Pure Mecanum overrides straight and turn
                if (gamepad1.right_trigger != 0) {
                    straight = 0;
                    turning = 0;
                    mecanuming = 0.7;
                } else if (gamepad1.left_trigger != 0) {
                    straight = 0;
                    turning = 0;
                    mecanuming = -0.7;
                }

                //set powers using this input
                fLeftPower = straight + turning + mecanuming;
                fRightPower = straight - turning - mecanuming;
                bLeftPower = straight + turning - mecanuming;
                bRightPower = straight - turning + mecanuming;


                //scale powers
                maxPower = maxAbsValueDouble(fLeftPower, bLeftPower, fRightPower, bRightPower);

                if (Math.abs(maxPower) > 1) {
                    scale = Math.abs(maxPower);
                    fLeftPower /= scale;
                    bLeftPower /= scale;
                    fRightPower /= scale;
                    bRightPower /= scale;
                }

                //uses different powers based on which bumper was pressed last
                if (slowMode) {
                    fLeftPower *= 0.7;
                    bLeftPower *= 0.7;
                    fRightPower *= 0.7;
                    bRightPower *= 0.7;
                }

                //set motor power ONLY if a value has changed. else, use previous value.
                if (fLeftPowerPrev != fLeftPower || fRightPowerPrev != fRightPower
                        || bLeftPowerPrev != bLeftPower || bRightPowerPrev != bRightPower) {
                    drivetrain.setMotorPower(fLeftPower, fRightPower, bLeftPower, bRightPower);

                    fLeftPowerPrev = fLeftPower;
                    fRightPowerPrev = fRightPower;
                    bLeftPowerPrev = bLeftPower;
                    bRightPowerPrev = bRightPower;
                }

                Log.d("vision ls", "teleOpWhileLoop: lsFront position " + lsFront.getCurrentPosition());

                trayAngle.setPosition(trayAngleDefault);

                if (gamepad2.left_stick_y < 0) {
                    lsFront.setPower(0);
                    lsBack.setPower(0);
                    break;
                }

                distanceToMove = 0 - lsFront.getCurrentPosition();

                if(distanceToMove > -80) {
                    lsFront.setPower(0);
                    lsBack.setPower(0);
                    //set powers using this input
                    fLeftPower = straight + turning + mecanuming;
                    fRightPower = straight - turning - mecanuming;
                    bLeftPower = straight + turning - mecanuming;
                    bRightPower = straight - turning + mecanuming;


                    //scale powers
                    maxPower = maxAbsValueDouble(fLeftPower, bLeftPower, fRightPower, bRightPower);

                    if (Math.abs(maxPower) > 1) {
                        scale = Math.abs(maxPower);
                        fLeftPower /= scale;
                        bLeftPower /= scale;
                        fRightPower /= scale;
                        bRightPower /= scale;
                    }

                    //uses different powers based on which bumper was pressed last
                    if (slowMode) {
                        fLeftPower *= 0.7;
                        bLeftPower *= 0.7;
                        fRightPower *= 0.7;
                        bRightPower *= 0.7;
                    }

                    //set motor power ONLY if a value has changed. else, use previous value.
                    if (fLeftPowerPrev != fLeftPower || fRightPowerPrev != fRightPower
                            || bLeftPowerPrev != bLeftPower || bRightPowerPrev != bRightPower) {
                        drivetrain.setMotorPower(fLeftPower, fRightPower, bLeftPower, bRightPower);

                        fLeftPowerPrev = fLeftPower;
                        fRightPowerPrev = fRightPower;
                        bLeftPowerPrev = bLeftPower;
                        bRightPowerPrev = bRightPower;
                    }
                    break;
                }


        }

    }

    public void linearSlidesMoveToZeroParallel(double slideStartingPos) {
        double distanceToMove = slideStartingPos - lsFront.getCurrentPosition();
        Log.d("ls", String.valueOf(distanceToMove));
        double p_constant = 0.001;

        if (distanceToMove < -10) {
            lsFront.setPower(distanceToMove * p_constant);
            lsBack.setPower(distanceToMove * p_constant);
        } else {
            lsFront.setPower(0);
            lsBack.setPower(0);
        }

    }

    public void boardToMiddle () {

        int polarity = (isRedAlliance) ? -1 : 1;

        switch (wantedAprTagId) {
            case 1:
                mecanumAndSlidesDownToZero(-35, slideStartingPosition);
                break;
            case 2:
                //blue center
                mecanumAndSlidesDownToZero(-30, slideStartingPosition);
                break;
            case 3:
                //blue right
                mecanumAndSlidesDownToZero(-23, slideStartingPosition);
                break;
            case 4:
                // red left
                mecanumAndSlidesDownToZero(18, slideStartingPosition);
                break;
            case 5:
                //red center
                mecanumAndSlidesDownToZero(23, slideStartingPosition);
                break;
            case 6:
                //red right
                mecanumAndSlidesDownToZero(29, slideStartingPosition);
                break;
            default:
                break;
        }

        setHeading(90 * polarity, 0.7);
    }

    public void servoToInitPositions() {
        // set clamp, hook, spike
        outtake.closeClamp(false);
        openHook();
        moveFingerDown();
        opMode.sleep(100);
    }

    public void middleToStackAndIntake(int delay) {

        outtake.openClamp(true, true, false);
        stackAttachmentOut();
        if (isRedAlliance) {
            if (isLong) {
                straightBlocking2FixHeading(107);
            } else {
                straightBlocking2FixHeading(105);
            }
        } else {
            straightBlocking2FixHeading(100);
        }

        opMode.sleep(delay);

        intakeMotor.setPower(-1);

        if (isRedAlliance) {
            Log.d("freeway", "free");
            setHeadingRelativeToBoard(-20, 0.7);
            straightBlockingWithTimer(8, true, 1, 0.7); // 3 knocks stack, 6 knocks pixel!!
            straightBlocking(2.5, false, 1);

            stackAttachmentIn();
            opMode.sleep(500);

            straightBlockingWithTimer(5, true, 1, 0.4);
            straightBlocking(3, false, 1);
            setHeadingRelativeToBoard(0, 0.7);
        } else {
            setHeadingRelativeToBoard(-20, 0.7);
            straightBlockingWithTimer(8, true, 1, 0.7); // 3 knocks stack, 6 knocks pixel!!
            straightBlocking(2.5, false, 1);

            stackAttachmentIn();
            opMode.sleep(500);

            straightBlockingWithTimer(5, true, 1, 0.4);
            straightBlocking(3, false, 1);
            setHeadingRelativeToBoard(0, 0.7);
        }

        outtake.closeClamp(true);
        opMode.sleep(100);

        intakeMotor.setPower(1);

        if (!isRedAlliance) {
            mecanumBlocking2(-1);
        }

    }

    public void stackToBoard() {

        int polarity = (isRedAlliance) ? -1 : 1;

        straightBlocking(9, false, 0.6);
        straightBlocking2FixHeading(-82);

        switch (wantedAprTagId) {
            case 1:
                //blue left
                mecanumBlocking2(23);
                break;
            case 2:
                //blue center
                mecanumBlocking2(21);
                break;
            case 3:
                //blue right
                mecanumBlocking2(30);
                break;
            case 4:
                //red left
                mecanumBlocking2(-30);
                break;
            case 5:
                //red center
                mecanumBlocking2(-21);
                break;
            case 6:
                // red right
                mecanumBlocking2(-21);
                break;
            default:
                if (isRedAlliance) {
                    mecanumBlocking2(-30);
                } else {
                    mecanumBlocking2(30);
                }
        }
        setHeading(90 * polarity, 0.7);

    }

    public void setSecondWantedTagId () {

        switch (wantedAprTagId) {
            case 1:
                secondWantedTagId = 3;
                break;
            case 2:
                secondWantedTagId = 3;
                break;
            case 3:
                secondWantedTagId = 1;
                break;
            case 4:
                secondWantedTagId = 6;
                break;
            case 5:
                secondWantedTagId = 6;
                break;
            case 6:
                secondWantedTagId = 4;
                break;
            default:
                if (isRedAlliance) {
                    secondWantedTagId = 5;
                } else {
                    secondWantedTagId = 2;
                }
        }

        Log.d("apriltag", "setSecondWantedTagId: SECOND WANTED TAG ID IS " + secondWantedTagId);
    }

    public void longMoveToBoardTruss () {
        int polarity = (isRedAlliance) ? -1 : 1;

        Log.d("vision", "path: Pos " + markerPos);
        Log.d("vision", "path: Tag " + wantedAprTagId);

        while (opMode.opModeIsActive()) {

            if (markerLocation == MARKER_LOCATION.INNER) {
                Log.d("vision", "path: Inner Spike");

                straightBlocking2(-27);

                setHeading(90 * polarity, 0.7);

                if (isRedAlliance) {
                    straightBlocking2(-2);
                } else {
                    straightBlocking2(-2);
                }
                setHeading(90 * polarity, 0.7);

                if (!testingOnBert) {
                    moveFingerUp();
                    opMode.sleep(200);
                }

                if(isRedAlliance) {
                    straightBlocking2(3);
                } else {

                }

                if (isRedAlliance) { //
                    mecanumBlocking2(-24.5);
                } else {
                    mecanumBlocking2(24.5);
                }
                setHeading(90 * polarity, 0.7);

                straightBlocking2FixHeading(-74.5);
                setHeading(90 * polarity, 0.7);

                visionPortal.setProcessorEnabled(aprilTagProcessor, true);

                if (isRedAlliance) {
                    mecanumBlocking2(21);
                } else {
                    mecanumBlocking2(-21);
                }
                setHeading(90 * polarity, 0.7);

                break;

            } else if (markerLocation == MARKER_LOCATION.OUTER) {
                Log.d("vision", "path: Outer Spike");

                if (isRedAlliance) {
                    mecanumBlocking2(20);
                } else {
                    mecanumBlocking2(-23);
                }

                setHeading(0, 0.7);

                straightBlocking2(-31);

                setHeading(90 * polarity, 0.7);

                //spike
                if (!testingOnBert) {
                    moveFingerUp();
                    opMode.sleep(200);
                }

                if(isRedAlliance){
                    straightBlocking(2, true, 0.7);
                } else {
                    //straightBlocking(2, true, 0.7); //no need to move
                }
                setHeading(90 * polarity, 0.7);

                if (isRedAlliance) {
                    mecanumBlocking2(-31);
                } else {
                    mecanumBlocking2(30);
                }
                setHeading(90 * polarity, 0.7);

                straightBlocking2FixHeading(-97);

                setHeading(90 * polarity, 0.7);

                visionPortal.setProcessorEnabled(aprilTagProcessor, true);

                if (isRedAlliance) {
                    mecanumBlocking2(27);
                } else {
                    mecanumBlocking2(-30);
                }
                setHeading(90 * polarity, 0.7);

                break;

            } else { //center, default
                Log.d("vision", "path: Center Spike");

                if (isRedAlliance) {
                    mecanumBlocking2(13);
                } else {
                    mecanumBlocking2(-13);
                }
                setHeading(0, 0.7);

                if (isRedAlliance) {
                    straightBlocking2(-36);
                } else {
                    straightBlocking2(-35);
                }

                setHeading(90 * polarity, 0.7);

                if (isRedAlliance) {
                    straightBlocking(2, false, 0.7);
                } else {

                }

                if (!testingOnBert) {
                    moveFingerUp();
                    opMode.sleep(200);
                }

                if (isRedAlliance) {
                    straightBlocking(4, true, 0.7);
                } else {
                    straightBlocking(3,true,0.7);
                }

                if (isRedAlliance) {
                    mecanumBlocking2(-36);
                } else {
                    mecanumBlocking2(33);
                }

                setHeading(90 * polarity, 0.7);

                if (isRedAlliance) {
                    straightBlocking2FixHeading(-87.5); // subtracted 4 here
                } else {
                    straightBlocking2FixHeading(-83.5);
                }
                setHeading(90 * polarity, 0.7);

                if (isRedAlliance) {
                    mecanumBlocking2(23);
                } else {
                    mecanumBlocking2(-26);
                }
                setHeading(90 * polarity, 0.7);

                break;

            }

        }
    }

    public void boardToTruss () {

        int polarity = (isRedAlliance) ? -1 : 1;

        switch (wantedAprTagId) {
            case 1:
                mecanumAndSlidesDownToZero(17, slideStartingPosition);
                break;
            case 2:
                mecanumAndSlidesDownToZero(22, slideStartingPosition); //center blue
                break;
            case 3:
                mecanumAndSlidesDownToZero(29, slideStartingPosition);
                break;
            case 4:
                mecanumAndSlidesDownToZero(-32, slideStartingPosition);
                break;
            case 5:
                mecanumAndSlidesDownToZero(-25.5, slideStartingPosition); //center red
                break;
            case 6:
                mecanumAndSlidesDownToZero(-19, slideStartingPosition);
                break;
            default: {
                if (isRedAlliance) {
                    mecanumBlocking2(-24.5);
                } else {
                    mecanumBlocking2(22);
                }
            }
        }

        setHeading(90 * polarity, 0.7);

    }

    public void trussToStackAndIntake() {

        outtake.openClamp(true, true, false);
        setHeadingRelativeToBoard(0, 0.7);
        stackAttachmentOut();
        if (isRedAlliance) {
            if (isLong) {
                straightBlocking2FixHeading(102);
            } else {
                straightBlocking2FixHeading(100);
            }

        } else {
            straightBlocking2FixHeading(102.5);
        }

        intakeMotor.setPower(-1);

        if (isRedAlliance) {
            if (isLong) {
                mecanumBlocking2(20);
            } else {
                mecanumBlocking2(22);
            }
        } else {
            mecanumBlocking2(-30);
        }

        if (isRedAlliance) {
            setHeadingRelativeToBoard(-20, 0.7);
            straightBlockingWithTimer(8, true, 1, 0.7); // 3 knocks stack, 6 knocks pixel!!
            straightBlocking(2.5, false, 1);

            stackAttachmentIn();
            opMode.sleep(500);

            straightBlockingWithTimer(5, true, 1, 0.4);
            straightBlocking(3, false, 1);

            setHeadingRelativeToBoard(0, 0.7);
        } else {
            mecanumBlocking2(6);

            straightBlocking(2, false, 1); //back

            stackAttachmentIn();
            opMode.sleep(500);

            straightBlockingWithTimer(6, true, 0.5, 0.8); //forward
            straightBlockingWithTimer(6, false, 1, 0.8); //forward
            //straightBlocking(6, true, 0.5);
            //straightBlocking(6, false, 1); //back

            setHeadingRelativeToBoard(0, 0.7);
        }

        //straightBlocking(1.5, true, 1); //forward
        //straightBlocking(1.5, false, 1); //backward

        //closeClamp(true);
        //opMode.sleep(100);


    }

    public void stackToBoardTruss() {

        int polarity = (isRedAlliance) ? -1 : 1;

        //closeClamp(true);

        mecanumBlocking2(polarity * 23);

        outtake.closeClamp(true);

        //regurgitate
        intakeMotor.setPower(1);
        opMode.sleep(100);

        straightBlocking2FixHeading(-99.5);

        Log.d("wanted tag", "wanted tag" + wantedAprTagId);
        switch (wantedAprTagId) {
            case 1:
                mecanumBlocking2(-33);
                break;
            case 2:
                mecanumBlocking2(-24); //center blue
                break;
            case 3:
                mecanumBlocking2(-24);
                break;
            case 4:
                mecanumBlocking2(18);
                break;
            case 5:
                mecanumBlocking2(24); //center red
                break;
            case 6:
                mecanumBlocking2(30);
                break;
            default: {
                if (isRedAlliance) {
                    mecanumBlocking2(24);
                    Log.d("double mec", "red");
                } else {
                    mecanumBlocking2(-24);
                    Log.d("double mec", "blue");
                }
                break;
            }

        }

        //0 is aligned to board
        setHeadingRelativeToBoard(0, 0.7);
    }

    public static double angleWrap (double angle) {
        double moddedAngle = angle % 360;
        if (moddedAngle > 180) {
            return moddedAngle - 360;
        } else if (moddedAngle <= -180) {
            return moddedAngle + 360;
        }
        return moddedAngle;
    }

    public void mecanumAndSlidesDownToZero(double inchesMove, double slideStartingPos) {
        resetDrivetrainEncoders();
        fLeftMecanumController.state.integral = 0;
        fLeftMecanumController.state.lastError = 0;
        fLeftMecanumController.state.lastTime = 0;
        double currentPos = fLeft.getCurrentPosition();
        double targetPos = currentPos + (bRightMecanumController.convertInchesToTicks(inchesMove));
        double power;
        double ERROR_TOLERANCE_IN_TICKS = 15;
        int counter = 0;

        while (opMode.opModeIsActive() && (counter < 3 || lsFront.getCurrentPosition() > 30)) {
            if ((Math.abs(fLeftMecanumController.state.lastError) < ERROR_TOLERANCE_IN_TICKS)) {
                counter++;
            } else { //todo: test this
                counter = 0;
            }

            linearSlidesMoveToZeroParallel(slideStartingPos);

            currentPos = fLeft.getCurrentPosition();
            power = fLeftMecanumController.calculatePID(currentPos, targetPos);
            drivetrain.setMotorPower(power, -1 * power, -1 * power, power);
        }

        drivetrain.setMotorPower(0, 0, 0, 0); // stop, to be safe
        opMode.sleep(100);
    }

    public static void moveMotorToTicksParallel(DcMotor motor, int ticks) {
        int errorTicks = ticks - motor.getCurrentPosition();
        double p_constant = 0.005;
        motor.setPower(errorTicks*p_constant);
    }

    public void mecanumRobotToTicksParallel(int inchesMove) {
        resetDrivetrainEncoders();
        fLeftMecanumController.state.integral = 0;
        fLeftMecanumController.state.lastError = 0;
        fLeftMecanumController.state.lastTime = 0;
        double currentPos = fLeft.getCurrentPosition();
        double targetPos = currentPos + (bRightMecanumController.convertInchesToTicks(inchesMove));
        double power;
        double ERROR_TOLERANCE_IN_TICKS = 15;
        int counter = 0;

        if (opMode.opModeIsActive() && (counter < 3 || lsFront.getCurrentPosition() > 30)) {
            if ((Math.abs(fLeftMecanumController.state.lastError) < ERROR_TOLERANCE_IN_TICKS)) {
                counter++;
            } else { //todo: test this
                counter = 0;
            }

            currentPos = fLeft.getCurrentPosition();
            power = fLeftMecanumController.calculatePID(currentPos, targetPos);
            drivetrain.setMotorPower(power, -1 * power, -1 * power, power);
        }

        drivetrain.setMotorPower(0, 0, 0, 0); // stop, to be safe
        opMode.sleep(100);
    }

    public DrivetrainPowers mecanumParallelPowerPID(double currentPos, double targetPos) {
        double power;
        double ERROR_TOLERANCE_IN_TICKS = 15;

        if (Math.abs((targetPos - currentPos)) > ERROR_TOLERANCE_IN_TICKS) {
            power = fLeftMecanumController.calculatePID(currentPos, targetPos);
//            setMotorPower(power, -1 * power, -1 * power, power);
            return new DrivetrainPowers(power, -1 * power, -1 * power, power);
        }
        return new DrivetrainPowers(0,0,0,0);
    }

    public double motorParallelPowerP(int currentPos, double targetPos, double maxPower) {
        double ERROR_TOLERANCE = 15;
        double power;
        double p_constant = 0.001;

        double error = targetPos - currentPos;

        if (Math.abs(error) >= ERROR_TOLERANCE && opMode.opModeIsActive()) {
            power = error * p_constant;
            power = Range.clip(power, -maxPower, maxPower);
            return power;
        }

        power = 0;
        return power;
    }


    public double motorParallelPowerPWait(int currentPos, double targetPos, double maxPower, double currentTimeStampMillis, double timeStampStartMillis) {
        double ERROR_TOLERANCE = 15;
        double power;
        double p_constant = 0.001;

        double error = targetPos - currentPos;

        if (Math.abs(error) >= ERROR_TOLERANCE && opMode.opModeIsActive() && (timeStampStartMillis >= currentTimeStampMillis)) {
            power = error * p_constant;
            power = Range.clip(power, -maxPower, maxPower);
            return power;
        }

        power = 0;
        return power;
    }

    public void servoParallelWait(Servo servo, double pos, double currentTimeStampMillis, double timeStampStartMillis) {
        if (timeStampStartMillis > currentTimeStampMillis) {
            servo.setPosition(pos);
        }
    }

    public boolean checkMotorPos(DcMotor motor, double ticks) {
        boolean atTicks = (Math.abs(motor.getCurrentPosition() - ticks) <= 15);

        return atTicks;
    }

    public boolean checkServoPos(Servo servo, double pos) {
        boolean atPos = (Math.abs(servo.getPosition() - pos) <= 0.05);

        return atPos;
    }

    public boolean checkStatus(DcMotor motor) {
        double goalValue = 0; // placeholder
        if (motor.getCurrentPosition() == goalValue) {
            return true;
        } else {
            return false;
        }
    }

    public void motorParallelPowerPCheckStatus(boolean checkedStatus) {
        if (checkedStatus) {
            //run
        }
    }

    public void moveFingerUp() {
        setServoPos(spikeServo, 0.5);
    }

    public void moveFingerDown () {
        setServoPos(spikeServo, 0.68);
    }

    public void setConfigPresets(int delay, PARKING_POSITION parkingPos, boolean lowOuttake) {
        autoDelayInSeconds = delay;
        parkingPosition = parkingPos;
        this.lowOuttake = lowOuttake;
    }

    public void buttonConfigAtInit (Gamepad gamepad1) {

        boolean wasPressedB = false;
        boolean wasPressedX = false;
        boolean wasPressedDPadLeft = false;
        boolean wasPressedDPadRight = false;
        boolean wasPressedDPadUp = false;

        while (!opMode.opModeIsActive()) {
            
            if (gamepad1.y) {
                this.lowOuttake = false;
            } else if (gamepad1.a) {
                this.lowOuttake = true;
            }

            // b increases wait counter by 1
            if (wasPressedB && !gamepad1.b) {
                autoDelayInSeconds++;
            }
            wasPressedB = gamepad1.b;

            // x decreases wait counter by 1
            if (wasPressedX && !gamepad1.x) {
                autoDelayInSeconds--;
            }
            wasPressedX = gamepad1.x;

            // wait counter cannot be negative
            if (autoDelayInSeconds < 0) {
                autoDelayInSeconds = 0;
            }

            // dpad left - park left
            if (wasPressedDPadLeft && !gamepad1.dpad_left) {
                parkingPosition = PARKING_POSITION.FREEWAY;
            }
            wasPressedDPadLeft = gamepad1.dpad_left;

            // dpad right - park right
            if (wasPressedDPadRight && !gamepad1.dpad_right) {
                parkingPosition = PARKING_POSITION.TRUSS;
            }
            wasPressedDPadRight = gamepad1.dpad_right;

            // dpad up - don't park
            if (wasPressedDPadUp && !gamepad1.dpad_up) {
                parkingPosition = PARKING_POSITION.BOARD;
            }
            wasPressedDPadUp = gamepad1.dpad_up;

            if (gamepad1.left_bumper && gamepad1.right_bumper) {

                this.autoDelayInSeconds = autoDelayInSeconds;

                telemetry.addLine("CONFIRMED");
                telemetry.addLine("");

                telemetry.addLine("+/- WAIT: B/X");
                telemetry.addData("WAIT TIME: ", autoDelayInSeconds);
                telemetry.addLine("");

                telemetry.addLine("PARKING: DPAD LEFT/UP/RIGHT");
                telemetry.addData("PARKING POS: ", parkingPosition);
                telemetry.addLine("");

                telemetry.addLine("OUTTAKE LOW/HIGH: GAMEPAD A/Y");
                telemetry.addData("OUTTAKE: ", (lowOuttake? "LOW":"HIGH"));


                telemetry.update();
                break;
            }

            telemetry.addLine("+/- WAIT: B/X");
            telemetry.addData("WAIT TIME: ", autoDelayInSeconds);
            telemetry.addLine("");

            telemetry.addLine("PARKING: DPAD LEFT/UP/RIGHT");
            telemetry.addData("PARKING POS: ", parkingPosition);
            telemetry.addLine("");

            telemetry.addLine("OUTTAKE LOW/HIGH: GAMEPAD A/Y");
            telemetry.addData("OUTTAKE: ", (lowOuttake? "LOW":"HIGH"));
            telemetry.addLine("");


            telemetry.addLine("PRESS BOTH BUMPERS TO CONFIRM");


            telemetry.update();
        }
    }

    public void configuredParking() {

        if (parkingPosition == PARKING_POSITION.FREEWAY) {
            boardToMiddle();
            straightBlockingWithTimer(12, false, 0.7, 1);
        } else if (parkingPosition == PARKING_POSITION.TRUSS) {
            boardToTruss();
            straightBlockingWithTimer(12, false, 0.7, 1);
        } else {
            moveLinearSlideByTicksBlocking(slideStartingPosition);
        }

        // else do nothing
    }
}

// todo write timeout for apriltag final forward
// todo turns need a timeout, and maybe other control loops
// todo tune tray pivot outtake pos
