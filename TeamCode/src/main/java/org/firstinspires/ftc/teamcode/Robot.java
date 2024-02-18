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
    HardwareMap hardwareMap;
    Telemetry telemetry;
    LinearOpMode opMode;
    DcMotor fLeft;
    DcMotor fRight;
    DcMotor bLeft;
    DcMotor bRight;
    DcMotor intake;
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

    PIDController straightController;
    PIDController fLeftMecanumController;
    PIDController bRightMecanumController;
    PIDController setHeadingController;
    double robotX = 0;
    double robotY = 0;
    private boolean previousHookButtonValue;
    private boolean hookShouldClose;
    private boolean stackShouldBeDown;
    private boolean previousStackButtonValue;
    private double trayAngleSlope;
    private double teleOpTuneValueTrayAngle;

    public enum MARKER_LOCATION {
        INNER, CENTER, OUTER
    }
    MARKER_LOCATION markerLocation;

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
        } else if (!isLong) {
            robotX = 83;
            robotY = 17;
        }

        if (!testingOnBert) {
            intake = hardwareMap.dcMotor.get("intake");
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

    public void trayToIntakePos(boolean blocking) {
        setServoPos(tray, 0.45);
        if (blocking) {
            opMode.sleep(500);
        }
    }

    public void trayToOuttakePos(boolean blocking) {
        setServoPos(tray, 0.15);
        if (blocking) {
            opMode.sleep(100);
        }
    }

    public void autoOuttake(boolean lowOuttake, double startingPosition) {

        // move linear slide up
        if (lowOuttake) {
            trayToOuttakePos(false); // pivot tray to outtake position
            moveLinearSlideByTicksBlocking(startingPosition + 1550); //1700
            openClamp(true, true, false); // drop pixel
            opMode.sleep(200);
            moveLinearSlideByTicksBlocking(startingPosition + 1950);
        } else {
            trayToOuttakePos(false); // pivot tray to outtake position
            moveLinearSlideByTicksBlocking(startingPosition + 1900);
            openClamp(true, true, false); // drop pixel
            opMode.sleep(200);
            moveLinearSlideByTicksBlocking(startingPosition + 2300);
        }

        straightBlocking(4, true, 0.7); //move back 2

        setServoPos(trayAngle, 0.5);
        trayToIntakePos(true); //intake pos
        moveLinearSlideByTicksBlocking(startingPosition); // linear slide down
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
        intake = hardwareMap.dcMotor.get("intake");
        lsBack = hardwareMap.dcMotor.get("lsBack");
        lsFront = hardwareMap.dcMotor.get("lsFront");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lsFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lsBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        lsFront.setDirection(DcMotorSimple.Direction.REVERSE);
        lsBack.setDirection(DcMotorSimple.Direction.REVERSE);

        lsFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lsFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setMotorPower(double fLeft, double fRight, double bLeft, double bRight) {
        this.fLeft.setPower(fLeft);
        this.bRight.setPower(bRight);
        this.bLeft.setPower(bLeft);
        this.fRight.setPower(fRight);
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

            setMotorPower(-1 * power, power, -1 * power, power);
            prevError = error;
            prevTime = currentTime;
        }
        setMotorPower(0, 0, 0, 0);
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

            setMotorPower(power, -1 * power, -1 * power, power);

            error = targetTick - fLeft.getCurrentPosition();
        }
        setMotorPower(0, 0, 0, 0);
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

            setMotorPower(power, power, power, power);

            currentTick = fLeft.getCurrentPosition();
            prevTime = currentTime;
            prevError = error;
        }
        setMotorPower(0, 0, 0, 0);
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

        while ((Math.abs(error) >= ERROR_TOLERANCE || elapsedTime.milliseconds() < seconds*1000) && opMode.opModeIsActive()) {

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

            setMotorPower(power, power, power, power);

            currentTick = fLeft.getCurrentPosition();
            prevTime = currentTime;
            prevError = error;
        }
        setMotorPower(0, 0, 0, 0);
        opMode.sleep(100);
    }

    public void detectMarkerPosition() {

        //boolean isTesting = false;
        int visionTimeout = 2; // timeout detection after 2 seconds
        double time;
        MarkerDetector.MARKER_POSITION position;


        //detect marker position
        position = markerProcessor.getPosition();

        elapsedTime.reset();
        time = elapsedTime.seconds();

        while (position == MarkerDetector.MARKER_POSITION.UNDETECTED && opMode.opModeIsActive()) {
            Log.d("vision", "undetected marker, keep looking" + visionPortal.getCameraState());
            position = markerProcessor.getPosition();
            if (elapsedTime.seconds() > time + visionTimeout) {
                position = MarkerDetector.MARKER_POSITION.CENTER;
                Log.d("vision", "detected time out. Picking CENTER");
                break;
            }
        }

        //save marker position and apriltag position in robot class
        setMarkerPos(position);
        setWantedAprTagId(position, isRedAlliance ? MarkerDetector.ALLIANCE_COLOR.RED : MarkerDetector.ALLIANCE_COLOR.BLUE);
        setSecondWantedTagId();

        //print position
        Log.d("vision", "detected position: " + position);
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

                straightBlocking2(-3);
                setHeading(-90 * polarity, 0.7);

                if (!testingOnBert) {
                    moveFingerUp();
                    opMode.sleep(200);
                }

                straightBlocking2(31);

                setHeading(90 * polarity, 0.7); //TODO: the robot couldn't terminate this in one run

                visionPortal.setProcessorEnabled(aprilTagProcessor, true);

                setHeading(90 * polarity, 0.7);

                break;

            } else if (markerLocation == MARKER_LOCATION.OUTER) {
                Log.d("vision", "path: Outer Spike");

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
        straightBlocking(distanceToBoard, false, 0.4);
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

                // P2: (35, 44)

                setHeading(90 * polarity, 0.7);

                // P3: (43.5, 35.5)

                straightBlocking2(-2);
                setHeading(90 * polarity, 0.7);

                if (!testingOnBert) {
                    moveFingerUp();
                    opMode.sleep(200);
                }

                straightBlocking2(2);

                // P3: (43.5, 35.5)

                if (isRedAlliance) {
                    mecanumBlocking2(24.5);
                } else {
                    mecanumBlocking2(-27.5);
                }
                setHeading(90 * polarity, 0.7);

                // P4: (43.5, 60)

                straightBlocking2FixHeading(-73.5);
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

                // P1: (35, 17)

                if (isRedAlliance) {
                    mecanumBlocking2(21);
                } else {
                    mecanumBlocking2(-24);
                }
                setHeading(0, 0.7);

                // P2: (14, 17)

                straightBlocking2(-31);

                // P3: (14, 48)

                setHeading(90 * polarity, 0.7);

                // P4: (22.5, 39.5)

                if (!testingOnBert) {
                    moveFingerUp();
                    opMode.sleep(200);
                }

                straightBlocking(2, true, 0.7);
                setHeading(90 * polarity, 0.7);

                // P5: (19.5, 39.5)

                if (isRedAlliance) {
                    mecanumBlocking2(18);
                } else {
                    mecanumBlocking2(-21);
                }
                setHeading(90 * polarity, 0.7);

                // P6: (19.5, 57.5)

                straightBlocking2FixHeading(-94);
                setHeading(90 * polarity, 0.7);

                // P7: (117.5, 57.5)

                visionPortal.setProcessorEnabled(aprilTagProcessor, true);

                if (isRedAlliance) {
                    mecanumBlocking2(-21);
                } else {
                    mecanumBlocking2(21);
                }
                setHeading(90 * polarity, 0.7);

                // P8: (117.5, 36.5)

                break;

            } else { //center, default
                Log.d("vision", "path: Center Spike");

                // P1: (36, 17)

                if (isRedAlliance) {
                    mecanumBlocking2(13);
                } else {
                    mecanumBlocking2(-13);
                }
                setHeading(0, 0.7);

                // P2: (23, 17)

                straightBlocking2(-35);

                // P3: (23, 52)
                setHeading(90 * polarity, 0.7);

                // P4: (31.5, 44)

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

                straightBlocking2FixHeading(-85.5); // subtracted 4 here
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

    public void closeClamp(boolean blocking) {
        setServoPos(clamp, 0.57);
        if (blocking) {
            opMode.sleep(300);
        }
    }

    public void openClamp(boolean wide, boolean auto, boolean blocking) {
        if (wide) {
            if (auto) {
                setServoPos(clamp, 0.471);
            }
            else {
                setServoPos(clamp, 0.471);
            }
        } else {
            setServoPos(clamp, 0.51);
        }

        if (blocking) {
            opMode.sleep(300);
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
        double absoluteHeading = getCurrentHeading();
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
        closeClamp(false);
        trayToIntakePos(true);
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

        double trayAngleDefault = 0.5;
        trayAngleSlope = -0.004;
        teleOpTuneValueTrayAngle = 0;
        double relativeHeadingToBoard = getCurrentHeading();
        double trayAngleServoPos = trayAngleDefault;
        boolean dpadDownPreviousValue = false;
        boolean stackAttachmentOut = false;

        final long KONSTANT_TIME_GAP_MILLISECONDS = 100;
        double launcherSpeedTarget = 0.49;
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

            // b aligns bot to board
            if (gamepad1.b) {
                if (isRedAlliance) {
                    setHeading(-90, 0.7);
                } else {
                    setHeading(90, 0.7);
                }
            }


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
                setMotorPower(fLeftPower, fRightPower, bLeftPower, bRightPower);

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
                trayToIntakePos(false);
            } else if (gamepad2.y) { // y - outtake position
                allowTrayAngle = true;
                trayToOuttakePos(false);
            }

            hardStopTrayAngleBig = trayAngleDefault + 0.18;
            hardStopTrayAngleSmall = trayAngleDefault - 0.18;

            //override tray angle toggle
            if (!gamepad2.dpad_down && dpadDownPreviousValue) {

                allowTrayAngleOverride = !allowTrayAngleOverride;

                trayAngle.setPosition(trayAngleDefault);
            } else if (allowTrayAngle && !allowTrayAngleOverride) {
                relativeHeadingToBoard = getHeadingRelativeToBoard();

                //checking imu in correct range
                if ((relativeHeadingToBoard <= 60 && relativeHeadingToBoard >-60)) {
                    //-0.004
                    trayAngleServoPos = Math.min(trayAngleSlope*(relativeHeadingToBoard + teleOpTuneValueTrayAngle) + trayAngleDefault, hardStopTrayAngleBig);
                    trayAngleServoPos = Math.max(trayAngleServoPos, hardStopTrayAngleSmall);
                } else {

                    trayAngleServoPos = trayAngleDefault;
                }

                trayAngle.setPosition(trayAngleServoPos);
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
                openClamp(true, false, false);
            } else {
                closeClamp(false);
            }

            // intake regurgitate
            if (gamepad2.left_trigger > TRIGGER_PRESSED && gamepad2.left_bumper) { // both - nothing
                // do nothing
            } else if (gamepad2.left_trigger > TRIGGER_PRESSED) { // left trigger - intake
                intake.setPower(-1);
            } else if (gamepad2.left_bumper) { // left bumper - regurgitate
                intake.setPower(1);
            } else { // neither - stop
                intake.setPower(0);
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
                        lsBack.setPower(lsPowerFast);
                        lsFront.setPower(lsPowerFast);
                    }
                } else if (-gamepad2.left_stick_y < 0) {
                    //only if the linear slides aren't at the lower limit
                    if (lsFront.getCurrentPosition() > 50 || gamepad2.x) {
                        lsBack.setPower(-lsPowerFast);
                        lsFront.setPower(-lsPowerFast);
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

            //onebutton outtake
            if(gamepad2.dpad_left) {
                oneButtonOuttake(gamepad1, gamepad2);
            }

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

            Log.d("vision ls", "teleOpWhileLoop: lsFront position " + lsFront.getCurrentPosition());
            telemetry.addData("tray angle position servo", trayAngleServoPos);
            telemetry.addData("imu", getCurrentHeading());
            Log.d("trayAngle", trayAngle + "tray" + getCurrentHeading() + "heading");
            telemetry.update();
        }
    }

    private double maxAbsValueDouble(double a, double... others) {
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
        straightController.integral = 0;
        straightController.lastPos = 0;
        straightController.lastError = 0;
        straightController.lastTime = 0;
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

            if (Math.abs(straightController.lastError) < ERROR_TOLERANCE_IN_TICKS && velocity < 0.1) {
                counter++;
            } else { //todo: test this
                counter = 0;
            }

            setMotorPower(power, power, power, power);
        }

        currentPos = fLeft.getCurrentPosition();
        finalError = targetPos - currentPos;
        Log.d("new pid", "straightBlocking2: final error is " + finalError);
        setMotorPower(0, 0, 0, 0); // stop, to be safe
        opMode.sleep(100);
    }
    public void straightBlocking2FixHeading (double inches) {
        resetDrivetrainEncoders();
        straightController.integral = 0;
        straightController.lastPos = 0;
        straightController.lastError = 0;
        straightController.lastTime = 0;
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

        while (opMode.opModeIsActive() && counter < 3) {

            telemetry.addData("currentPos", currentPos);
            telemetry.addData("targetPos", targetPos);
            telemetry.update();

            currentPos = fLeft.getCurrentPosition();
            power = straightController.calculatePID(currentPos, targetPos);
            //get heading & heading error

            velocity = straightController.getVelocity(currentPos);
            Log.d("new pid", "straightBlocking2: velocity is  " + velocity);

            if (Math.abs(straightController.lastError) < ERROR_TOLERANCE_IN_TICKS && velocity < 0.1) {
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
                        leftPower = 0.2;
                        rightPower = 0.4;
                    } else {
                        leftPower = -0.4;
                        rightPower = -0.2;
                    }
                } else if (headingError > 0) {
                    //turn right
                    if (currentPos < targetPos) {
                        leftPower = 0.4;
                        rightPower = 0.2;
                    } else {
                        leftPower = -0.2;
                        rightPower = -0.4;
                    }
                }
                Log.d("fixHeading", "straightBlocking2FixHeading: power is " + power);
                Log.d("fixHeading", "straightBlocking2FixHeading: leftpower is " + leftPower);
                Log.d("fixHeading", "straightBlocking2FixHeading: rightpower is " + rightPower);
                Log.d("fixHeading", "straightBlocking2FixHeading: headingerror is " + headingError);
                setMotorPower(leftPower, rightPower, leftPower, rightPower);
            } else {
                setMotorPower(power, power, power, power);
            }
        }

        currentPos = fLeft.getCurrentPosition();
        finalError = targetPos - currentPos;
        Log.d("new pid", "straightBlocking2: final error is " + finalError);
        setMotorPower(0, 0, 0, 0); // stop, to be safe
        opMode.sleep(100);
    }

    // intake is at the front of the bot, and camera is at the back of the bot
    // positive inches - right
    // negative inches - left
    public void mecanumBlocking2(double inches) {
        resetDrivetrainEncoders();
        fLeftMecanumController.integral = 0;
        fLeftMecanumController.lastError = 0;
        fLeftMecanumController.lastTime = 0;
        double currentPos = fLeft.getCurrentPosition();
        double targetPos = currentPos + (bRightMecanumController.convertInchesToTicks(inches));
        double power;
        double ERROR_TOLERANCE_IN_TICKS = 15;
        int counter = 0;

        while (opMode.opModeIsActive() && counter < 3) {
            if ((Math.abs(fLeftMecanumController.lastError) < ERROR_TOLERANCE_IN_TICKS)) {
                counter++;
            } else { //todo: test this
                counter = 0;
            }

            currentPos = fLeft.getCurrentPosition();
            power = fLeftMecanumController.calculatePID(currentPos, targetPos);
            setMotorPower(power, -1 * power, -1 * power, power);

        }

        setMotorPower(0, 0, 0, 0); // stop, to be safe
        opMode.sleep(100);
    }

    public void oneButtonOuttake(Gamepad gamepad1, Gamepad gamepad2) {
        double distanceToMove = -lsFront.getCurrentPosition();

        openClamp(true, false, true);
        opMode.sleep(500);
        trayAngle.setPosition(0.50);
        allowTrayAngle = false;
        opMode.sleep(50);
        trayToIntakePos(false);
        opMode.sleep(50);

        Log.d("linear slides", "moving linear slides now");

        while(opMode.opModeIsActive() && distanceToMove < -10) {
            distanceToMove = 0 - lsFront.getCurrentPosition();
            linearSlidesMoveToZeroParallel();

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

                // b aligns bot to board
                if (gamepad1.b) {
                    if (isRedAlliance) {
                        setHeading(-90, 0.7);
                    } else {
                        setHeading(90, 0.7);
                    }
                }

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
                    setMotorPower(fLeftPower, fRightPower, bLeftPower, bRightPower);

                    fLeftPowerPrev = fLeftPower;
                    fRightPowerPrev = fRightPower;
                    bLeftPowerPrev = bLeftPower;
                    bRightPowerPrev = bRightPower;
                }

                Log.d("vision ls", "teleOpWhileLoop: lsFront position " + lsFront.getCurrentPosition());

                trayAngle.setPosition(0.50);

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
                        setMotorPower(fLeftPower, fRightPower, bLeftPower, bRightPower);

                        fLeftPowerPrev = fLeftPower;
                        fRightPowerPrev = fRightPower;
                        bLeftPowerPrev = bLeftPower;
                        bRightPowerPrev = bRightPower;
                    }
                    break;
                }


        }

    }

    public void linearSlidesMoveToZeroParallel() {
        double distanceToMove = 0 - lsFront.getCurrentPosition();
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
                mecanumBlocking2(-35);
                break;
            case 2:
                mecanumBlocking2(-29);
                break;
            case 3:
                mecanumBlocking2(-23);
                break;
            case 4:
                mecanumBlocking2(19);
                break;
            case 5:
                mecanumBlocking2(23);
                break;
            case 6:
                mecanumBlocking2(29);
                break;
            default:
                break;
        }

        setHeading(90 * polarity, 0.7);
    }

    public void servoToInitPositions() {
        // set clamp, hook, spike
        closeClamp(false);
        openHook();
        moveFingerDown();
        opMode.sleep(100);
    }
    public void middleToStackAndIntake() {

        openClamp(true, true, false);
        stackAttachmentOut();
        straightBlocking2FixHeading(104);
        intake.setPower(-1);

        straightBlocking(5, true, 1);
        straightBlocking(1, false, 1);
        straightBlocking(1.5, true, 1);
        straightBlocking(1.5, false, 1);
        straightBlocking(1, true, 1);
        straightBlocking(1, false, 1);
        mecanumBlocking2(1);
        straightBlocking(1.5, true, 1);
        straightBlocking(1.5, false, 1);

        closeClamp(true);
        opMode.sleep(100);

        intake.setPower(1);
        opMode.sleep(100);

        if (!isRedAlliance) {
            mecanumBlocking2(-1);
        }

    }

    public void stackToBoard() {

        int polarity = (isRedAlliance) ? -1 : 1;

        straightBlocking(9, false, 0.6);
        straightBlocking2FixHeading(-84);

        switch (wantedAprTagId) {
            case 1:
                mecanumBlocking2(23);
                break;
            case 2:
                mecanumBlocking2(23);
                break;
            case 3:
                mecanumBlocking2(30);
                break;
            case 4:
                mecanumBlocking2(-30);
                break;
            case 5:
                mecanumBlocking2(-21);
                break;
            case 6:
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

                // P1: (35, 17)

                straightBlocking2(-27);

                // P2: (35, 44)

                setHeading(90 * polarity, 0.7);

                // P3: (43.5, 35.5)

                straightBlocking2(-3);
                setHeading(90 * polarity, 0.7);

                if (!testingOnBert) {
                     moveFingerUp();
                    opMode.sleep(200);
                }

                straightBlocking2(3);

                // P3: (43.5, 35.5)

                if (isRedAlliance) { //
                    mecanumBlocking2(-24.5);
                } else {
                    mecanumBlocking2(24.5);
                }
                setHeading(90 * polarity, 0.7);

                // P4: (43.5, 60)

                straightBlocking2FixHeading(-74.5);
                setHeading(90 * polarity, 0.7);

                // P5: (120, 60)

                visionPortal.setProcessorEnabled(aprilTagProcessor, true);

                if (isRedAlliance) {
                    mecanumBlocking2(21);
                } else {
                    mecanumBlocking2(-21);
                }
                setHeading(90 * polarity, 0.7);

                // P6: (120, 30)

                break;

            } else if (markerLocation == MARKER_LOCATION.OUTER) {
                Log.d("vision", "path: Outer Spike");

                // P1: (35, 17)

                if (isRedAlliance) {
                    mecanumBlocking2(20);
                } else {
                    mecanumBlocking2(-22);
                }

                setHeading(0, 0.7);

                // P2: (14, 17)

                straightBlocking2(-31);

                // P3: (14, 48)

                setHeading(90 * polarity, 0.7);

                // P4: (22.5, 39.5)

                //spike
                if (!testingOnBert) {
                    moveFingerUp();
                    opMode.sleep(200);
                }

                straightBlocking(2, true, 0.7);
                setHeading(90 * polarity, 0.7);

                // P5: (19.5, 39.5)

                if (isRedAlliance) {
                    mecanumBlocking2(-31);
                } else {
                    mecanumBlocking2(30);
                }
                setHeading(90 * polarity, 0.7);

                // P6: (19.5, 57.5)

                straightBlocking2FixHeading(-95);
                setHeading(90 * polarity, 0.7);

                // P7: (117.5, 57.5)

                visionPortal.setProcessorEnabled(aprilTagProcessor, true);

                if (isRedAlliance) {
                    mecanumBlocking2(27);
                } else {
                    mecanumBlocking2(-30);
                }
                setHeading(90 * polarity, 0.7);

                // P8: (117.5, 36.5)

                break;

            } else { //center, default
                Log.d("vision", "path: Center Spike");

                // P1: (36, 17)

                if (isRedAlliance) {
                    mecanumBlocking2(13);
                } else {
                    mecanumBlocking2(-13);
                }
                setHeading(0, 0.7);

                // P2: (23, 17)

                straightBlocking2(-36);

                // P3: (23, 52)
                setHeading(90 * polarity, 0.7);

                // P4: (31.5, 44)

                if (!testingOnBert) {
                    moveFingerUp();
                    opMode.sleep(200);
                }

                straightBlocking(3, true, 0.7);

                // P5: (27.5, 44)
                // actually ending up at around 48 here

                if (isRedAlliance) {
                    mecanumBlocking2(-36);
                } else {
                    mecanumBlocking2(36);
                }

                setHeading(90 * polarity, 0.7);

                // P6: (27.5, 60)

                straightBlocking2FixHeading(-86.5); // subtracted 4 here
                setHeading(90 * polarity, 0.7);

                // P7: (120, 60)

                // visionPortal.setProcessorEnabled(aprilTagProcessor, true);

                if (isRedAlliance) {
                    mecanumBlocking2(23);
                } else {
                    mecanumBlocking2(-26);
                }
                setHeading(90 * polarity, 0.7);

                // P8: (120, 35)

                break;

            }

        }
    }

    public void boardToTruss () {

        int polarity = (isRedAlliance) ? -1 : 1;

        switch (wantedAprTagId) {
            case 1:
                mecanumBlocking2(17);
                break;
            case 2:
                mecanumBlocking2(22); //center blue
                break;
            case 3:
                mecanumBlocking2(29);
                break;
            case 4:
                mecanumBlocking2(-32);
                break;
            case 5:
                mecanumBlocking2(-23.5); //center red
                break;
            case 6:
                mecanumBlocking2(-19);
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

        openClamp(true, true, false);
        stackAttachmentOut();
        if (isRedAlliance) {
            straightBlocking2FixHeading(102);
        } else {
            straightBlocking2FixHeading(105);
        }

        intake.setPower(-1);

        if (isRedAlliance) {
            mecanumBlocking2(20);
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

            //straightBlockingWithTimer(3, true, 1, 0.4);
            //.        straightBlocking(3, false, 1);

            setHeadingRelativeToBoard(0, 0.7);
        } else {
            mecanumBlocking2(6);

            straightBlocking(2, false, 1); //back

            stackAttachmentIn();
            opMode.sleep(500);

            //straightBlockingWithTimer(5.5, true, 0.5, 0.8); //forward

            straightBlocking(5, true, 0.5);

            straightBlocking(5, false, 1); //back

            //closeClamp(true);

            //sideways to get second pixel
            //mecanumBlocking2(-2); //sideway

            //closeClamp(true);

            setHeadingRelativeToBoard(0, 0.7);
        }

        //straightBlocking(1.5, true, 1); //forward
        //straightBlocking(1.5, false, 1); //backward

        //closeClamp(true);
        //opMode.sleep(100);


    }

    public void stackToBoardTruss() {

        int polarity = (isRedAlliance) ? -1 : 1;

        closeClamp(true);

        mecanumBlocking2(polarity * 24);

        closeClamp(true);

        //regurgitate
        intake.setPower(1);
        opMode.sleep(100);

        straightBlocking2FixHeading(-96);

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

    public void moveFingerUp() {
        setServoPos(spikeServo, 0.5);
    }

    public void moveFingerDown () {
        setServoPos(spikeServo, 0.68);
    }

}
