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
    private MarkerProcessor markerProcessor;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    boolean isRedAlliance;
    boolean testingOnBert = false;
    PIDController straightController;
    PIDController fLeftMecanumController;
    PIDController bRightMecanumController;
    PIDController setHeadingController;

    //CONSTRUCTOR
    public Robot(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry, boolean red, boolean isAutonomous) {
        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
        this.telemetry = telemetry;
        AprilTagProcessor aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        setUpDrivetrainMotors();
        setUpImu(isAutonomous);
        isRedAlliance = red;
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
            stackAttachment = hardwareMap.servo.get("stackAttachment");trayAngle = hardwareMap.servo.get("trayAngle");

            // initialize controllers
            straightController = new PIDController("straight", 0.003, 0.0000005, 0.4, false);
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

            Log.d("vision ls", "moveLinearSlideByTicksBlocking: lsFront position " + lsFront.getCurrentPosition());
            Log.d("vision ls", "moving linear slide: remaining distance " + remainingDistanceLow);
            Log.d("vision ls", "moving linear slide: power " + power);
        }

        lsFront.setPower(0);
        lsBack.setPower(0);

        telemetry.update();
    }

    public void setServoPosBlocking(Servo servo, double targetServoPos) {

        servo.setPosition(targetServoPos);
    }

    public void trayToIntakePos(boolean blocking) {
        setServoPosBlocking(tray, 0.35);
        if (blocking) {
            opMode.sleep(500);
        }
    }

    public void trayToOuttakePos(boolean blocking) {
        setServoPosBlocking(tray, 0.05);
        if (blocking) {
            opMode.sleep(100);
        }
    }

    public void autoOuttake(boolean lowOuttake, double startingPosition) {

        openClamp(true, true); // drop pixel
        opMode.sleep(100);

        // move linear slide up
        if (lowOuttake) {
            moveLinearSlideByTicksBlocking(startingPosition + 1900); //2010
        } else {
            moveLinearSlideByTicksBlocking(startingPosition + 2100); //2200
        }

        //opMode.sleep(100);
        straightBlocking(4, true, 0.7); //move back 2
        if (isRedAlliance) {
            setHeading(-90, 0.7);
        } else {
            setHeading(90, 0.7);
        }
        trayToIntakePos(true); //intake pos
        moveLinearSlideByTicksBlocking(startingPosition); // linear slide down
    }

    public void setMarkerPos(MarkerDetector.MARKER_POSITION position) {
        markerPos = position;
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
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        currentYaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        return currentYaw;
    }

    // todo: change to use pidcontroller class. do we need maxpower?
    public void setHeading(double targetAbsDegrees, double maxPower) {
        if (targetAbsDegrees == 180) {
            setHeading(179.5, maxPower);
        } else {
            YawPitchRollAngles robotOrientation;
            double KP = 0.06;
            double KD = 2_500_000;
            double ERROR_TOLERANCE = 0.5; //degrees
            double currentHeading = getCurrentHeading();
            double error = targetAbsDegrees - currentHeading;
            double errorDer;
            double power;
            double currentTime;
            double minPower = 0.15;
            //while start
            while (Math.abs(error) > ERROR_TOLERANCE && opMode.opModeIsActive()) {
                robotOrientation = imu.getRobotYawPitchRollAngles();
                currentHeading = robotOrientation.getYaw(AngleUnit.DEGREES);
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
    }

    public void setTrayAngle() {
        double currentHeading = getCurrentHeading();
        //setServoPosBlocking();
    }

    public void mecanumBlocking(double inches, boolean right, double maxPower) {

        double ERROR_TOLERANCE = 10;
        double power;
        double targetTick;
        final double KP_MECANUM = 0.002;
        final double minPower = 0.3;
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

    public void detectPropEarly() {
        MarkerDetector.MARKER_POSITION position;

        while (!opMode.isStarted()) {
            //detect marker position
            position = markerProcessor.getPosition();
            //save marker position and apriltag position in robot class
            setMarkerPos(position);
            setWantedAprTagId(position, isRedAlliance ? MarkerDetector.ALLIANCE_COLOR.RED : MarkerDetector.ALLIANCE_COLOR.BLUE);
            Log.d("early vision", "detected position: " + position);
        }
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

        while (opMode.opModeIsActive()) {
            while (position == MarkerDetector.MARKER_POSITION.UNDETECTED) {
                Log.d("vision", "undetected marker, keep looking" + visionPortal.getCameraState());
                position = markerProcessor.getPosition();
                if (elapsedTime.seconds() > time + visionTimeout) {
                    position = MarkerDetector.MARKER_POSITION.CENTER;
                    Log.d("vision", "detected time out. Picking CENTER");
                    break;
                }
            }
            break;
        }
        //save marker position and apriltag position in robot class
        setMarkerPos(position);
        setWantedAprTagId(position, isRedAlliance ? MarkerDetector.ALLIANCE_COLOR.RED : MarkerDetector.ALLIANCE_COLOR.BLUE);

        //print position
        Log.d("vision", "detected position: " + position);
        telemetry.addData("position ", markerPos);
        telemetry.update();
    }

    public void shortMoveToBoard() {
        int polarity;
        double VERTICAL_TOTAL;
        double vertical1;
        double vertical2;
        double horizontal3;
        double HORIZONTAL_TOTAL;
        double horizontal1;
        double horizontal2;

        while (opMode.opModeIsActive()) {
            if (isRedAlliance) {
                polarity = -1;
            } else {
                polarity = 1;
            }

            if (!testingOnBert) {
                trayToIntakePos(true);
                closeClamp(true);
                openHook();
                setServoPosBlocking(spikeServo, 0.5);
                opMode.sleep(100);
            }

            Log.d("vision", "moveToMarker: Pos " + markerPos);
            Log.d("vision", "moveToMarker: Tag " + wantedAprTagId);
            VERTICAL_TOTAL = 68;
            HORIZONTAL_TOTAL = 48;
            if ((markerPos == MarkerDetector.MARKER_POSITION.RIGHT && isRedAlliance)
                    || (markerPos == MarkerDetector.MARKER_POSITION.LEFT && !isRedAlliance)) { //OUTER

                // Calculate distances

                HORIZONTAL_TOTAL = 18; //h1-h2+h3
                VERTICAL_TOTAL = 29; //v1+v2
                vertical1 = 11;
                horizontal1 = 20;
                horizontal2 = 10;
                vertical2 = VERTICAL_TOTAL - vertical1; //adjust for left
                horizontal3 = HORIZONTAL_TOTAL - horizontal1 + horizontal2;

                // Start moving
                mecanumBlocking(vertical1, !isRedAlliance, 0.5); //go right if blue, go left if red
                setHeading(0, 0.7);
                straightBlockingFixHeading(horizontal1, false, 0.7); //go forward FAST
                setServoPosBlocking(spikeServo, 0.2); //lift finger
                opMode.sleep(100);
                straightBlockingFixHeading(horizontal2, true, 1); //move back FAST
                setHeading(90 * polarity, 0.7); //turn
                straightBlockingFixHeading(vertical2, false, 0.7);
                mecanumBlocking(horizontal3, isRedAlliance, 0.7);
                break;

            } else if ((markerPos == MarkerDetector.MARKER_POSITION.LEFT && isRedAlliance)
                    || (markerPos == MarkerDetector.MARKER_POSITION.RIGHT && !isRedAlliance)) { //INNER

                horizontal1 = 20;
                horizontal2 = 14; //ORIGINALLY 15.
                vertical1 = 3;
                vertical2 = 26;

                // move!
                straightBlockingFixHeading(horizontal1, false, 0.7); //go forward FAST
                setHeading(-45 * polarity, 0.7); //turn right
                straightBlockingFixHeading(7, false, 0.7); //forward
                if (!testingOnBert) {
                    setServoPosBlocking(spikeServo, 0.2); //lift finger
                    // opMode.sleep(100); TIME OPTIMIZATION
                }
                straightBlockingFixHeading(7, true, 0.7); //dropoff, back
                setHeading(0, 0.7); //turn back
                mecanumBlocking(vertical1, !isRedAlliance, 0.7); //mecanum away from prop
                setHeading(90 * polarity, 0.7); //turn
                straightBlockingFixHeading(vertical2, false, 0.7);
                mecanumBlocking(horizontal2, isRedAlliance, 0.7);

                break;

                //20 forward
                //turn 45 right
                //7 forward, lift, 7 back, turn to center

            } else { //center, default
                Log.d("vision", "moveToMarker: Center Spike");

                // Calculate distances
                vertical1 = 6;
                vertical2 = 20;
                horizontal1 = 30;
                horizontal2 = 10;
                horizontal3 = 5;

                // Start moving
                mecanumBlocking(vertical1, !isRedAlliance, 0.5); //go left if blue, go right if red
                setHeading(0, 0.7);
                straightBlockingFixHeading(horizontal1, false, 0.7); //go forward FAST
                setServoPosBlocking(spikeServo, 0.2); //lift finger
                opMode.sleep(100);
                straightBlockingFixHeading(horizontal2, true, 1); //move back FAST
                setHeading(90 * polarity, 0.7); //turn
                straightBlockingFixHeading(vertical2, false, 0.7); //go forward FAST
                setHeading(90 * polarity, 0.7); //turn
                mecanumBlocking(horizontal3, isRedAlliance, 0.5); //go left if blue, go right if red

                break;
            }
        }
    }

    public void goToAnyTag() {
        List<AprilTagDetection> myAprilTagDetections;
        double distanceToBoard = 10;
        boolean tagVisible = false;
        double PIXEL_SIZE = 4;
        int numberOfDetectionsProcessed = 0;

        while (opMode.opModeIsActive()) {
            if (numberOfDetectionsProcessed > 5) {
                break;
            } else {
                numberOfDetectionsProcessed++;
            }
            myAprilTagDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : myAprilTagDetections) {
                if (detection.metadata != null) {
                    distanceToBoard = Math.abs(detection.ftcPose.range) - PIXEL_SIZE;
                    tagVisible = true;
                    Log.d("anytag", "goToAnyTag: distance to board is " + distanceToBoard);
                } else {
                    tagVisible = false;
                    Log.d("anytag", "goToAnyTag: NOT VISIBLE. distance to board is " + distanceToBoard);
                }
                if (tagVisible)
                    break;
            }
            if (tagVisible)
                break;
        }
        straightBlocking(distanceToBoard, false, 0.6);
        if (isRedAlliance) {
            setHeading(-90, 0.75);
        } else {
            setHeading(90, 0.75);
        }
    }

    //contains most apriltag logic
    public void alignToBoard() {

        boolean tagVisible = false;
        boolean aligned = false;
        List<AprilTagDetection> myAprilTagDetections;
        double distanceToBoard = 12;
        int polarity = isRedAlliance ? -1 : 1;
        double PIXEL_SIZE = 4;
        int inchesMovedBack = 0;
        int numberOfDetectionsProcessed = 0;

        while (opMode.opModeIsActive()) { //while robot isnt aligned to tag
            while (!aligned) {
                //get detections

                if (numberOfDetectionsProcessed > 5) {
                    break;
                } else {
                    numberOfDetectionsProcessed++;
                }

                myAprilTagDetections = aprilTagProcessor.getDetections();
                //process detection list
                for (AprilTagDetection detection : myAprilTagDetections) {
                    if (detection.metadata != null) { //if there's an apriltag
                        if (detection.id == wantedAprTagId) { //check if its desired tag
                            Log.d("vision", "runOpMode: tag visible - this one " + wantedAprTagId);
                            tagVisible = true; //if it is, the desired tag is visible
                            //alignment based on bearing
                            if (Math.abs(detection.ftcPose.bearing) > 5) {
                                Log.d("vision", "runOpMode: bearing > +/-5, move left");
                                mecanumBlocking(1, (detection.ftcPose.bearing > 0), 0.75);
                                setHeading(90 * polarity, 0.7);
                                aligned = false;
                            } else {
                                Log.d("vision", "runOpMode: aligned");
                                Log.d("vision", "alignToBoard: Range is " + detection.ftcPose.range);
                                distanceToBoard = Math.abs(detection.ftcPose.range) - PIXEL_SIZE;
                                aligned = true;
                            }
                        }
                    } else {
                        tagVisible = false;
                    }
                }

                if (!tagVisible) { //if tag isnt visible
                    Log.d("vision", "runOpMode: tag not visible, move back");
                    straightBlocking(2, true, 0.6); //V IMP: should NOT go back into partner's space
                    setHeading(90 * polarity, 0.75);
                    inchesMovedBack = inchesMovedBack + 2;
                    tagVisible = false;
                    aligned = false;
                }

                if (!aligned && inchesMovedBack >= 4) { //TIMEOUT SITUATION
                    Log.d("vision", "alignToBoard: apriltag detection timed out");
                    distanceToBoard = distanceToBoard + inchesMovedBack;
                    Log.d("vision", "alignToBoard: distanceToBoard is " + distanceToBoard);
                    break;
                }
            }

            Log.d("vision", "alignToBoard: broken out of !aligned while loop");
            Log.d("vision", "alignToBoard: distanceToBoard is " + distanceToBoard);

            straightBlocking(distanceToBoard, false, 0.6);
            if (isRedAlliance) {
                setHeading(-90, 0.75);
            } else {
                setHeading(90, 0.75);
            }
            //TODO: do things based on apriltag id
            break;
        }
    }

    public void longMoveToBoard(boolean isJuice) {
        int polarity;
        double VERTICAL_TOTAL;
        double vertical1;
        double vertical4;
        double vertical6;
        double horizontal2;
        double horizontal3;
        double HORIZONTAL_TOTAL;
        double horizontal5;
        double horizontal7;

        while (opMode.opModeIsActive()) {
            if (isRedAlliance) {
                polarity = -1;
            } else {
                polarity = 1;
            }

            if (!testingOnBert) {
                closeClamp(false);
                openHook();
                setServoPosBlocking(spikeServo, 0.5);
                opMode.sleep(100);
            }


            Log.d("vision", "path: Pos " + markerPos);
            Log.d("vision", "path: Tag " + wantedAprTagId);

            HORIZONTAL_TOTAL = 53;
            VERTICAL_TOTAL = 76;

            if ((markerPos == MarkerDetector.MARKER_POSITION.RIGHT && isRedAlliance)
                    || (markerPos == MarkerDetector.MARKER_POSITION.LEFT && !isRedAlliance)) {
                Log.d("vision", "path: Inner Spike");
                // calculate distances
                vertical1 = 2;
                horizontal2 = 20;
                horizontal5 = HORIZONTAL_TOTAL - horizontal2;
                vertical6 = VERTICAL_TOTAL + vertical1;
                horizontal7 = HORIZONTAL_TOTAL - 27;

                // move!
                straightBlockingFixHeading(horizontal2, false, 0.7); //go forward FAST
                setHeading(45 * polarity, 0.25); //turn right
                straightBlockingFixHeading(7, false, 0.25); //forward
                if (!testingOnBert) {
                    setServoPosBlocking(spikeServo, 0.2); //lift finger
                    opMode.sleep(100);
                }
                straightBlockingFixHeading(7, true, 0.7); //dropoff, back
                setHeading(0, 0.7); //turn back
                mecanumBlocking(vertical1, isRedAlliance, 0.7);
                straightBlockingFixHeading(horizontal5, false, 0.7); //go forward & around marker
                setHeading(90 * polarity, 0.7); //turn
                if (isJuice) {
                    opMode.sleep(10000);
                }
                straightBlockingFixHeading(vertical6, false, 0.7);
                //moveStraightChunkingDONT_USE_NEGATIVE(vertical6, false, 0.7, 90 * polarity, 0.7);
                setHeading(90 * polarity, 0.7);
                mecanumBlocking(horizontal7, !isRedAlliance, 0.5); //mecanum directly in front of board left if blue
                setHeading(90 * polarity, 0.7);
                break;
            } else if ((markerPos == MarkerDetector.MARKER_POSITION.LEFT && isRedAlliance)
                    || (markerPos == MarkerDetector.MARKER_POSITION.RIGHT && !isRedAlliance)) {
                Log.d("vision", "path: Outer Spike");

                // Calculate distances
                vertical1 = 11;
                horizontal2 = 20;
                horizontal3 = 12;
                vertical4 = 9; //adjust for left
                horizontal5 = HORIZONTAL_TOTAL - horizontal2 + horizontal3;
                vertical6 = VERTICAL_TOTAL + vertical1 - vertical4;
                horizontal7 = HORIZONTAL_TOTAL - 35;

                // Start moving
                mecanumBlocking(vertical1, isRedAlliance, 0.5); //go left if blue, go right if red
                setHeading(0, 0.6);
                straightBlockingFixHeading(horizontal2, false, 0.7); //go forward FAST
                if (!testingOnBert) {
                    setServoPosBlocking(spikeServo, 0.2); //lift finger
                    opMode.sleep(100);
                }
                straightBlockingFixHeading(horizontal3, true, 1); //move back FAST
                setHeading(0, 0.7);
                mecanumBlocking(vertical4, !isRedAlliance, 0.5); //move left if red
                setHeading(0, 0.7);
                straightBlockingFixHeading(horizontal5, false, 0.7); //go forward & around marker
                setHeading(90 * polarity, 0.7); //turn
                if (isJuice) {
                    opMode.sleep(10000);
                }
                straightBlockingFixHeading(vertical6, false, 0.7);
                setHeading(90 * polarity, 0.7);
                mecanumBlocking(horizontal7, !isRedAlliance, 0.5); //mecanum directly in front of board left if blue
                setHeading(90 * polarity, 0.7);
                break;
            } else { //center, default
                Log.d("vision", "path: Center Spike");

                // Calculate distances
                vertical1 = 6;
                horizontal2 = 30;
                horizontal3 = 10;
                vertical4 = 13;
                horizontal5 = HORIZONTAL_TOTAL - horizontal2 + horizontal3;
                vertical6 = VERTICAL_TOTAL + vertical1 + vertical4;
                horizontal7 = HORIZONTAL_TOTAL - 30;

                // Start moving
                mecanumBlocking(vertical1, isRedAlliance, 0.7); //go left if blue, go right if red
                setHeading(0, 0.7);
                straightBlockingFixHeading(horizontal2, false, 0.5); //go forward
                if (!testingOnBert) {
                    setServoPosBlocking(spikeServo, 0.2); //lift finger
                    opMode.sleep(100);
                }
                straightBlockingFixHeading(horizontal3, true, 1); //move back FAST
                setHeading(0, 0.7);
                mecanumBlocking(vertical4, isRedAlliance, 0.7); //move left if red
                setHeading(0, 0.7);
                straightBlockingFixHeading(horizontal5, false, 0.8); //go forward & around marker
                setHeading(90 * polarity, 0.7); //turn
                if (isJuice) {
                    opMode.sleep(10000);
                }
                straightBlockingFixHeading(vertical6, false, 0.8);
                setHeading(90 * polarity, 0.7);
                mecanumBlocking(horizontal7, !isRedAlliance, 0.7); //mecanum directly in front of board left if blue
                setHeading(90 * polarity, 0.7);
                break;
            }
        }
    }

    public void parkBot(boolean longPath) {
        /*
         * Where to park depends on 3 factors
         * long or short run  (not optimized for combining this yet)
         * Red or blue alliance
         * position of april tag
         * There are two options to park
         * Park near wall
         * Park near center
         */

        int parkDistance = 25; // distance from center tag in inches
        int distanceBetweenTags = 4; // inches
        while (opMode.opModeIsActive()) {
            if (longPath && isRedAlliance || !longPath && !isRedAlliance) {
                // move left to park
                if (markerPos == MarkerDetector.MARKER_POSITION.LEFT) {
                    mecanumBlocking(parkDistance - distanceBetweenTags, true, 0.5);
                } else if (markerPos == MarkerDetector.MARKER_POSITION.RIGHT) {
                    mecanumBlocking(parkDistance + distanceBetweenTags, true, 0.5);
                } else { // center tag
                    mecanumBlocking(parkDistance, true, 0.5);
                }
            } else {
                // move right to park
                if (markerPos == MarkerDetector.MARKER_POSITION.LEFT) {
                    mecanumBlocking(parkDistance + distanceBetweenTags, false, 0.5);
                } else if (markerPos == MarkerDetector.MARKER_POSITION.RIGHT) {
                    mecanumBlocking(parkDistance - distanceBetweenTags, false, 0.5);
                } else { // center tag
                    mecanumBlocking(parkDistance, false, 0.5);
                }
            }
            straightBlocking(9, false, 0.7);
            break;
        }
    }

    public void straightBlockingFixHeading(double inches, boolean forward, double maxPower) {

        double ERROR_TOLERANCE_IN_TICKS = 15;
        double power;
        double leftPower;
        double rightPower;
        double prevLeftPower = 0;
        double prevRightPower = 0;
        double endTick;
        final double KP = 10; //started 100
        final double KD = 0;
        final double minPower = 0.2;

        //inch to tick
        final double wheelDiaMm = 96;
        final double PI = Math.PI;
        final double wheelCircIn = wheelDiaMm * PI / 25.4; //~11.87
        final double IN_TO_TICK = 537 / wheelCircIn;  //45.24

        double errorDer;
        double currentTime;
        double currentHeading;
        double targetHeading = botHeading;
        double headingError;
        double currentTick;

        if (forward) {
            currentTick = fLeft.getCurrentPosition();
        } else {
            currentTick = bRight.getCurrentPosition();
        }

        double tickRange = inches * IN_TO_TICK;

        int counter = 0;

        //define desired position
        if (forward) {
            endTick = currentTick + tickRange;
        } else {
            endTick = currentTick - tickRange;
        }

        double tickError = endTick - currentTick;
        double error;
        boolean setPrevTime = false; //this becomes true when previous time has been set

        while (counter < 10 && opMode.opModeIsActive()) {
            if (Math.abs(tickError) < ERROR_TOLERANCE_IN_TICKS) {
                counter++;
            }

            currentTime = SystemClock.elapsedRealtimeNanos();
            tickError = endTick - currentTick;
            error = tickError / tickRange;

            //don't use d component in first loop
            if (setPrevTime) {
                errorDer = (error - prevError) / (currentTime - prevTime);
            } else {
                errorDer = 0;
            }

            //find power using PID
            power = (KP * error * error * error) + (KD * errorDer);

            //make sure there is enough power
            if (error > 0 && power < minPower) {
                power = minPower;
            } else if (error < 0 && power > (-1 * minPower)) {
                power = (-1 * minPower);
            }

            //clip power
            power = Range.clip(power, -1 * maxPower, maxPower);

            //get heading & heading error
            currentHeading = getCurrentHeading();
            headingError = currentHeading - targetHeading;

            //default l/r power
            leftPower = power;
            rightPower = power;

            //correction based on the current heading
            if (Math.abs(headingError) > 1) {
                if (headingError < 0) {
                    //turn left
                    if (currentTick < endTick) {
                        leftPower = 0.1;
                        rightPower = 0.5;
                    } else {
                        leftPower = -0.5;
                        rightPower = -0.1;
                    }
                } else if (headingError > 0) {
                    //turn right
                    if (currentTick < endTick) {
                        leftPower = 0.5;
                        rightPower = 0.1;
                    } else {
                        leftPower = -0.1;
                        rightPower = -0.5;
                    }
                }
            }

            //set powers
            if (leftPower != prevLeftPower || rightPower != prevRightPower) {
                setMotorPower(leftPower, rightPower, leftPower, rightPower);
            }

            prevRightPower = rightPower;
            prevLeftPower = leftPower;

            //set variables

            if (forward) {
                currentTick = fLeft.getCurrentPosition();
            } else {
                currentTick = bRight.getCurrentPosition();
            }

            prevError = error;
            prevTime = currentTime;

            //when prev time is set, d component can be used - 2nd loop onward
            setPrevTime = true;
        }
        //stop, sleep
        setMotorPower(0, 0, 0, 0);
        opMode.sleep(100);
        currentHeading = getCurrentHeading();
        currentTick = fLeft.getCurrentPosition();
        Log.d("pid", "straightBlockingFixHeading: final currentTick is " + currentTick);
        Log.d("pid turn", "straightBlockingFixHeading: currentHeading is " + currentHeading);
    }

    public void closeClamp(boolean blocking) {
        setServoPosBlocking(clamp, 0.55);
        if (blocking) {
            opMode.sleep(300);
        }
    }

    public void openClamp(boolean wide, boolean blocking) {
        if (wide) {
            setServoPosBlocking(clamp, 0.472);
        } else {
            setServoPosBlocking(clamp, 0.51);
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

    public void initForTeleOp() {

        // initialize robot class
        setUpDrivetrainMotors();
        setUpIntakeOuttake();
        //planeLauncher = hardwareMap.servo.get("planeLauncher");
        planeLauncher = hardwareMap.dcMotor.get("planeLauncher");
        stackAttachment = hardwareMap.servo.get("stackAttachment");

        openHook();
        closeClamp(false);
        trayToIntakePos(true);
        opMode.sleep(100);
        planeLauncher.setPower(0);
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

        double targetLinearSlideTicks = 0;

        boolean allowTrayAngle = false;
        double currentHeading = getCurrentHeading();
        double trayAngleServoPos = 0.52;

        boolean linearSlideFlag = false;
        while (opMode.opModeIsActive()) {

            if(lsFront.getCurrentPosition() > 857) {
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
                planeLauncher.setPower(-1);
            } else {
                planeLauncher.setPower(0);
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
            if (gamepad2.dpad_up) { // up - close
                closeHook();
            } else if (gamepad2.dpad_down) { // down - open
                openHook();
            }

            // pivoting tray
            if (gamepad2.a && gamepad2.y) { // both - stay at current
                // do nothing
            } else if (gamepad2.a) { // a - intake position
                allowTrayAngle = false;
                trayAngle.setPosition(0.52);
                trayToIntakePos(false);
            } else if (gamepad2.y) { // y - outtake position
                allowTrayAngle = true;
                trayToOuttakePos(false);
            }

            if (allowTrayAngle) {
                currentHeading = getCurrentHeading();
                trayAngleServoPos = -0.00413*currentHeading + 0.15;
                trayAngle.setPosition(trayAngleServoPos);
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
                openClamp(true, false);
            } else {
                closeClamp(false);
            }

            // intake regurgitate
            if (gamepad2.left_trigger > TRIGGER_PRESSED && gamepad2.left_bumper) { // both - nothing
                // do nothing
            } else if (gamepad2.left_trigger > TRIGGER_PRESSED) { // left trigger - intake
                intake.setPower(-0.7);
            } else if (gamepad2.left_bumper) { // left bumper - regurgitate
                intake.setPower(0.7);
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
            telemetry.addData("tray angle position servo", trayAngle);
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
        stackAttachment.setPosition(0.8);
    }

    public void stackAttachmentIn() {
        stackAttachment.setPosition(0.1);
    }

    public void autoIntake() {
        int count = 0;
        openClamp(true, false);
        intake.setPower(-1);
        while (count < 3) {
            setMotorPower(0.3, 0.3, 0.3, 0.3);
            opMode.sleep(500);
            setMotorPower(-0.3, -0.3, -0.3, -0.3);
            opMode.sleep(300);
            count++;
        }

        setMotorPower(0, 0, 0, 0);
        intake.setPower(0);
        closeClamp(true);
    }

    public void fastStraightFixHeading(double inches, boolean forward, double maxPower) {

        double ERROR_TOLERANCE_IN_TICKS = 15;
        double power;
        double leftPower;
        double rightPower;
        double prevLeftPower = 0;
        double prevRightPower = 0;
        double endTick;
        final double KP = 50;
        final double KD = 0;
        final double minPower = 0.2;

        //inch to tick
        final double wheelDiaMm = 96;
        final double PI = Math.PI;
        final double wheelCircIn = wheelDiaMm * PI / 25.4; //~11.87
        final double IN_TO_TICK = 537 / wheelCircIn;  //45.24

        double errorDer;
        double currentTime;
        double currentHeading;
        double targetHeading = botHeading;
        double headingError;
        double currentTick;

        if (forward) {
            currentTick = fLeft.getCurrentPosition();
        } else {
            currentTick = bRight.getCurrentPosition();
        }

        double tickRange = inches * IN_TO_TICK;

        int counter = 0;

        //define desired position
        if (forward) {
            endTick = currentTick + tickRange;
        } else {
            endTick = currentTick - tickRange;
        }

        double tickError = endTick - currentTick;
        double error;
        boolean setPrevTime = false; //this becomes true when previous time has been set

        while (counter < 10 && opMode.opModeIsActive()) {
            if (Math.abs(tickError) < ERROR_TOLERANCE_IN_TICKS) {
                counter++;
            }

            currentTime = SystemClock.elapsedRealtimeNanos();
            tickError = endTick - currentTick;
            error = tickError / tickRange;

            //don't use d component in first loop
            if (setPrevTime) {
                errorDer = (error - prevError) / (currentTime - prevTime);
            } else {
                errorDer = 0;
            }

            //find power using PID
            power = (KP * error * error * error) + (KD * errorDer);

            //make sure there is enough power
            if (error > 0 && power < minPower) {
                power = minPower;
            } else if (error < 0 && power > (-1 * minPower)) {
                power = (-1 * minPower);
            }

            //clip power
            power = Range.clip(power, -1 * maxPower, maxPower);

            //get heading & heading error
            currentHeading = getCurrentHeading();
            headingError = currentHeading - targetHeading;

            //default l/r power
            leftPower = power;
            rightPower = power;

            //correction based on the current heading
            if (Math.abs(headingError) > 1) {
                if (headingError < 0) {
                    //turn left
                    if (currentTick < endTick) {
                        leftPower = 0.1;
                        rightPower = 0.5;
                    } else {
                        leftPower = -0.5;
                        rightPower = -0.1;
                    }
                } else if (headingError > 0) {
                    //turn right
                    if (currentTick < endTick) {
                        leftPower = 0.5;
                        rightPower = 0.1;
                    } else {
                        leftPower = -0.1;
                        rightPower = -0.5;
                    }
                }
            }

            //set powers
            if (leftPower != prevLeftPower || rightPower != prevRightPower) {
                setMotorPower(leftPower, rightPower, leftPower, rightPower);
            }

            prevRightPower = rightPower;
            prevLeftPower = leftPower;

            //set variables

            if (forward) {
                currentTick = fLeft.getCurrentPosition();
            } else {
                currentTick = bRight.getCurrentPosition();
            }

            prevError = error;
            prevTime = currentTime;

            //when prev time is set, d component can be used - 2nd loop onward
            setPrevTime = true;
        }
        //stop, sleep
        setMotorPower(0, 0, 0, 0);
        opMode.sleep(100);
        currentHeading = getCurrentHeading();
        currentTick = fLeft.getCurrentPosition();
        Log.d("pid", "straightBlockingFixHeading: final currentTick is " + currentTick);
        Log.d("pid turn", "straightBlockingFixHeading: currentHeading is " + currentHeading);
    }

    public void boardToCenter() {

        if (isRedAlliance) {
            if (markerPos == MarkerDetector.MARKER_POSITION.RIGHT) { // outer tag
                mecanumBlocking(28, isRedAlliance, 0.7);
            } else if (markerPos == MarkerDetector.MARKER_POSITION.LEFT) { // inner tag
                mecanumBlocking(16, isRedAlliance, 0.7);
            } else { // center tag
                mecanumBlocking(23, isRedAlliance, 0.7);
            }
        } else {
            if (markerPos == MarkerDetector.MARKER_POSITION.RIGHT) { // inner tag
                mecanumBlocking(21, isRedAlliance, 0.7);
            } else if (markerPos == MarkerDetector.MARKER_POSITION.LEFT) { // outer tag
                mecanumBlocking(34, isRedAlliance, 0.7);
            } else { // center tag
                mecanumBlocking(28, isRedAlliance, 0.7);
            }
        }

        if (isRedAlliance) {
            setHeading(-90, 0.7);
        } else {
            setHeading(90, 0.7);
        }
    }

    // intake is at the front of the bot, and camera is at the back of the bot
    // positive inches - go forward
    // negative inches - go backward
    public void straightBlocking2(double inches) {
        double currentPos = fLeft.getCurrentPosition();
        double targetPos = currentPos + straightController.convertInchesToTicks(inches);
        double power;
        double ERROR_TOLERANCE_IN_TICKS = 15;
        int counter = 0;

        while (opMode.opModeIsActive() && counter < 3) {
            if (Math.abs(straightController.lastError) < ERROR_TOLERANCE_IN_TICKS) {
                counter++;
            } else { //todo: test this
                counter = 0;
            }

            currentPos = fLeft.getCurrentPosition();
            power = straightController.calculatePID(currentPos, targetPos);
            setMotorPower(power, power, power, power);
        }

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

    public void mecanumBlockingTwoMotors(double inches) {
        resetDrivetrainEncoders();
        double fLeftCurrentPos = fLeft.getCurrentPosition();
        double bRightCurrentPos = bRight.getCurrentPosition();
        double fLeftTargetPos = fLeftCurrentPos + fLeftMecanumController.convertInchesToTicks(inches);
        Log.d("new pid", "mecanumBlockingTwoMotors: fl target position is " + fLeftTargetPos);
        double bRightTargetPos = bRightCurrentPos + fLeftMecanumController.convertInchesToTicks(inches);
        Log.d("new pid", "mecanumBlockingTwoMotors: br target position is " + bRightTargetPos);
        double leftPower;
        double rightPower;
        double ERROR_TOLERANCE_IN_TICKS = 15;
        int counter = 0;

        while (opMode.opModeIsActive() && counter < 3) {
            if ((Math.abs(fLeftMecanumController.lastError) < ERROR_TOLERANCE_IN_TICKS) &&
                    (Math.abs(bRightMecanumController.lastError) < ERROR_TOLERANCE_IN_TICKS)) {
                counter++;
            } else { //todo: test this
                //counter = 0;
            }

            fLeftCurrentPos = fLeft.getCurrentPosition();
            bRightCurrentPos = bRight.getCurrentPosition();

            Log.d("new pid", "mecanumBlockingTwoMotors: fl pwr " + fLeft.getPower());
            Log.d("new pid", "mecanumBlockingTwoMotors: br pwr " + bRight.getPower());
            Log.d("new pid", "mecanumBlockingTwoMotors: fl pos " + fLeftCurrentPos);
            Log.d("new pid", "mecanumBlockingTwoMotors: fr pos " + fRight.getCurrentPosition());
            Log.d("new pid", "mecanumBlockingTwoMotors: bl pos " + bRightCurrentPos);
            Log.d("new pid", "mecanumBlockingTwoMotors: br pos " + bRight.getCurrentPosition());

            leftPower = fLeftMecanumController.calculatePID(fLeftCurrentPos, fLeftTargetPos);
            rightPower = bRightMecanumController.calculatePID(bRightCurrentPos, bRightTargetPos);
            setMotorPower(leftPower, -1 * rightPower, -1 * leftPower, rightPower);
        }

        setMotorPower(0, 0, 0, 0); // stop, to be safe
        opMode.sleep(100);
    }

    public void mecanumBlockingFixHeading (double inches) {
        YawPitchRollAngles robotOrientation;
        final double IN_TO_TICK = 56.3; //todo: test
        double ERROR_TOLERANCE = 0.5; //degrees
        double ERROR_TOLERANCE_IN_TICKS = 15;
        double initialHeading = getCurrentHeading();
        double currentHeading = getCurrentHeading();
        double error = initialHeading - currentHeading;
        double angleCorrection;
        double fLeftCurrentPos = fLeft.getCurrentPosition();
        double bRightCurrentPos = bRight.getCurrentPosition();
        double fLeftTargetPos = fLeftCurrentPos + (inches * IN_TO_TICK);
        double bRightTargetPos = bRightCurrentPos + (inches * IN_TO_TICK);
        double flPower;
        double frPower;
        double blPower;
        double brPower;
        double leftPower;
        double rightPower;
        double largestPower;
        int counter = 0;

        //while start
        while (Math.abs(error) > ERROR_TOLERANCE && opMode.opModeIsActive() && counter < 5) {

            if ((Math.abs(fLeftMecanumController.lastError) < ERROR_TOLERANCE_IN_TICKS) &&
                    (Math.abs(bRightMecanumController.lastError) < ERROR_TOLERANCE_IN_TICKS)) {
                counter++;
            } else { //todo: test this
                counter = 0;
            }

            //set current position
            fLeftCurrentPos = fLeft.getCurrentPosition();
            bRightCurrentPos = bRight.getCurrentPosition();
            robotOrientation = imu.getRobotYawPitchRollAngles();
            currentHeading = robotOrientation.getYaw(AngleUnit.DEGREES);

            leftPower = fLeftMecanumController.calculatePID(fLeftCurrentPos, fLeftTargetPos);
            rightPower = bRightMecanumController.calculatePID(bRightCurrentPos, bRightTargetPos);

            angleCorrection = setHeadingController.calculatePID(currentHeading, initialHeading);

            flPower = leftPower + (-1 * angleCorrection);
            frPower = (-1 * rightPower) + angleCorrection;
            blPower = (-1 * leftPower) + (-1 * angleCorrection);
            brPower = rightPower + angleCorrection;

            largestPower = maxAbsValueDouble(flPower, frPower, blPower, brPower);
            flPower /= largestPower;
            frPower /= largestPower;
            blPower /= largestPower;
            brPower /= largestPower;

            setMotorPower(flPower, frPower, blPower, brPower);
        }
        setMotorPower(0, 0, 0, 0);
        opMode.sleep(100);
        botHeading = initialHeading;
    }

    public void setHeading2 (double targetHeading) {
        if (targetHeading == 180) {
            setHeading(179.5, targetHeading);
        } else {
            YawPitchRollAngles robotOrientation;
            double ERROR_TOLERANCE = 0.5; //degrees
            double currentHeading = getCurrentHeading();
            double power;

            //while start
            while (Math.abs(setHeadingController.lastError) > ERROR_TOLERANCE && opMode.opModeIsActive()) {
                robotOrientation = imu.getRobotYawPitchRollAngles();
                currentHeading = robotOrientation.getYaw(AngleUnit.DEGREES);
                power = setHeadingController.calculatePID(currentHeading, targetHeading);
                setMotorPower(-1 * power, power, -1 * power, power);
            }
            setMotorPower(0, 0, 0, 0);
            opMode.sleep(100);
            currentHeading = getCurrentHeading();
            botHeading = targetHeading;
            Log.d("pid", "setHeading: final heading is " + currentHeading);
        }
    }

    public void oneButtonOuttake(Gamepad gamepad1, Gamepad gamepad2) {
        double distanceToMove = 0 - lsFront.getCurrentPosition();

        openClamp(true, true);
        trayToIntakePos(false);

        while(opMode.opModeIsActive() && distanceToMove > 10) {
            distanceToMove = 0 - lsFront.getCurrentPosition();
            linearSlidesMoveToZeroParallel();

            distanceToMove = 0 - lsFront.getCurrentPosition();

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

            double targetLinearSlideTicks = 0;

            boolean linearSlideFlag = false;
            while (opMode.opModeIsActive()) {

                if(lsFront.getCurrentPosition() > 857) {
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
                    planeLauncher.setPower(-1);
                } else {
                    planeLauncher.setPower(0);
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
                if (gamepad2.dpad_up) { // up - close
                    closeHook();
                } else if (gamepad2.dpad_down) { // down - open
                    openHook();
                }

                // pivoting tray
                if (gamepad2.a && gamepad2.y) { // both - stay at current
                    // do nothing
                } else if (gamepad2.a) { // a - intake position
                    trayToIntakePos(false);
                } else if (gamepad2.y) { // y - outtake position
                    trayToOuttakePos(false);
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
                    openClamp(true, false);
                } else {
                    closeClamp(false);
                }

                // intake regurgitate
                if (gamepad2.left_trigger > TRIGGER_PRESSED && gamepad2.left_bumper) { // both - nothing
                    // do nothing
                } else if (gamepad2.left_trigger > TRIGGER_PRESSED) { // left trigger - intake
                    intake.setPower(-0.7);
                } else if (gamepad2.left_bumper) { // left bumper - regurgitate
                    intake.setPower(0.7);
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

            if (distanceToMove < 10) {
                break;
            }
        }

    }

    public void linearSlidesMoveToZeroParallel() {
        double distanceToMove = 0 - lsFront.getCurrentPosition();
        double p_constant = 0.1;

        if (distanceToMove > 10) {
            lsFront.setPower(-distanceToMove * p_constant);
            lsBack.setPower(-distanceToMove * p_constant);
        } else {
            lsFront.setPower(0);
            lsBack.setPower(0);
        }

    }

}