package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import android.util.Log;
import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    public static final int CHUNK_DISTANCE_INCHES = 24;
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
    Servo flipper;
    Servo hook;
    Servo planeLauncher;
    Servo spikeServo;
    IMU imu;
    double prevError = 0;
    double prevTime = 0;
    ElapsedTime elapsedTime = new ElapsedTime();
    public MarkerDetector.MARKER_POSITION markerPos;
    int wantedAprTagId;
    private MarkerProcessor markerProcessor; //TODO: COMBINE THESE AND ALL METHODS USING THEM
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    boolean isRedAlliance;

    //for debugging :)
    double avgLeftCb;
    double avgRightCb;
    double avgCenterCb;
    double avgLeftCr;
    double avgCenterCr;
    double avgRightCr;
    double avgLeftY;
    double avgCenterY;
    double avgRightY;

    //CONSTRUCTOR
    public Robot(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry, boolean red, boolean isAutonomous) {
        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
        this.telemetry = telemetry;
        AprilTagProcessor aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        setUpDrivetrainMotors();
        setUpImu(isAutonomous);
        intake = hardwareMap.dcMotor.get("intake");
        lsFront = hardwareMap.dcMotor.get("lsFront");
        lsFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lsBack = hardwareMap.dcMotor.get("lsBack");
        lsBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        isRedAlliance = red;
        tray = hardwareMap.servo.get("arm");
        clamp = hardwareMap.servo.get("holderClamp");
        hook = hardwareMap.servo.get("linearLocker");
        planeLauncher = hardwareMap.servo.get("planeLauncher");
        spikeServo = hardwareMap.servo.get("spikeServo");
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }

    public void moveLinearSlideByTicks(double targetDistanceInTicks) {

        //target distance negative when going up
        double remainingDistanceLow = targetDistanceInTicks - lsFront.getCurrentPosition();
        double remainingDistanceZero = -lsFront.getCurrentPosition();

        while (opMode.opModeIsActive() && Math.abs(targetDistanceInTicks - lsFront.getCurrentPosition()) > 100) {
            remainingDistanceLow = targetDistanceInTicks - lsFront.getCurrentPosition();
            remainingDistanceZero = -lsFront.getCurrentPosition();

            telemetry.addData("linear slide pos in ticks", lsFront.getCurrentPosition());
            telemetry.addData("linear slides power", remainingDistanceLow * 0.02);

            lsFront.setPower(remainingDistanceLow * 0.002);
            lsBack.setPower(remainingDistanceLow * 0.002);

            telemetry.update();

            opMode.sleep(100);
        }

        lsFront.setPower(0);
        lsBack.setPower(0);


        telemetry.update();
    }

    public void setServoPosBlocking(Servo servo, double targetServoPos) {
        telemetry.addData("not started, servo pos" + servo.getDeviceName(), servo.getPosition());
        telemetry.update();
        servo.setPosition(targetServoPos);
        opMode.sleep(500);
        telemetry.addData("done, servo pos" + servo.getDeviceName(), servo.getPosition());
        telemetry.update();
    }

    public void trayToIntakePos() {
        setServoPosBlocking(tray, 0.47);
    }
    public void trayToOuttakePos() {
        setServoPosBlocking(tray, 0.78);
    }

    public void autoOuttake() {
        trayToIntakePos();
        opMode.sleep(100);
        setServoPosBlocking(clamp, 0.6); // close clamp
        opMode.sleep(100);
        moveLinearSlideByTicks(-1700); // move linear slide up
        opMode.sleep(100);
        trayToOuttakePos();
        opMode.sleep(100);
        setServoPosBlocking(clamp, 0.45); // open clamp
        opMode.sleep(100);
        moveLinearSlideByTicks(-1900); // move slide up more
        straightBlocking(2, true, 0.7); //move back 2
        opMode.sleep(100);
        trayToIntakePos(); //intake
        opMode.sleep(100);
        setServoPosBlocking(clamp, 0.5); // close clamp
        opMode.sleep(100);
        moveLinearSlideByTicks(0); // linear slide down
        opMode.sleep(500);
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
    }

    public void setUpIntakeOuttake() {
        intake = hardwareMap.dcMotor.get("intake");
        lsBack = hardwareMap.dcMotor.get("lsBack");
        lsFront = hardwareMap.dcMotor.get("lsFront");
        spikeServo = hardwareMap.servo.get("spikeServo");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lsFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lsBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        lsFront.setDirection(DcMotorSimple.Direction.REVERSE);
        lsBack.setDirection(DcMotorSimple.Direction.REVERSE);

        lsFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lsFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tray = hardwareMap.servo.get("arm");
        flipper = hardwareMap.servo.get("flipper");
    }

    public double maxAbsValueDouble(double[] values) {

        double max = -Double.MIN_VALUE;


        for (double value : values) {
            if (Math.abs(value) > Math.abs(max)) {
                max = value;
            }

        }

        return Math.abs(max);
    }

    public double[] scalePowers(double[] powers) {
        double maxPower = maxAbsValueDouble(powers);

        if (maxPower < 1) {
            return powers;
        }

        return new double[]{
                powers[0] / maxPower,
                powers[1] / maxPower,
                powers[2] / maxPower,
                powers[3] / maxPower
        };
    }

    public void setMotorPower(double fLeft, double fRight, double bLeft, double bRight) {
        this.fLeft.setPower(fLeft);
        this.fRight.setPower(fRight);
        this.bLeft.setPower(bLeft);
        this.bRight.setPower(bRight);
    }

    public void setMotorPower(double[] powers) {
        this.fLeft.setPower(powers[0]);
        this.fRight.setPower(powers[1]);
        this.bLeft.setPower(powers[2]);
        this.bRight.setPower(powers[3]);
    }

    public double getCurrentHeading() {
        double currentYaw;
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();
        currentYaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        return currentYaw;
    }

    //the desired heading must be relative to last imu reset. also, REMEMBER -180 < desired heading <= 180.
    public void setHeading(double targetAbsDegrees, double maxPower) {
    /*
        assert(targetAbsDegrees > 180);
        assert(targetAbsDegrees <= -180);

     */
        if (targetAbsDegrees == 180) {
            setHeading(179.5, maxPower);
        } else {

            YawPitchRollAngles robotOrientation;

            double KP = 0.06; //started 0.15
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

                if (power > 0 && power < minPower) {
                    power += minPower;
                } else if (power < 0 && power > -1 * minPower) {
                    power -= minPower;
                }

                //cap power
                power = Range.clip(power, -1 * maxPower, maxPower);

                setMotorPower(-1 * power, power, -1 * power, power);

                prevError = error;
                prevTime = currentTime;
            }
            setMotorPower(0, 0, 0, 0);
            opMode.sleep(100);
        }
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
        final double PI = 3.14159;
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

    public void detectMarkerPosition() {

        int visionTimeout = 2; // timeout detection after 2 seconds
        double time;

        //detect marker position
        MarkerDetector.MARKER_POSITION position = markerProcessor.getPosition();
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
        //save marker position and Apriltag position in robot class
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

            setServoPosBlocking(clamp, 0.6);
            setServoPosBlocking(hook, 0.5);
            setServoPosBlocking(spikeServo, 0.5);

            Log.d("vision", "moveToMarker: Pos " + markerPos);
            Log.d("vision", "moveToMarker: Tag " + wantedAprTagId);
            VERTICAL_TOTAL = 68;
            HORIZONTAL_TOTAL = 48;
            if ((markerPos == MarkerDetector.MARKER_POSITION.RIGHT && isRedAlliance)
                    || (markerPos == MarkerDetector.MARKER_POSITION.LEFT && !isRedAlliance)) {

                // Calculate distances

                HORIZONTAL_TOTAL = 17; //h1-h2+h3
                VERTICAL_TOTAL = 26; //v1+v2
                vertical1 = 11;
                horizontal1 = 17;
                horizontal2 = 10;
                vertical2 = VERTICAL_TOTAL - vertical1; //adjust for left
                horizontal3 = HORIZONTAL_TOTAL - horizontal1 + horizontal2;

                // Start moving
                mecanumBlocking(vertical1, !isRedAlliance, 0.5); //go right if blue, go left if red
                setHeading(0, 0.7);
                straightBlocking(horizontal1, false, 0.7); //go forward FAST
                setServoPosBlocking(spikeServo, 0.2); //lift finger
                straightBlocking(horizontal2, true, 1); //move back FAST
                setHeading(90 * polarity, 0.7); //turn
                straightBlocking(vertical2, false, 0.7);
                mecanumBlocking(horizontal3, isRedAlliance, 0.7);
                break;

            } else if ((markerPos == MarkerDetector.MARKER_POSITION.LEFT && isRedAlliance)
                    || (markerPos == MarkerDetector.MARKER_POSITION.RIGHT && !isRedAlliance)) {

                HORIZONTAL_TOTAL = 17;
                VERTICAL_TOTAL = 26;
                horizontal1 = 17;
                vertical1 = 11;
                horizontal2 = 13;
                double slantDistance = 7;

                // Start moving

                straightBlocking(horizontal1, false, 0.7);
                setHeading(-45 * polarity, 0.7);
                straightBlocking(slantDistance, false, 0.7);
                setServoPosBlocking(spikeServo, 0.2);
                straightBlocking(slantDistance, true, 0.7);
                setHeading(90 * polarity, 0.7);
                straightBlocking(vertical1, false, 0.7);
                setHeading(90 * polarity, 0.7);
                mecanumBlocking(horizontal2, isRedAlliance, 0.7);
                break;

            } else { //center, default
                Log.d("vision", "moveToMarker: Center Spike");

                // Calculate distances
                vertical1 = 6;
                vertical2 = 20;
                horizontal1 = 27;
                horizontal2 = 10;
                horizontal3 = 8;

                // Start moving
                mecanumBlocking(vertical1, !isRedAlliance, 0.5); //go left if blue, go right if red
                setHeading(0, 0.7);
                straightBlocking(horizontal1, false, 0.7); //go forward FAST
                setServoPosBlocking(spikeServo, 0.2); //lift finger
                straightBlocking(horizontal2, true, 1); //move back FAST
                setHeading(90 * polarity, 0.7); //turn
                straightBlocking(vertical2, false, 0.7); //go forward FAST
                setHeading(90 * polarity, 0.7); //turn
                mecanumBlocking(horizontal3, isRedAlliance, 0.5); //go left if blue, go right if red

                break;
            }
        }
    }
    /* Deprecated
    public void shortMoveToBoard() { //TODO: add polarity, use in ShortRedAuto.java, and TEST
        Log.d("vision", "moveToMarker: Pos " + markerPos);
        Log.d("vision", "moveToMarker: Tag " + wantedAprTagId);

        int polarity;
        double VERTICAL_TOTAL;
        double vertical1;
        double vertical4;
        double vertical6;
        double horizontal2;
        double horizontal3;
        double HORIZONTAL_TOTAL_BEFORE_CHUNKING;
        double horizontal5;
        double horizontal7;

        while (opMode.opModeIsActive()) {

            if (isRedAlliance) {
                polarity = -1;
            } else {
                polarity = 1;
            }

            setServoPosBlocking(clamp, 0.5);
            setServoPosBlocking(hook, 0.5);
            setServoPosBlocking(spikeServo, 0.5);
            if (markerPos == MarkerDetector.MARKER_POSITION.RIGHT) {
                mecanumBlocking(10, false, 0.5);
                setHeading(0, 0.25);
                setServoPosBlocking(spikeServo, 0.2);
                sleeplessStraightBlocking(18, false, 0.7);
                straightBlocking(8, true, 0.7);
                setHeading(-90, 0.7);
                straightBlocking(19, false, 0.7);
                setHeading(-90, 0.7);
                mecanumBlocking(10, true, 0.7);
                setHeading(-90, 0.7);
                break;
            } else if (markerPos == MarkerDetector.MARKER_POSITION.LEFT) {
                Log.d("vision", "shortMoveToBoard: left");
                straightBlocking(16, false, 0.25); //forward
                setHeading(45, 0.25); //turn
                setServoPosBlocking(spikeServo, 0.2);
                straightBlocking(10, false, 0.25); //forward
                setHeading(45, 0.25);
                straightBlocking(15, true, 0.9); //dropoff, backward
                setHeading(-90, 0.7); //turn
                straightBlocking(24, false, 0.7);
                setHeading(-90, 0.7);
                mecanumBlocking(18, true, 0.4);
                setHeading(-90, 0.7);
                break;
            } else { //center, default
                Log.d("vision", "shortMoveToBoard: " + markerPos);
                straightBlocking(2, false, 0.25);
                mecanumBlocking(3, false, 0.7); //go left
                setHeading(0, 0.25);
                setServoPosBlocking(spikeServo, 0.2);
                sleeplessStraightBlocking(26, false, 0.7); //go forward FAST
                straightBlocking(10, true, 0.7); //dropoff & move back
                setHeading(-90, 0.7);
                straightBlocking(22, false, 0.7);
                mecanumBlocking(4, true, 0.4);
                break;
            }
        }
    }
    */
    //contains most apriltag logic
    public void alignToBoard() {

        boolean tagVisible = false;
        boolean aligned = false;
        List<AprilTagDetection> myAprilTagDetections;
        double distanceToBoard = 15;
        int polarity = isRedAlliance ? -1 : 1;
        int PIXEL_SIZE = 4;
        int inchesMovedBack = 0;

        while (opMode.opModeIsActive()) { //while robot isnt aligned to tag
            while (!aligned) {
                //get detections
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
                                opMode.sleep(100);
                                aligned = false;
                            } else {
                                Log.d("vision", "runOpMode: aligned");
                                Log.d("vision", "alignToBoard: Range is " + detection.ftcPose.range);
                                distanceToBoard = Math.abs(detection.ftcPose.range) - PIXEL_SIZE;
                                aligned = true;
                            }
                        }
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
                    distanceToBoard = distanceToBoard + inchesMovedBack - 3;
                    Log.d("vision", "alignToBoard: distanceToBoard is " + distanceToBoard);
                    break;
                }
            }

            Log.d("vision", "alignToBoard: broken out of !aligned while loop");
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

    public void moveStraightChunkingDONT_USE_NEGATIVE(double inches, boolean forward, double maxPower, double globalHeading, double globalHeadingPower) {
        assert inches > 0;
        int chunkNumber = (int)Math.floor(inches/CHUNK_DISTANCE_INCHES);
        for (int i = 0; i < chunkNumber; i++) {
            straightBlocking(CHUNK_DISTANCE_INCHES, forward, maxPower);
            setHeading(globalHeading, globalHeadingPower);
        }
        straightBlocking(inches % CHUNK_DISTANCE_INCHES, forward, maxPower);
    }
    public void longMoveToBoard() {
        int polarity;
        double VERTICAL_TOTAL;
        double vertical1;
        double vertical4;
        double vertical6;
        double horizontal2;
        double horizontal3;
        double HORIZONTAL_TOTAL_BEFORE_CHUNKING;
        double horizontal5;
        double horizontal7;

        while (opMode.opModeIsActive()) {
            if (isRedAlliance) {
                polarity = -1;
            } else {
                polarity = 1;
            }

            setServoPosBlocking(clamp, 0.5);
            setServoPosBlocking(hook, 0.5);
            setServoPosBlocking(spikeServo, 0.5);

            Log.d("vision", "moveToMarker: Pos " + markerPos);
            Log.d("vision", "moveToMarker: Tag " + wantedAprTagId);
            VERTICAL_TOTAL = 68;
            HORIZONTAL_TOTAL_BEFORE_CHUNKING = 48;
            if ((markerPos == MarkerDetector.MARKER_POSITION.RIGHT && isRedAlliance)
                    || (markerPos == MarkerDetector.MARKER_POSITION.LEFT && !isRedAlliance)) {
                Log.d("vision", "moveToMarker: Inner Spike");

                // Calculate distances
                vertical1 = 0;
                horizontal2 = 18;
                horizontal3 = 0;
                vertical4 = 0; //adjust for left
                horizontal5 = HORIZONTAL_TOTAL_BEFORE_CHUNKING - horizontal2 + horizontal3;
                vertical6 = VERTICAL_TOTAL + vertical1 - vertical4;
                horizontal7 = HORIZONTAL_TOTAL_BEFORE_CHUNKING - 20;

                // Start moving
                straightBlocking(horizontal2, false, 0.7); //go forward FAST
                setHeading(45 * polarity, 0.25); //turn right
                sleeplessStraightBlocking(7, false, 0.25); //forward
                setServoPosBlocking(spikeServo, 0.2); //lift finger
                straightBlocking(7, true, 0.7); //dropoff, back
                setHeading(0, 0.7); //turn back
                mecanumBlocking(2, isRedAlliance, 0.7);
                straightBlocking(horizontal5, false, 0.7); //go forward & around marker
                setHeading(90 * polarity, 0.7); //turn
                moveStraightChunkingDONT_USE_NEGATIVE(vertical6, false, 0.7, 90 * polarity, 0.7);
                setHeading(90 * polarity, 0.7);
                mecanumBlocking(horizontal7, !isRedAlliance, 0.5); //mecanum directly in front of board left if blue
                setHeading(90 * polarity, 0.7);
                break;
            } else if ((markerPos == MarkerDetector.MARKER_POSITION.LEFT && isRedAlliance)
                    || (markerPos == MarkerDetector.MARKER_POSITION.RIGHT && !isRedAlliance)) {
                Log.d("vision", "moveToMarker: Outer Spike");

                // Calculate distances
                vertical1 = 11;
                horizontal2 = 18;
                horizontal3 = 10;
                vertical4 = vertical1; //adjust for left
                horizontal5 = HORIZONTAL_TOTAL_BEFORE_CHUNKING - horizontal2 + horizontal3;
                vertical6 = VERTICAL_TOTAL + vertical1 - vertical4;
                horizontal7 = HORIZONTAL_TOTAL_BEFORE_CHUNKING - 24;

                // Start moving
                mecanumBlocking(vertical1, isRedAlliance, 0.5); //go left if blue, go right if red
                setHeading(0, 0.6);
                straightBlocking(horizontal2, false, 0.7); //go forward FAST
                setServoPosBlocking(spikeServo, 0.2); //lift finger
                straightBlocking(horizontal3, true, 1); //move back FAST
                setHeading(0, 0.7);
                mecanumBlocking(vertical4, !isRedAlliance, 0.5); //move left if red
                setHeading(0, 0.7);
                straightBlocking(horizontal5, false, 0.7); //go forward & around marker
                setHeading(90 * polarity, 0.7); //turn
                moveStraightChunkingDONT_USE_NEGATIVE(vertical6, false, 0.7, 90 * polarity, 0.7);
                setHeading(90 * polarity, 0.7);
                mecanumBlocking(horizontal7, !isRedAlliance, 0.5); //mecanum directly in front of board left if blue
                setHeading(90 * polarity, 0.7);
                break;
            } else { //center, default
                Log.d("vision", "moveToMarker: Center Spike");

                // Calculate distances
                vertical1 = 6;
                horizontal2 = 26;
                horizontal3 = 6;
                vertical4 = 10;
                horizontal5 = HORIZONTAL_TOTAL_BEFORE_CHUNKING - horizontal2 + horizontal3;
                vertical6 = VERTICAL_TOTAL + vertical1 + vertical4;
                horizontal7 = HORIZONTAL_TOTAL_BEFORE_CHUNKING - 21;

                // Start moving
                mecanumBlocking(vertical1, isRedAlliance, 0.5); //go left if blue, go right if red
                setHeading(0, 0.7);
                straightBlocking(horizontal2, false, 0.7); //go forward FAST
                setServoPosBlocking(spikeServo, 0.2); //lift finger
                straightBlocking(horizontal3, true, 1); //move back FAST
                setHeading(0, 0.7);
                mecanumBlocking(vertical4, isRedAlliance, 0.5); //move left if red
                setHeading(0, 0.7);
                straightBlocking(horizontal5, false, 0.7); //go forward & around marker
                setHeading(90 * polarity, 0.7); //turn
                moveStraightChunkingDONT_USE_NEGATIVE(vertical6, false, 0.7, 90 * polarity, 0.7);
                setHeading(90 * polarity, 0.7);
                mecanumBlocking(horizontal7, !isRedAlliance, 0.5); //mecanum directly in front of board left if blue
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
        int distanceBetweenTags = 5; // inches
        while (opMode.opModeIsActive()) {
            straightBlocking(2, true, 0.7);
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

    public void sleeplessStraightBlocking(double inches, boolean forward, double maxPower) {

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
        final double PI = 3.14159;
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


}