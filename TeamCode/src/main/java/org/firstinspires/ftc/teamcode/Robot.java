package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

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
import org.openftc.easyopencv.OpenCvWebcam;

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
    Servo arm;
    Servo holderClamp;
    Servo flipper;

    //arm motor is not used
    DcMotor armMotor;
    IMU imu;
    double yaw;
    double prevError = 0;
    double prevTime = 0;
    MarkerDetectorRed.MARKER_POSITION markerPosRed;
    MarkerDetectorBlue.MARKER_POSITION markerPosBlue;
    int wantedAprTagId;
    private MarkerProcessorRed markerProcessorRed; //TODO: COMBINE THESE AND ALL METHODS USING THEM
    private MarkerProcessorBlue markerProcessorBlue;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    boolean redAlliance;

    //CONSTRUCTOR
    public Robot(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry, boolean redAlliance) {
        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
        this.telemetry = telemetry;
        AprilTagProcessor aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        setUpDrivetrainMotors();
        setUpImu();
        lsFront = hardwareMap.dcMotor.get("lsFront");
        lsFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lsBack = hardwareMap.dcMotor.get("lsBack");
        lsBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = hardwareMap.servo.get("arm");
        holderClamp = hardwareMap.servo.get("holderClamp");
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

            lsFront.setPower(remainingDistanceLow * 0.002);
            lsBack.setPower(remainingDistanceLow * 0.002);

            telemetry.update();

            sleep(100);
        }

        lsFront.setPower(0);
        lsBack.setPower(0);


        telemetry.update();
    }

    public void setServoPosBlocking(Servo servo, double targetServoPos) {
        telemetry.addData("not started, servo pos" + servo.getDeviceName(), servo.getPosition());
        telemetry.update();
        servo.setPosition(targetServoPos);
        sleep(500);
        telemetry.addData("done, servo pos" + servo.getDeviceName(), servo.getPosition());
        telemetry.update();
    }

    public void autoOuttake() {
        Log.d("vision", "autoOuttake: in autoouttake()");
        double armPos = 0.594; //down position
        setServoPosBlocking(arm, armPos);
        Log.d("vision", "autoOuttake: servo to down");
        double holderClampClosed = 0.5; //closed
        setServoPosBlocking(holderClamp, holderClampClosed);
        Log.d("vision", "autoOuttake: holder closed");

        moveLinearSlideByTicks(-2400);
        Log.d("vision", "autoOuttake: move lin s");

        armPos = 0.845; //up (outtake) position
        setServoPosBlocking(arm, armPos);
        Log.d("vision", "autoOuttake: servo to up");
        double holderClampOpen = 0.3;
        setServoPosBlocking(holderClamp, holderClampOpen);
        Log.d("vision", "autoOuttake: holder open");

        armPos = 0.594; //down (intake) position
        setServoPosBlocking(arm, armPos);
        moveLinearSlideByTicks(0);
        Log.d("vision", "autoOuttake: move lin s again");
    }

    public void setMarkerPosRed(MarkerDetectorRed.MARKER_POSITION position) {
        markerPosRed = position;
    }

    public void setMarkerPosBlue(MarkerDetectorBlue.MARKER_POSITION position) {
        markerPosBlue = position;
    }

    //change boolean to enum later
    public void setWantedAprTagIdRed(MarkerDetectorRed.MARKER_POSITION position, boolean redAlliance) {
        if (redAlliance) {
            switch (position) {
                case CENTER:
                    wantedAprTagId = 5;
                    break;
                case RIGHT:
                    wantedAprTagId = 6;
                    break;
                case LEFT:
                    wantedAprTagId = 4;
                    break;
                default:
                    Log.d("vision", "setWantedAprTagId: enter default");
                    wantedAprTagId = 5;
            }
        } else {
            switch (position) {
                case CENTER:
                    wantedAprTagId = 2;
                    break;
                case RIGHT:
                    wantedAprTagId = 3;
                    break;
                case LEFT:
                    wantedAprTagId = 1;
                    break;
                default:
                    Log.d("vision", "setWantedAprTagId: enter default");
                    wantedAprTagId = 2;
            }
        }
    }

    public void setWantedAprTagIdBlue(MarkerDetectorBlue.MARKER_POSITION position, boolean blueAlliance) {
        if (blueAlliance) {
            switch (position) {
                case CENTER:
                    wantedAprTagId = 2;
                    break;
                case RIGHT:
                    wantedAprTagId = 3;
                    break;
                case LEFT:
                    wantedAprTagId = 1;
                    break;
                default:
                    Log.d("vision", "setWantedAprTagId: enter default");
                    wantedAprTagId = 2;
            }
        } else {
            switch (position) {
                case CENTER:
                    wantedAprTagId = 5;
                    break;
                case RIGHT:
                    wantedAprTagId = 6;
                    break;
                case LEFT:
                    wantedAprTagId = 4;
                    break;
                default:
                    Log.d("vision", "setWantedAprTagId: enter default");
                    wantedAprTagId = 5;
            }
        }
    }

    //vision processing setup
    public void initVisionProcessingRed() {

        // Initializing marker and apriltag processors and setting them with visionportal
        markerProcessorRed = new MarkerProcessorRed(telemetry);
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setLiveViewContainerId(VisionPortal.DEFAULT_VIEW_CONTAINER_ID)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .addProcessor(markerProcessorRed)
                .build();
    }

    public void initVisionProcessingBlue() {

        // Initializing marker and apriltag processors and setting them with visionportal
        markerProcessorRed = new MarkerProcessorRed(telemetry);
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setLiveViewContainerId(VisionPortal.DEFAULT_VIEW_CONTAINER_ID)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .addProcessor(markerProcessorBlue)
                .build();
    }

    public double getAprilTagXPos(int idNumber) {

        double xValue = 0;
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        if (currentDetections != null) {
            Log.d("vision", "num of detections" + currentDetections.size());
            for (AprilTagDetection detection : currentDetections) {
                Log.d("vision", "id detections" + detection.metadata.id);
                if (detection.metadata.id == idNumber) {
                    xValue = detection.ftcPose.x;
                    telemetry.addData("id detected", idNumber);
                    break;
                }
            }
        } else {
            telemetry.addLine("not detected");
            Log.d("vision", "detected");
            return 0;
        }
        return -xValue;
    }

    public double getAprilTagRange(int idNumber) {

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        double range = 0;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata.id == idNumber) {
                range = detection.ftcPose.range;
                break;
            }

        }
        return range;
    }

    public boolean moveRelativeToAprilTagX(double mmFromAprilTag, int idNumber) {
        double inchesFromAprilTag = mmFromAprilTag / 25.4;
        boolean done = false;

        double aprilTagXPos = getAprilTagXPos(idNumber);

        double[] powerMoveNegative = calculateMecanumPower((inchesFromAprilTag - aprilTagXPos));
        double[] powerMovePositive = calculateMecanumPower(-(inchesFromAprilTag - aprilTagXPos));

        double[] stopPower = {0, 0, 0, 0};

        if (!(aprilTagXPos > inchesFromAprilTag - 0.5 &&
                aprilTagXPos < inchesFromAprilTag + 0.5)) {
            if (aprilTagXPos < inchesFromAprilTag) {
                powerMovePositive = calculateMecanumPower(-25.4 * (inchesFromAprilTag - aprilTagXPos));
                setMotorPower(powerMovePositive);
            } else {
                powerMoveNegative = calculateMecanumPower(-25.4 * (inchesFromAprilTag - aprilTagXPos));
                setMotorPower(powerMoveNegative);
            }
        } else {
            setMotorPower(stopPower);
            done = true;
        }

        telemetry.addData("power", powerMovePositive);
        telemetry.addData("power 2", powerMoveNegative);
        telemetry.addData("x ", aprilTagXPos);
        telemetry.addData("left range", aprilTagXPos > inchesFromAprilTag - 0.5);
        telemetry.addData("right range", aprilTagXPos < inchesFromAprilTag + 0.5);

        return done;
    }

    public boolean moveRelativeToAprilTagRange(double mmFromAprilTag, int idNumber) {
        double targetFromAprilTagInches = mmFromAprilTag / 25.4;
        boolean done = false;
        double aprilTagRange = getAprilTagRange(idNumber);
        double moveDistanceInches;

        if (aprilTagRange == 0) {
            moveDistanceInches = targetFromAprilTagInches;
        } else {
            moveDistanceInches = ((aprilTagRange - targetFromAprilTagInches));
        }

        double[] stopPower = {0, 0, 0, 0};
        telemetry.addData("april tag range", aprilTagRange);
        Log.d("vision", "april tag range" + aprilTagRange);
        Log.d("vision", "move distance" + moveDistanceInches);
        if (moveDistanceInches > 0.5 || moveDistanceInches < -0.5) {
            //robot is backward
            //move closer to april tag
            //move away from april tag
            straightBlocking(Math.abs(moveDistanceInches), !(moveDistanceInches > 0), 0.75);
        } else if (aprilTagRange == 0) {

            setMotorPower(stopPower);
            telemetry.addLine("not seeing april tag, not moving");
            Log.d("vision", "not seeing april tag not moving");
            done = true;
        } else {
            setMotorPower(stopPower);
            done = true;
        }

        telemetry.addData("range", getAprilTagRange(idNumber));

        return done;
    }

    public void setUpImu() {

        this.imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        imu.resetYaw();


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

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lsFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lsBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        lsFront.setDirection(DcMotorSimple.Direction.REVERSE);
        lsBack.setDirection(DcMotorSimple.Direction.REVERSE);

        lsFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lsFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hardwareMap.servo.get("arm");
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

    public double calculateImuPower(int degrees) {


        final double P_VALUE_FOR_TURNING_IMU = 0.002;

        yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double proportionalPowerForImu;

        yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


        double angleError = degrees - yaw;

        proportionalPowerForImu = P_VALUE_FOR_TURNING_IMU * angleError;

        //telemetry.addData("yaw", yaw);

        return proportionalPowerForImu;

    } //IMU

    public void setMotorPower(double lFront, double rFront, double lBack, double rBack) {
        this.fLeft.setPower(lFront);
        this.fRight.setPower(rFront);
        this.bLeft.setPower(lBack);
        this.bRight.setPower(rBack);
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

    //the desired heading must be relative to last imu reset
    //-180 < desired heading <= 180
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
            sleep(100);
        }
    }

    public void autoForward(double targetDistanceInMM) throws InterruptedException {


        final double P_VALUE = 0.006;

        //301 = circumference mm
        //537.7, ticks per motor revolution
        //1.4, gear ratio
        //converting mm to ticks
        final double MM_TO_TICKS = (537.7 / 1.4) / 301.59;

        double targetPos = targetDistanceInMM * MM_TO_TICKS + fLeft.getCurrentPosition();
        double error = targetPos - fLeft.getCurrentPosition();

        final double PROPORTIONAL_POWER = P_VALUE * error;


        long lastCheckMillis = System.currentTimeMillis();
        long millis = System.currentTimeMillis();

        double oldTick = fLeft.getCurrentPosition();

        boolean isStopped = false;

        while (!(error <= 10 && isStopped) && opMode.opModeIsActive()) {

            millis = System.currentTimeMillis();


            error = targetPos - fLeft.getCurrentPosition();

            fLeft.setPower(PROPORTIONAL_POWER);
            bLeft.setPower(PROPORTIONAL_POWER);
            fRight.setPower(PROPORTIONAL_POWER);
            bRight.setPower(PROPORTIONAL_POWER);


            if (millis > lastCheckMillis + 500) {
                lastCheckMillis = millis;
                double newTicks = fLeft.getCurrentPosition();

                if (oldTick == newTicks) {
                    isStopped = true;
                }

                oldTick = newTicks;


            }

        }

        fLeft.setPower(0);
        bLeft.setPower(0);
        fRight.setPower(0);
        bRight.setPower(0);

    }//Auto Forward but Better

    public double convertMMToTicksForMecanum(double targetDistanceInMM) {

        //301 = circumference mm
        //537.7, ticks per motor revolution
        //converting mm to ticks
        final double MM_TO_TICKS = (537.7 / 301.59) * 1.2;

        return targetDistanceInMM * MM_TO_TICKS;
    }

    public double getRemainingTicksForDrivetrainMecanum(double targetDistanceInMM) {
        double targetDistanceInTicks = convertMMToTicksForMecanum(targetDistanceInMM);
        return targetDistanceInTicks - fLeft.getCurrentPosition();
    }

    //TODO document for notebook

    public boolean checkReachedDistanceForMecanum(double targetDistanceInMM) {

        boolean done = false;
        double remainingDistance = Math.abs(getRemainingTicksForDrivetrainMecanum(targetDistanceInMM));


        //telemetry.addData("remaining distance in ticks for mecananamasm", remainingDistance);

        if (remainingDistance < 30 && -yaw < 5 && -yaw > -5) {
            done = true;
        }
        return done;
    }

    public double[] calculateMecanumPower(double targetDistanceInMM) {
        final double P_VALUE_FOR_MECANUM = 0.002;

        if (checkReachedDistanceForMecanum(targetDistanceInMM)) {
            return new double[]{0, 0, 0, 0};
        }

        double remainingDistance = getRemainingTicksForDrivetrainMecanum(targetDistanceInMM);
        double proportionalPower = P_VALUE_FOR_MECANUM * remainingDistance;

        double scaleImu = 0; //8.15;

        return scalePowers(new double[]{
                proportionalPower + calculateImuPower(0) * scaleImu,
                -proportionalPower - calculateImuPower(0) * scaleImu,
                -proportionalPower + calculateImuPower(0) * scaleImu,
                proportionalPower - calculateImuPower(0) * scaleImu
        });
    }

    public void mecanumBlocking(double inches, boolean right, double maxPower) {

        double ERROR_TOLERANCE = 10;
        double power;
        double targetTick;
        final double KP_MECANUM = 0.002;
        final double minPower = 0.15;
        final double IN_TO_TICK = 56.3;

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
                power += minPower;
            } else if (power < 0 && power > -1 * minPower) {
                power -= minPower;
            }

            //cap power
            power = Range.clip(power, -1 * maxPower, maxPower);

            setMotorPower(power, -1 * power, -1 * power, power);

            error = targetTick - fLeft.getCurrentPosition();
        }
        setMotorPower(0, 0, 0, 0);
        sleep(100);
    }

    public void straightBlocking(double inches, boolean forward, double maxPower) {

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
        sleep(100);
    }

    /**
     * Detects the T-marker and move this marker.
     *
     * @return
     */

    public void detectMarkerPositionRed() {

        //detect marker position
        MarkerDetectorRed.MARKER_POSITION position = markerProcessorRed.getPosition();

        while (position == MarkerDetectorRed.MARKER_POSITION.UNDETECTED) {
            Log.d("vision", "undetected marker, keep looking" + visionPortal.getCameraState());
            position = markerProcessorRed.getPosition();
        }

        //print position
        Log.d("vision", "detected position: " + position);

        //save marker position, apriltag position
        setMarkerPosRed(position);
        setWantedAprTagIdRed(position, true);

    }

    public void detectMarkerPositionBlue() {

        //detect marker position
        MarkerDetectorBlue.MARKER_POSITION position = markerProcessorBlue.getPosition();

        while (position == MarkerDetectorBlue.MARKER_POSITION.UNDETECTED) {
            Log.d("vision", "undetected marker, keep looking" + visionPortal.getCameraState());
            position = markerProcessorBlue.getPosition();
        }

        //print position
        Log.d("vision", "detected position: " + position);

        //save marker position, apriltag position
        setMarkerPosBlue(position);
        setWantedAprTagIdBlue(position, true);

    }

    public void shortRedMoveToBoard() {
        Log.d("vision", "moveToMarker: Pos " + markerPosRed);
        Log.d("vision", "moveToMarker: Tag " + wantedAprTagId);
        if (markerPosRed == MarkerDetectorRed.MARKER_POSITION.RIGHT) {
            straightBlocking(19, false, 0.5);
            setHeading(-45, 0.25);
            straightBlocking(5, false, 0.7);
            setHeading(-45, 0.25);
            straightBlocking(5, true, 0.7);
            setHeading(-90, 0.25);
            straightBlocking(24, false, 0.7);
            setHeading(-90, 0.25);
            mecanumBlocking(9, true, 0.25);
        } else if (markerPosRed == MarkerDetectorRed.MARKER_POSITION.LEFT) {
            straightBlocking(19, false, 0.5);
            setHeading(45, 0.25);
            straightBlocking(5, false, 0.7);
            setHeading(45, 0.25);
            straightBlocking(5, true, 0.7);
            setHeading(0, 0.25);
            straightBlocking(9, false, 0.7);
            setHeading(-90, 0.25);
            straightBlocking(24, false, 0.7);
            setHeading(-90, 0.25);
        } else { //center, default
            Log.d("vision", "moveToMarker: center or default");
            straightBlocking(20, false, 0.5);
            setHeading(0, 0.7);
            mecanumBlocking(4, true, 0.25);
            setHeading(0, 0.7);
            straightBlocking(12, false, 0.7);
            setHeading(0, 0.7);
            straightBlocking(5, true, 0.7);
            setHeading(-90, 0.25);
            straightBlocking(24, false, 0.7);
            setHeading(-90, 0.25);
        }
    }

    public void longBlueMoveToBoard() {
        Log.d("vision", "moveToMarker: Pos " + markerPosBlue);
        Log.d("vision", "moveToMarker: Tag " + wantedAprTagId);
        while (opMode.opModeIsActive()) {
            if (markerPosBlue == MarkerDetectorBlue.MARKER_POSITION.RIGHT) {
                straightBlocking(19, false, 0.5);
                setHeading(-45, 0.25);
                straightBlocking(6, false, 0.7);
                setHeading(-45, 0.25);
                straightBlocking(12, true, 0.7);
                setHeading(0, 0.7);
                straightBlocking(39, false, 0.7);
                setHeading(-90, 0.7);
                straightBlocking(77, false, 0.7);
                setHeading(-90, 0.75);
                mecanumBlocking(38, false, 0.7);
                setHeading(-90, 0.7);
                break;
            } else if (markerPosBlue == MarkerDetectorBlue.MARKER_POSITION.LEFT) {
                straightBlocking(19, false, 0.5);
                setHeading(45, 0.25);
                straightBlocking(5, false, 0.7);
                setHeading(45, 0.25);
                straightBlocking(6, true, 0.7);
                setHeading(0, 0.7);
                straightBlocking(33, false, 0.7);
                setHeading(-90, 0.7);
                straightBlocking(72, false, 0.7);
                setHeading(-179.5, 0.75);
                straightBlocking(24, false, 0.7);
                setHeading(-90, 0.7);
                break;
            } else { //center, default
                Log.d("vision", "moveToMarker: center or default");
                straightBlocking(20, false, 0.5);
                setHeading(0, 0.7);
                mecanumBlocking(8, true, 0.5);
                setHeading(0, 0.7);
                straightBlocking(12, false, 0.5);
                setHeading(0, 0.7);
                straightBlocking(5, true, 0.7);
                setHeading(0, 0.7);
                mecanumBlocking(8, true, 0.25);
                setHeading(0, 0.7);
                straightBlocking(25, false, 0.7);
                setHeading(-90, 0.7);
                straightBlocking(84, false, 0.7);
                mecanumBlocking(24, false,  0.5);
                setHeading(-90, 0.7);
                break;
            }
        }
    }

    //contains most apriltag logic
    public void alignToBoard() {

        boolean tagVisible = false;
        boolean aligned = false;
        List<AprilTagDetection> myAprilTagDetections;
        double distanceToBoard = 0;

        while (opMode.opModeIsActive() && !aligned) { //while robot isnt aligned to tag

            //get detections
            myAprilTagDetections = aprilTagProcessor.getDetections();

            //process detection list
            for (AprilTagDetection detection : myAprilTagDetections) {

                if (detection.metadata != null) { //if there's an apriltag

                    if (detection.id == wantedAprTagId) { //check if its desired
                        Log.d("vision", "runOpMode: tag visible - this one " + wantedAprTagId);
                        tagVisible = true; //if it is, the desired tag is visible

                        //alignment based on bearing
                        if (detection.ftcPose.bearing > 1) {
                            Log.d("vision", "runOpMode: bearing > 1, move left");
                            mecanumBlocking(1, true, 0.75);
                            sleep(100);
                            aligned = false;
                        } else if (detection.ftcPose.bearing < -1) {
                            Log.d("vision", "runOpMode: bearing < -1, move right");
                            mecanumBlocking(1, false, 0.75);
                            sleep(100);
                            aligned = false;
                        } else {
                            Log.d("vision", "runOpMode: aligned");
                            distanceToBoard = detection.ftcPose.range - 5;
                            tagVisible = true;
                            aligned = true;
                        }
                    }
                }

                if (!tagVisible) { //if tag isnt visible
                    Log.d("vision", "runOpMode: tag not visible, move back");
                    straightBlocking(1, true, 0.6); //V IMP: should NOT go back into partner's space
                    tagVisible = false;
                    aligned = false;
                }
            }
        }

        Log.d("vision", "alignToBoard: broken out of !aligned while loop");
        if (aligned) {
            Log.d("vision", "alignToBoard: aligned is true");
            straightBlocking(distanceToBoard, false, 0.75);

            if (redAlliance) {
                setHeading(-90, 0.75);
            } else {
                setHeading(90, 0.75);
            }

        } else {
            Log.d("vision", "alignToBoard: not aligned, still broke out of !aligned. should not be happening");
        }
    }

    public void moveToBoard () {
        Log.d("vision", "moveToMarker: Pos " + markerPosRed);
        Log.d("vision", "moveToMarker: Tag " + wantedAprTagId);

        int polarity;
        if (redAlliance) {
            polarity = -1;
        } else {
            polarity = 1;
        }

        while (opMode.opModeIsActive()) {
            setServoPosBlocking(holderClamp, 0.5);
            if (markerPosRed == MarkerDetectorRed.MARKER_POSITION.RIGHT) { //RIGHT
                straightBlocking(20, false, 0.25); //forward
                setHeading(45 * polarity, 0.25); //turn right
                straightBlocking(8, false, 0.7); //forward
                setHeading(45 * polarity, 0.25);
                straightBlocking(12, true, 0.7); //dropoff, back
                setHeading(0, 0.7); //turn back
                straightBlocking(34, false, 0.7); //forward past spike
                setHeading(90 * polarity, 0.7); //turn right toward truss
                straightBlocking(77, false, 0.7); //forward under truss
                setHeading(90 * polarity, 0.75);
                mecanumBlocking(34, false, 0.7); //mecanum directly in front of board
                setHeading(90 * polarity, 0.7);
                break;
            } else if (markerPosRed == MarkerDetectorRed.MARKER_POSITION.LEFT) { //LEFT
                straightBlocking(18, false, 0.25); //forward
                setHeading(-45 * polarity, 0.25); //turn
                straightBlocking(7, false, 0.7); //forward
                setHeading(-45 * polarity, 0.25);
                straightBlocking(8, true, 0.7); //dropoff, backward
                setHeading(0, 0.7); //turn
                mecanumBlocking(1, false, 0.5); //mecanum right
                setHeading(0, 0.7);
                straightBlocking(33, false, 0.7); //forward to truss
                setHeading(90 * polarity, 0.7); //turn
                straightBlocking(72, false, 0.7); //forward to red line
                setHeading(90 * polarity, 0.7);
                mecanumBlocking(24, false, 0.7); //mecanum directly in front of board
                setHeading(90 * polarity, 0.7);
                break;
            } else { //center, default
                Log.d("vision", "moveToMarker: center or default");
                mecanumBlocking(4, true, 0.5); //go left
                setHeading(0, 0.6);
                straightBlocking(28.5, false, 0.5); //go forward
                setHeading(0, 0.6);
                straightBlocking(6, true, 0.7); //dropoff & move back
                setHeading(0, 0.6);
                mecanumBlocking(12, true, 0.25); //move left
                setHeading(0, 0.7);
                straightBlocking(24, false, 0.7); //go forward & around marker
                setHeading(90 * polarity, 0.7); //turn
                straightBlocking(84, false, 0.7); //forward to red line
                setHeading(90 * polarity, 0.7);
                mecanumBlocking(29, false,  0.5); //mecanum directly in front of board
                setHeading(90 * polarity, 0.7);
                break;
            }
        }
    }
}