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
    Servo flipper;


    //arm motor is not used
    DcMotor armMotor;
    IMU imu;

    ElapsedTime elapsedTime;

    double yaw;

    double prevError = 0;
    double prevTime = 0;


    OpenCvWebcam webcam;
    MarkerDetector.MARKER_POSITION markerPos;

    int wantedAprTagId;

    private MarkerDetector detector;
    private MarkerProcessor markerProcessor;
    private AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;


    //CONSTRUCTOR
    public Robot(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
        this.telemetry = telemetry;
        AprilTagProcessor aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        setUpDrivetrainMotors();
        setUpImu();
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }

    public void setMarkerPos (MarkerDetector.MARKER_POSITION position) {
        markerPos = position;
    }

    public MarkerDetector.MARKER_POSITION getMarkerPos () {
        return markerPos;
    }

    //change boolean to enum later
    public void setWantedAprTagId (MarkerDetector.MARKER_POSITION position, boolean redAlliance) {
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

    public int getWantedAprTagId () {
        return wantedAprTagId;
    }

//vision processing setup
    public void initVisionProcessing() {

        // Initializing marker and apriltag processors and setting them with visionportal
        markerProcessor = new MarkerProcessor(telemetry);
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

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public MarkerProcessor getMarkerProcessor() {
        return markerProcessor;
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }

//    public double getAprilY(int idNumber) {
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//
//        double yValue = 0;
//
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                yValue = detection.ftcPose.y;
//            }
//
//        }
//
//        return yValue;
//    }

    public double getAprilTagXPos(int idNumber) {

//        visionPortal.setProcessorEnabled(aprilTag, true);
//        visionPortal.setProcessorEnabled(markerProcessor, false);

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

    public double getAprilZ(int idNumber) {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        double yawAprilTag = 0;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                yawAprilTag = detection.ftcPose.yaw;
            }

        }
        return yawAprilTag;
    }

    public double getAprilTagRange(int idNumber) {
//        visionPortal.setProcessorEnabled(markerProcessor, false);
//        m.setProcessorEnabled(aprilTag, true);

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
        double moveDistanceInches ;

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
            straightBlocking(Math.abs(moveDistanceInches), !(moveDistanceInches > 0));
        } else if (aprilTagRange == 0){

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

    public void aprilTagFnaggling(int idNumber, double rangeFromAprilTagMM, double xAxisFromAprilTag) {
        boolean isDoneWithAprilTagX = false;
        boolean isDoneWithAprilTagRange = false;
        while (opMode.opModeIsActive() && !isDoneWithAprilTagX) {
            isDoneWithAprilTagX = moveRelativeToAprilTagX(xAxisFromAprilTag, idNumber); //method returns boolean
            telemetry.addData("x value: ", isDoneWithAprilTagX);
            telemetry.update();
        }

        telemetry.addLine("done with aprtag x");

        while (opMode.opModeIsActive() && !isDoneWithAprilTagRange) {
            isDoneWithAprilTagRange = moveRelativeToAprilTagRange(rangeFromAprilTagMM, idNumber); //method returns boolean
            telemetry.addData("range: ", isDoneWithAprilTagRange);
            telemetry.update();
        }
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

    public void setUpArmMotor() {
        armMotor = hardwareMap.dcMotor.get("arm");

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*
        public double calculateArmPower(int targetAngleInDegrees) {

            final double P_VALUE = 0.006;

            double remainingDistance = getRemainingTicksForArm(targetAngleInDegrees);
            if (remainingDistance < 10){
                return 0;
            }
            double proportionalPower = P_VALUE*remainingDistance;
            return proportionalPower;

        }
    */
    /*
    public void setArmPower(double armPower) {
        arm.setPower(armPower);
    }
    */
    /*
        public boolean checkArmPos(int targetAngleInDegrees) {

            boolean doneArm = false;
            double remainingDistance = getRemainingTicksForArm(targetAngleInDegrees);

            if (remainingDistance < 10) {
                setArmPower(0);
                doneArm = true;
            }
            return doneArm;
        }
    */
    /*
    public double getRemainingTicksForArm(double targetDistanceInDegrees) {
        double targetDistanceInTicks = convertDegreesToTicks(targetDistanceInDegrees);
        double remainingDistance = targetDistanceInTicks - arm.getCurrentPosition();

        return remainingDistance;
    }
    */

    public double convertDegreesToTicks(double targetDistanceInDegrees) {

        //537.7, ticks per motor revolution
        //1.4, gear ratio
        //converting mm to ticks
        final double Degrees_TO_TICKS = 537.7 / 15;

        return targetDistanceInDegrees * Degrees_TO_TICKS;

    }//Arm

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

    public double convertMMToTicks(double targetDistanceInMM) {

        //301 = circumference mm
        //537.7, ticks per motor revolution
        //converting mm to ticks
        final double MM_TO_TICKS = (537.7 / 301.59);

        return targetDistanceInMM * MM_TO_TICKS;

    }

    public boolean checkReachedDistance(double targetDistanceInMM) {

        boolean done = false;
        double remainingDistance = Math.abs(getRemainingTicksForDrivetrain(targetDistanceInMM));


        //telemetry.addData("remainig distance for ffowrard and backeward", remainingDistance);

        if (remainingDistance < 30 && -yaw < 5 && -yaw > -5) {
            done = true;
        }
        return done;
    }

    public double getRemainingTicksForDrivetrain(double targetDistanceInMM) {
        double targetDistanceInTicks = convertMMToTicks(targetDistanceInMM);
        return targetDistanceInTicks - fLeft.getCurrentPosition();
    }

    public void resetEncoder() {
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double[] calculateDrivetrainPower(double targetDistanceInMM) {
        final double P_VALUE = 0.001;

        if (checkReachedDistance(targetDistanceInMM)) {
            return new double[]{0, 0, 0, 0};
        }

        double remainingDistance = getRemainingTicksForDrivetrain(targetDistanceInMM);

        double proportionalPower = P_VALUE * remainingDistance;
        double scaleImu = 0;
        return scalePowers(new double[]{
                proportionalPower + calculateImuPower(0) * scaleImu,
                proportionalPower - calculateImuPower(0) * scaleImu,
                proportionalPower + calculateImuPower(0) * scaleImu,
                proportionalPower - calculateImuPower(0) * scaleImu
        });
    }

    public double[] calculateDrivetrainPowerWithTurning(double targetDistanceInMM, int angleToTurn) {
        final double P_VALUE = 0.0015;

        if (checkReachedDistance(targetDistanceInMM)) {
            return new double[]{0, 0, 0, 0};
        }

        double remainingDistance = getRemainingTicksForDrivetrain(targetDistanceInMM);

        double proportionalPower = P_VALUE * remainingDistance;
        double scaleImu = 1.5;
        return scalePowers(new double[]{
                proportionalPower + calculateImuPower(angleToTurn) * scaleImu,
                proportionalPower - calculateImuPower(angleToTurn) * scaleImu,
                proportionalPower + calculateImuPower(angleToTurn) * scaleImu,
                proportionalPower - calculateImuPower(angleToTurn) * scaleImu
        });
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
    public void setHeading(double targetAbsDegrees) {
    /*
        assert(targetAbsDegrees > 180);
        assert(targetAbsDegrees <= -180);

     */
        if (targetAbsDegrees == 180) {
            setHeading(179.5);
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
                power = Range.clip(power, -1, 1);

                setMotorPower(-1 * power, power, -1 * power, power);

                prevError = error;
                prevTime = currentTime;
            }
            setMotorPower(0, 0, 0, 0);
        }
    }

    public boolean isHeadingWithinError(double targetHeading) {
        double ERROR_TOLERANCE = 0.5;
        double currentHeading = getCurrentHeading();
        double deltaHeading = Math.abs(targetHeading - currentHeading);
        //telemetry.addLine("within error, return true");
        //telemetry.addLine("not within error, return false");
        return deltaHeading <= ERROR_TOLERANCE;
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

    public void autoMecanuming(double targetMecanumDistance) {
        final double P_VALUE = 0.004;

        //301 = circumference mm
        //537.7, ticks per motor revolution
        //1.4, gear ratio
        //converting mm to ticks

        final double MM_TO_TICKS = 537.7 / 301.59;

        double targetPos = targetMecanumDistance * MM_TO_TICKS * (1.225);
        double error = targetPos - fLeft.getCurrentPosition();

        final double PROPORTIONAL_POWER = P_VALUE * error;


        long lastCheckMillis = System.currentTimeMillis();
        long millis = System.currentTimeMillis();

        double oldTick = fLeft.getCurrentPosition();

        boolean isStopped = false;


        millis = System.currentTimeMillis();


        error = targetPos - fLeft.getCurrentPosition();

        double imuPower = calculateImuPower(0);

        setMotorPower(
                PROPORTIONAL_POWER + imuPower,
                -PROPORTIONAL_POWER + imuPower,
                -PROPORTIONAL_POWER + imuPower,
                PROPORTIONAL_POWER + imuPower
        );

        if (millis > lastCheckMillis + 500) {
            lastCheckMillis = millis;
            double newTicks = fLeft.getCurrentPosition();

            if (oldTick == newTicks) {
                isStopped = true;
            }

            oldTick = newTicks;

        }
    }

    public void mecanumBlocking(double inches, boolean right) {

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
            power = Range.clip(power, -1, 1);

            setMotorPower(power, -1 * power, -1 * power, power);

            error = targetTick - fLeft.getCurrentPosition();
        }
        setMotorPower(0, 0, 0, 0);
    }

    public void straightBlocking(double inches, boolean forward) {

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
                power = Range.clip(power, -0.7, 0.7);

                setMotorPower(power, power, power, power);

                currentTick = fLeft.getCurrentPosition();
                prevTime = currentTime;
                prevError = error;
            }
            setMotorPower(0, 0, 0, 0);
    }

    /**
     * Detects the T-marker and move this marker.
     *
     * @return
     */

    public void detectMarkerPosition () {

        //detect marker position
        MarkerDetector.MARKER_POSITION position = markerProcessor.getPosition();

        while (position == MarkerDetector.MARKER_POSITION.UNDETECTED) {
            Log.d("vision", "undetected marker, keep looking");
            position = markerProcessor.getPosition();
        }

        //print position
        Log.d("vision", "detected position: " + position);

        //save marker position, apriltag position
        setMarkerPos(position);
        setWantedAprTagId(position, true);

    }

    //contains most opencv logic
    public void moveToMarker() {
        Log.d("vision", "moveToMarker: Pos " + markerPos);
        Log.d("vision", "moveToMarker: Tag " + wantedAprTagId);
        if (markerPos == MarkerDetector.MARKER_POSITION.CENTER) {
            straightBlocking(20, false);
            setHeading(15);
            sleep(100);
            straightBlocking(6, false);
            sleep(100);
            straightBlocking(6, true);
            setHeading(0);
            sleep(100);
            straightBlocking(19, true);
        } else if (markerPos == MarkerDetector.MARKER_POSITION.LEFT) {
            straightBlocking(18, false);
            setHeading(30);
            sleep(100);
            straightBlocking(4, false);
            sleep(100);
            straightBlocking(4, true);
            setHeading(0);
            sleep(100);
            straightBlocking(17, true);
        } else { //right or other
            straightBlocking(14, false);
            setHeading(-45);
            sleep(100);
            straightBlocking(11, false);
            sleep(100);
            straightBlocking(11, true);
            setHeading(0);
            sleep(100);
            straightBlocking(13, true);
        }
    }

    //contains most apriltag logic
    public void moveToBoard () {

        boolean tagVisible = false;
        boolean aligned = false;
        List<AprilTagDetection> myAprilTagDetections;

        while (opMode.opModeIsActive() && !aligned) {

            //get detections
            myAprilTagDetections = aprilTagProcessor.getDetections();

            //process detection list
            for (AprilTagDetection detection : myAprilTagDetections) {

                if (detection.metadata != null) {

                    if (detection.id == wantedAprTagId) {
                        Log.d("vision", "runOpMode: tag visible - this one " + wantedAprTagId);
                        tagVisible = true;

                        if (detection.ftcPose.bearing > 1) {
                            Log.d("vision", "runOpMode: bearing > 1, move left");
                            mecanumBlocking(1, true);
                        } else if (detection.ftcPose.bearing < -1) {
                            Log.d("vision", "runOpMode: bearing < -1, move right");
                            mecanumBlocking(1, false);
                        } else {
                            Log.d("vision", "runOpMode: aligned");
                            aligned = true;
                        }

                        sleep(100);
                    }
                }

                if (!tagVisible) {
                    Log.d("vision", "runOpMode: tag not visible, move back");
                    straightBlocking(1, true);
                    sleep(100);
                }
            }
        }

        if (aligned) {

            myAprilTagDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : myAprilTagDetections) {
                if (detection.metadata != null) {
                    if (detection.id == wantedAprTagId) {
                        Log.d("vision", "runOpMode: bearing is " + detection.ftcPose.bearing);
                        double distanceToBoard = detection.ftcPose.range - 5;
                        Log.d("vision", "runOpMode: distance to travel is " + distanceToBoard);
                        straightBlocking(distanceToBoard, false);
                        sleep(100);
                        break;
                    }
                }
            }
        }
    }

}

