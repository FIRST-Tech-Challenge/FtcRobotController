package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.slf4j.Marker;

import java.util.List;

public class Robot {


    HardwareMap hardwareMap;
    Telemetry telemetry;
    LinearOpMode opMode;
    DcMotor fLeft;
    DcMotor fRight;
    DcMotor bLeft;
    DcMotor bRight;
    DcMotor arm;
    IMU imu;

    ElapsedTime elapsedTime;

    double yaw;

    double prevError = 0;
    double prevTime = 0;


    OpenCvWebcam webcam;
    MarkerDetector.MARKER_POSITION position;
    private MarkerDetector detector;
    private MarkerProcessor markerProcessor;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    int idNumber;


    //CONSTRUCTOR
    public Robot(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
        this.telemetry = telemetry;
        setUpDrivetrainMotors();
        setUpImu();
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }

    public void setUpVisionProcessing() {

//        aprilTag = new AprilTagProcessor.Builder().build();
        markerProcessor = new MarkerProcessor();

        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessors(markerProcessor)
            .build();
//        visionPortal.setProcessorEnabled(markerProcessor, false);
//        visionPortal.setProcessorEnabled(aprilTag, false);
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
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        Log.d("vision", "num of detections" + currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            Log.d("vision", "id detections" + detection.metadata.id);
            if (detection.metadata.id == idNumber) {
                xValue = detection.ftcPose.x;
                telemetry.addData("id detected", idNumber);
                break;
            }
            telemetry.addLine("not detected");
        }
        return -xValue;
    }

    public double getAprilZ(int idNumber) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

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

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        double range = 0;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata.id == idNumber) {
                range = detection.ftcPose.range;
                break;
            }

        }
        return -range;
    }

    public boolean moveRelativeToAprilTagX(double mmFromAprilTag, int idNumber) {
        double inchesFromAprilTag = mmFromAprilTag / 25.4;
        boolean done = false;

        double[] powerMoveNegative = calculateMecanumPower((inchesFromAprilTag - getAprilTagXPos(idNumber)));
        double[] powerMovePositive = calculateMecanumPower(-(inchesFromAprilTag - getAprilTagXPos(idNumber)));

        double[] stopPower = {0, 0, 0, 0};

        if (!(getAprilTagXPos(idNumber) > inchesFromAprilTag - 0.5 &&
                getAprilTagXPos(idNumber) < inchesFromAprilTag + 0.5)) {
            if (getAprilTagXPos(idNumber) < inchesFromAprilTag) {
                powerMovePositive = calculateMecanumPower(-25.4 * (inchesFromAprilTag - getAprilTagXPos(idNumber)));
                setMotorPower(powerMovePositive);
            } else {
                powerMoveNegative = calculateMecanumPower(-25.4 * (inchesFromAprilTag - getAprilTagXPos(idNumber)));
                setMotorPower(powerMoveNegative);
            }
        } else {
            setMotorPower(stopPower);
            done = true;
        }

        telemetry.addData("power", powerMovePositive);
        telemetry.addData("power 2", powerMoveNegative);
        telemetry.addData("x ", getAprilTagXPos(idNumber));
        telemetry.addData("left range", getAprilTagXPos(idNumber) > inchesFromAprilTag - 0.5);
        telemetry.addData("right range", getAprilTagXPos(idNumber) < inchesFromAprilTag + 0.5);

        return done;
    }

    public boolean moveRelativeToAprilTagRange(double mmFromAprilTag, int idNumber) {
        double inchesFromAprilTag = mmFromAprilTag / 25.4;
        boolean done = false;

        double[] powerMoveNegative = calculateDrivetrainPower((inchesFromAprilTag - getAprilTagRange(idNumber)));
        double[] powerMovePositive = calculateDrivetrainPower(-(inchesFromAprilTag - getAprilTagRange(idNumber)));

        double[] stopPower = {0, 0, 0, 0};

        if (!(getAprilTagRange(idNumber) > inchesFromAprilTag - 0.5 &&
                getAprilTagRange(idNumber) < inchesFromAprilTag + 0.5)) {
            if (getAprilTagRange(idNumber) < inchesFromAprilTag) {
                powerMovePositive = calculateDrivetrainPower(-25.4 * (inchesFromAprilTag - getAprilTagRange(idNumber)));
                setMotorPower(powerMovePositive);
            } else {
                powerMoveNegative = calculateDrivetrainPower(-25.4 * (inchesFromAprilTag - getAprilTagRange(idNumber)));
                setMotorPower(powerMoveNegative);
            }
        } else {
            setMotorPower(stopPower);
            done = true;
        }

        telemetry.addData("power", powerMovePositive);
        telemetry.addData("power 2", powerMoveNegative);
        telemetry.addData("range", getAprilTagRange(idNumber));

        return done;
    }

    public void setUpImu() {

        this.imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
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

    private double maxAbsValueDouble(double[] values) {

        double max = -Double.MIN_VALUE;


        for (double value : values) {
            if (Math.abs(value) > Math.abs(max)) {
                max = value;
            }

        }

        return Math.abs(max);
    }

    private double[] scalePowers(double[] powers) {
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
        arm = hardwareMap.dcMotor.get("arm");

        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        if (deltaHeading <= ERROR_TOLERANCE) {
            //telemetry.addLine("within error, return true");
            return true;
        } else {
            //telemetry.addLine("not within error, return false");
            return false;
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

        telemetry.addLine("current ticks: " + String.valueOf(currentTick));
        telemetry.addLine("target ticks: " + String.valueOf(targetTick));
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
     */
    public void detectAndMoveToMarker() {

        elapsedTime = new ElapsedTime();

        /*webcam.setPipeline(detector);
        webcam.openCameraDevice();*/
//        visionPortal.setProcessorEnabled(aprilTag, false);
 //       visionPortal.setProcessorEnabled(markerProcessor, true);
        position = detector.position;


        //if seeing nothing then make center
        if (position == null) {
            telemetry.addLine("im seeing nothing forcing to be center");
            Log.d("vision", "im not seeing anything forcing to be center");
            position = MarkerDetector.MARKER_POSITION.CENTER;
        }

        while (opMode.opModeIsActive()) {

            if (position == MarkerDetector.MARKER_POSITION.CENTER) {
                straightBlocking(20, false);
                setHeading(15);
                waitFor(0.1);
                straightBlocking(6, false);
                waitFor(1.5);
                straightBlocking(6, true);
                setHeading(0);
                waitFor(0.1);
                straightBlocking(19, true);
                break;
            } else if (position == MarkerDetector.MARKER_POSITION.LEFT) {
                straightBlocking(18, false);
                setHeading(30);
                waitFor(0.1);
                straightBlocking(3, false);
                waitFor(1.5);
                straightBlocking(3, true);
                setHeading(0);
                waitFor(0.1);
                straightBlocking(17, true);
                break;
            } else if (position == MarkerDetector.MARKER_POSITION.RIGHT) {
                straightBlocking(14, false);
                setHeading(-45);
                waitFor(0.1);
                straightBlocking(11, false);
                waitFor(1.5);
                straightBlocking(11, true);
                setHeading(0);
                waitFor(0.1);
                straightBlocking(12, true);
                break;
            } else {
                break;
            }

        }

    }

    public void waitFor(double seconds) {
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        while (opMode.opModeIsActive()) {

            if (elapsedTime.milliseconds() >= seconds*1000) {
                break;

            }

        }
    }
    public void moveToApril() {

        webcam.openCameraDevice();
       // webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

//        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        /*
        setup

        if pos is left
        look for 4
        if center, 5
        if right, 6

        xval should be zero
        if xval < 0, move right
        if xval > 0, move left
        if xval = 0, break
        else telemetry.addLine("xval doesn't exist"); telemetry.update;
        */

        //setup

        /*
        if (position == MarkerDetector.MARKER_POSITION.LEFT) {
            idNumber = 4;
        } else if (position == MarkerDetector.MARKER_POSITION.CENTER) {
            idNumber = 5;
        } else if (position == MarkerDetector.MARKER_POSITION.RIGHT) {
            idNumber = 6;
        } else {
            telemetry.addLine("idNumber not 4 5 or 6");
        }
        */

        idNumber = 4;

        double aprX = getAprilTagXPos(4);
        while (opMode.opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addLine(currentDetections.toString());
        }
        /*
        double ERROR_TOLERANCE = 10;
        double power;
        double wantedAprX = 7_0000000; //TODO: this needs to be zero
        final double KP_MECANUM = 0.002;
        final double minPower = 0.2;
        double inches = 0; //TODO: what are the units
        final double IN_TO_TICK = 56.3;
        */

        /*
        if (aprX > ERROR_TOLERANCE) {
            wantedAprX = aprX + inches * IN_TO_TICK;
        } else {
            wantedAprX = aprX - inches * IN_TO_TICK;
        }
        */

        // double error = wantedAprX - aprX;

        /*
        while (Math.abs(error) >= ERROR_TOLERANCE && opMode.opModeIsActive()) {
            power = KP_MECANUM * error;

            //min pwr
            if (power > 0 && power < minPower) {
                power += minPower;
            } else if (power < 0 && power > -1 * minPower) {
                power -= minPower;
            }

            //cap power
            power = Range.clip(power, -1, 1);

            setMotorPower(power, -1 * power, -1 * power, power);

            error = wantedAprX - aprX;
        }
        setMotorPower(0, 0, 0, 0);
    }

        if (aprX < 0) {
            //move right
        } else if (aprX > 0) {
            //break
        } else if (aprX == 0) {
            //move left
        } else {
            telemetry.addLine("april tag X value (aprX) doesn't exist");
        }

        double aprY = getAprilY(idNumber);
        double wantedAprY = 0;
        if (aprY < 0) {
        } else if (aprY > 0) {
        } else if (aprY == 0) {
        } else {
            telemetry.addLine("april tag X value (aprX) doesn't exist");
        }

        double aprZ = getAprilZ(idNumber);
        double wantedAprZ = 0;
        if (aprY < 0) {
        } else if (aprY > 0) {
        } else if (aprY == 0) {
        } else {
            telemetry.addLine("april tag X value (aprX) doesn't exist");
        }



        telemetry.update();

        elapsedTime = new ElapsedTime();

        detector = new MarkerDetector();
        webcam.setPipeline(detector);
        webcam.openCameraDevice();
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        while (opMode.opModeIsActive()) {
            position = detector.position;

            if (position == MarkerDetector.MARKER_POSITION.CENTER) {
                straightBlocking(20, true);
                setHeading(15);
                waitFor(0.1);
                straightBlocking(6, true);
                waitFor(1.5);
                straightBlocking(6, false);
                setHeading(0);
                waitFor(0.1);
                straightBlocking(19, false);
                break;
            } else if (position == MarkerDetector.MARKER_POSITION.LEFT) {
                straightBlocking(18, true);
                setHeading(30);
                waitFor(0.1);
                straightBlocking(3, true);
                waitFor(1.5);
                straightBlocking(3, false);
                setHeading(0);
                waitFor(0.1);
                straightBlocking(17, false);
                break;
            } else if (position == MarkerDetector.MARKER_POSITION.RIGHT) {
                straightBlocking(14, true);
                setHeading(-45);
                waitFor(0.1);
                straightBlocking(11, true);
                waitFor(1.5);
                straightBlocking(11, false);
                setHeading(0);
                waitFor(0.1);
                straightBlocking(12, false);
                break;
            }
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();
        */
    }

}

