package org.firstinspires.ftc.teamcode.bots;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.stormbots.MiniPID;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

public class OdometryBot extends GyroBot {

    public DcMotor horizontal = null;
    public DcMotor verticalLeft = null;
    public DcMotor verticalRight = null;
    public Servo odometryRaise = null;

    String verticalLeftEncoderName = "v1", verticalRightEncoderName = "v2", horizontalEncoderName = "h";

    public double xBlue = 0, yBlue = 0, xBlueChange = 0, yBlueChange = 0, thetaDEG = 0;
    double xRed = 0, yRed = 0, xRedChange = 0, yRedChange = 0;
    double hError = 0;

    protected double[] driveAccelerationCurve = new double[]{0.5, 0.6, 0.8, 0.9, 0.8, 0.9};

    double savedXBlue, savedYBlue, savedThetaDEG;
    public double savedStartAngle;

    final int vLDirection = 1;
    final int vRDirection = -1;
    final int hDirection = 1;
    final double diameter = 18971; // actually diameter
    final double hDiameter = 11936; //radius of horizontal encoder

    double vLOffset, vROffset, hOffset = 0;

    public double previousVL = 0, previousVR = 0, previousH = 0;
    double angleChange = 0;

    double drive;
    double strafe;
    double twist;
    double driveAngle;
    double thetaDifference;
    double distanceToTarget;
    long startTime;
    long elapsedTime = 0;
    public boolean isCoordinateDriving = false;

    ElapsedTime robotLogTimer = new ElapsedTime();

    OutputStreamWriter odometryWriter;

    public OdometryBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        initDriveHardwareMap(ahwMap);
        context = hwMap.appContext;
        odometryRaise = hwMap.get(Servo.class, "odometryRaise");
        odometryRaise.setPosition(0.88);
        opMode.telemetry.addData("Status", "Init Complete");
        opMode.telemetry.update();
        robotLogTimer.reset();
    }

    private void initDriveHardwareMap(HardwareMap ahwMap){

        horizontal = ahwMap.dcMotor.get(horizontalEncoderName);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        verticalLeft = ahwMap.dcMotor.get(verticalLeftEncoderName);
//        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        verticalRight = ahwMap.dcMotor.get(verticalRightEncoderName);
//        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        opMode.telemetry.addData("Status", "Hardware Map Init Complete");
        opMode.telemetry.update();
    }

    Context context;

    protected void onTick(){
//        RobotLog.d(String.format("Position, heading: %.2f, %.2f, %.2f", xBlue, yBlue, thetaDEG));
//        RobotLog.d(String.format("v1: %d v2: %d h: %d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), horizontal.getCurrentPosition()));
//        opMode.telemetry.addData("X:", xBlue);
//        opMode.telemetry.addData("Y:", yBlue);
//        opMode.telemetry.addData("Theta:", thetaDEG);
//        opMode.telemetry.addData("v1", leftFront.getCurrentPosition());
//        opMode.telemetry.addData("v2", rightFront.getCurrentPosition());
        outputEncoders();
        super.onTick();
        //calculateCaseThree(leftFront.getCurrentPosition() - vLOffset, rightFront.getCurrentPosition() - vROffset, horizontal.getCurrentPosition() - hOffset, thetaDEG);
    }

    public void outputEncoders() {
//        opMode.telemetry.addData("h", horizontal.getCurrentPosition());
//        opMode.telemetry.update();
        RobotLog.d(String.format("h: %d time: %.0f", horizontal.getCurrentPosition(), robotLogTimer.milliseconds()));
    }

    public void driveAgainstWallWithEncodersVertical(int direction, CameraBot.autoSide side, int distance, int tolerance, int wait) {
        if (direction != DIRECTION_FORWARD && direction != DIRECTION_BACKWARD){
            String msg = String.format("Unaccepted direction value (%d) for driveStraightByGyro()", direction);
            print(msg);
            return;
        }

        int startingPosition = horizontal.getCurrentPosition();
        double powerMultiplier = 1;

        MiniPID drivePID = new MiniPID(0.00004, 0.000001, 0.000007);
        drivePID.setOutputLimits(1);
//        MiniPID gyroPID = new MiniPID(0.06, 0.005, 0.03);
//        gyroPID.setOutputLimits(1);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int targDistance;

        switch (direction) {
            default:
                targDistance = -distance;
            case DIRECTION_FORWARD:
                targDistance = -distance;
                break;
            case DIRECTION_BACKWARD:
                targDistance = distance;
                break;
        }

        int currentPosition = horizontal.getCurrentPosition() - startingPosition;
        double power = drivePID.getOutput(currentPosition, targDistance);
        double distanceToTarget = Math.abs(Math.abs(currentPosition) - Math.abs(targDistance));
//
//        double targetAngle = 0;
//        double currentAngle = getAngle();
//        double adjustPower = gyroPID.getOutput(currentAngle, targetAngle);

        int count = 0;
        RobotLog.d(String.format("current: %d dist: %d power: %.3f", currentPosition, targDistance, power));
        while ((distanceToTarget > tolerance && Math.abs(power) > 0.09) && opMode.opModeIsActive()) {
            if (count < 4) {
                powerMultiplier = driveAccelerationCurve[count];
            } else {
                powerMultiplier = 1;
            }
            RobotLog.d(String.format("current: %d dist: %d power: %.3f", currentPosition, targDistance, power));
            switch (direction) {
                case DIRECTION_FORWARD:
                    switch (side) {
                        case RED:
                            driveByVector(1, 0.45, 0.35, -power * powerMultiplier);
                            break;
                        case BLUE:
                            driveByVector(1, -0.45, -0.35, -power * powerMultiplier);
                            break;
                    }
                    break;
                case DIRECTION_BACKWARD:
                    switch (side) {
                        case RED:
                            driveByVector(1, -0.65, 0.45, -power * powerMultiplier);
                            break;
                        case BLUE:
                            driveByVector(1, 0.65, -0.45, -power * powerMultiplier);
                            break;
                    }
                    break;
            }
            onLoop(10, "wall drive 2");

            currentPosition = horizontal.getCurrentPosition() - startingPosition;
            power = drivePID.getOutput(currentPosition, targDistance);
            distanceToTarget = Math.abs(Math.abs(currentPosition) - Math.abs(targDistance));

            count++;
        }
        RobotLog.d("wall drive stop");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        sleep(wait, "after gyro wait");
    }

    public void driveByGyroWithEncodersVertical(int direction, int distance, boolean useCurrentAngle, int tolerance, int wait) {
        if (direction != DIRECTION_FORWARD && direction != DIRECTION_BACKWARD){
            String msg = String.format("Unaccepted direction value (%d) for driveStraightByGyro()", direction);
            print(msg);
            return;
        }
        double originalAngle;
        if (useCurrentAngle) {
            originalAngle = getAngle();
        } else {
            originalAngle = 0;
        }

        // distance (in mm) = revolution * pi * diameter (100 mm)
        int startingPosition = horizontal.getCurrentPosition();
        double powerMultiplier = 1;
        double increment = 0.8;

        MiniPID gyroPID = new MiniPID(0.03, 0, 0);
        MiniPID drivePID = new MiniPID(0.06, 0, 0);
        gyroPID.setOutputLimits(1);
        drivePID.setOutputLimits(1);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double angle;
        angle = getAngle();
        double adjustPower = gyroPID.getOutput(angle, originalAngle);
        int currentPosition = Math.abs(horizontal.getCurrentPosition() - startingPosition);
        int distanceToTarget = Math.abs(currentPosition - distance);
        double power = drivePID.getOutput((int) (distanceToTarget/2500), 0);
        int count = 0;
        RobotLog.d(String.format("driveStraightByGyro : Current: %d - Start:%d > DistanceToTarget:%d => power: %.3f  +/- PID(source: %.3f, target: %.3f) = adjustPower: %.3f", currentPosition, startingPosition, distanceToTarget, power, angle, originalAngle, adjustPower));
        while ((distanceToTarget > tolerance && Math.abs(currentPosition) < distance)&& opMode.opModeIsActive()) {
            if (count < 4) {
                powerMultiplier = driveAccelerationCurve[count];
            } else {
                powerMultiplier = 1;
            }
            RobotLog.d(String.format("driveStraightByGyro : Current: %d - Start:%d > DistanceToTarget:%d => power: %.3f  +/- PID(source: %.3f, target: %.3f) = adjustPower: %.3f", currentPosition, startingPosition, distanceToTarget, power, angle, originalAngle, adjustPower));
            switch (direction){
                case DIRECTION_FORWARD:
                    leftFront.setPower((- power - adjustPower) * powerMultiplier);
                    rightFront.setPower((- power + adjustPower) * powerMultiplier * highRPMToLowRPM);
                    leftRear.setPower((- power - adjustPower) * powerMultiplier * highRPMToLowRPM);
                    rightRear.setPower((- power + adjustPower) * powerMultiplier);
                    break;
                case DIRECTION_BACKWARD:
                    leftFront.setPower((power - adjustPower) * powerMultiplier);
                    rightFront.setPower((power + adjustPower) * powerMultiplier * highRPMToLowRPM);
                    leftRear.setPower((power - adjustPower) * powerMultiplier * highRPMToLowRPM);
                    rightRear.setPower((power + adjustPower) * powerMultiplier);
                    break;
            }
            onLoop(10, "gyro drive 2");
            angle = getAngle();
            adjustPower = gyroPID.getOutput(angle, originalAngle);
            currentPosition = Math.abs(horizontal.getCurrentPosition() - startingPosition);
            distanceToTarget = Math.abs(currentPosition - distance);
            power = drivePID.getOutput((int) (distanceToTarget/2000), 0);
            count++;
        }
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        sleep(wait, "after gyro wait");
    }

//    public double[] calculateCaseThree(double vL, double vR, double h, double angleDEG) {
//        vL = vL * vLDirection;
//        vR = vR * vRDirection;
//        h = h * hDirection;
//
//        double lC = vL - previousVL;
//        double rC = vR - previousVR;
//
//        angleChange = ((lC - rC) / (Math.PI * diameter * 2) * 360);
//
//        //angleDEG = angleDEG + angleChange;
//        //thetaDEG = angleDEG;
//        angleDEG = getDeltaAngle();
//        thetaDEG = getDeltaAngle();
//
//        hError = (angleChange / 360) * (Math.PI * hDiameter);
//
//        double hC = h - previousH;
//
//        xRedChange = hC + hError;
//        yRedChange = (lC + rC)/2;
//
//        xBlueChange = Math.cos(Math.toRadians(angleDEG - 90)) * xRedChange + Math.cos(Math.toRadians(angleDEG)) * yRedChange;
//        yBlueChange = Math.sin(Math.toRadians(angleDEG)) * yRedChange + Math.sin(Math.toRadians(angleDEG - 90)) * xRedChange;
//
//        xBlue = xBlue + yBlueChange;
//        yBlue = yBlue + xBlueChange;
//
//        previousVL = vL;
//        previousVR = vR;
//        previousH = h;
//
//        double[] position = {xBlue, yBlue};
//
//        return position;
//    }

//    public void resetOdometry(boolean button) {
//
//        if (button) {
////            vLOffset = leftFront.getCurrentPosition();
////            vROffset = rightFront.getCurrentPosition();
////            hOffset = horizontal.getCurrentPosition() + 79000;
//
//            horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            previousVL = 0;
//            previousVR = 0;
//            previousH = 0;
//
//            xBlue = 79000;
//            yBlue = 0;
//
//            thetaDEG = 0;
//        }
//    }
//    public void driveByGyroWithEncodersHorizontal(int direction, double distance, double maxPower, boolean useCurrentAngle, boolean decelerate) {
//        driveByGyroWithEncodersHorizontal(direction, distance, maxPower, useCurrentAngle, decelerate, 500);
//    }

//    public void driveByGyroWithEncodersHorizontal(int direction, double distance, double maxPower, boolean useCurrentAngle, boolean decelerate, int wait) {
//        if (direction != DIRECTION_FORWARD && direction != DIRECTION_BACKWARD && direction != DIRECTION_LEFT && direction != DIRECTION_RIGHT){
//            String msg = String.format("Unaccepted direction value (%d) for driveStraightByGyro()", direction);
//            print(msg);
//            return;
//        }
//        double originalAngle;
//        if (useCurrentAngle) {
//            originalAngle = getAngle();
//        } else {
//            originalAngle = startAngle;
//        }
//
//        // distance (in mm) = revolution * pi * diameter (100 mm)
//        int distanceTicks = (int) distance;
//        int startingPosition = horizontal.getCurrentPosition();
//
//        double powerMultiplier = 1;
//        double increment = 0.8;
//
//        MiniPID pid = new MiniPID(0.03, 0, 0);
//        pid.setOutputLimits(maxPower);
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        double angle;
//        angle = getAngle();
//        double adjustPower = pid.getOutput(angle, originalAngle);
//        int currentPosition = horizontal.getCurrentPosition();
//        while (Math.abs(currentPosition - startingPosition) < distanceTicks && this.opMode.opModeIsActive()) {
//            onLoop(60, "gyro drive 1");
//            RobotLog.d(String.format("driveStraightByGyro : Current: %d - Start:%d > 10 => power: %.3f  +/- PID(source: %.3f, target: %.3f) = adjustPower: %.3f", currentPosition, startingPosition, maxPower, angle, originalAngle, adjustPower));
//            if (Math.abs(currentPosition - startingPosition) > distanceTicks - (20000 * increment) && decelerate) {
//                powerMultiplier = powerMultiplier * increment;
//                increment -= 0.1;
//                RobotLog.d(String.format("Current Position: %d Powermultiplier: %.1f Increment: %.1f", currentPosition, powerMultiplier, increment));
//            }
//            switch (direction){
//                case DIRECTION_LEFT:
//                    leftFront.setPower((- maxPower - adjustPower) * powerMultiplier);
//                    rightFront.setPower((+ maxPower + adjustPower) * powerMultiplier * highRPMToLowRPM);
//                    leftRear.setPower((+ maxPower - adjustPower) * powerMultiplier * highRPMToLowRPM);
//                    rightRear.setPower((- maxPower + adjustPower) * powerMultiplier);
//                    break;
//                case DIRECTION_RIGHT:
//                    leftFront.setPower((+ maxPower - adjustPower) * powerMultiplier);
//                    rightFront.setPower((- maxPower + adjustPower) * powerMultiplier * highRPMToLowRPM);
//                    leftRear.setPower((- maxPower - adjustPower) * powerMultiplier * highRPMToLowRPM);
//                    rightRear.setPower((+ maxPower + adjustPower) * powerMultiplier);
//                    break;
//            }
//            //onLoop(30, "gyro drive 2");
//            angle = getAngle();
//            adjustPower = pid.getOutput(angle, originalAngle);
//            currentPosition = horizontal.getCurrentPosition();
//        }
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftFront.setPower(0);
//        rightFront.setPower(0);
//        leftRear.setPower(0);
//        rightRear.setPower(0);
//        sleep(wait, "after gyro wait");
//    }
//
//    public void driveWithEncodersHorizontal(int direction, double distance, double maxPower, boolean decelerate) {
//        if (direction != DIRECTION_FORWARD && direction != DIRECTION_BACKWARD && direction != DIRECTION_LEFT && direction != DIRECTION_RIGHT){
//            String msg = String.format("Unaccepted direction value (%d) for driveStraightByGyro()", direction);
//            print(msg);
//            return;
//        }
//
//        // distance (in mm) = revolution * pi * diameter (100 mm)
//        int distanceTicks = (int) distance;
//        int startingPosition = horizontal.getCurrentPosition();
//
//        double powerMultiplier = 1;
//        double increment = 0.8;
//
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        int currentPosition = horizontal.getCurrentPosition();
//        while (Math.abs(currentPosition - startingPosition) < distanceTicks && this.opMode.opModeIsActive()) {
//            onLoop(60, "gyro drive 1");
//            if (Math.abs(currentPosition - startingPosition) > distanceTicks - (40000 * increment) && decelerate) {
//                powerMultiplier = powerMultiplier * increment;
//                increment -= 0.1;
//                RobotLog.d(String.format("Current Position: %d Powermultiplier: %.1f Increment: %.1f", currentPosition, powerMultiplier, increment));
//            }
//            switch (direction){
//                case DIRECTION_LEFT:
//                    leftFront.setPower((- maxPower) * powerMultiplier);
//                    rightFront.setPower((+ maxPower) * powerMultiplier * highRPMToLowRPM);
//                    leftRear.setPower((+ maxPower) * powerMultiplier * highRPMToLowRPM);
//                    rightRear.setPower((- maxPower) * powerMultiplier);
//                    break;
//                case DIRECTION_RIGHT:
//                    leftFront.setPower((+ maxPower) * powerMultiplier);
//                    rightFront.setPower((- maxPower) * powerMultiplier * highRPMToLowRPM);
//                    leftRear.setPower((- maxPower) * powerMultiplier * highRPMToLowRPM);
//                    rightRear.setPower((+ maxPower) * powerMultiplier);
//                    break;
//            }
//            currentPosition = horizontal.getCurrentPosition();
//        }
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftFront.setPower(0);
//        rightFront.setPower(0);
//        leftRear.setPower(0);
//        rightRear.setPower(0);
//        sleep(500, "after driving wait");
//    }
//
//    public void driveToCoordinate(double xTarget, double yTarget, double targetTheta, int tolerance, double magnitude) {
//        if (xBlue > xTarget) {
//            distanceToTarget = - Math.sqrt(Math.pow(xBlue - xTarget, 2) + Math.pow(yBlue - yTarget, 2));
//        } else {
//            distanceToTarget = Math.sqrt(Math.pow(xBlue - xTarget, 2) + Math.pow(yBlue - yTarget, 2));
//        }
//        RobotLog.d(String.format("BlueX: %f BlueY: %f Theta: %f", xBlue, yBlue, thetaDEG));
//        startTime = System.currentTimeMillis();
//        if (isCoordinateDriving){
//            driveToCoordinateUpdate(xTarget, yTarget, targetTheta, tolerance, magnitude);
//        }
////            driveToCoordinateUpdate(xTarget, yTarget, targetTheta, tolerance, magnitude);
//        if ((xTarget + tolerance > xBlue) && (xTarget - tolerance < xBlue) && (yTarget + tolerance > yBlue) && (yTarget - tolerance < yBlue) && Math.abs(thetaDifference) < 2) {
//            isCoordinateDriving = false;
//            driveByVector(0, 0, 0, 1);
//            RobotLog.d("TARGET REACHED");
//        } else {
//            isCoordinateDriving = true;
//        }
////            elapsedTime = System.currentTimeMillis() - startTime;
////            if (elapsedTime > 10000) {
////                break;
////            }
//    }

//    public void driveToCoordinateUpdate(double xTarget, double yTarget, double targetTheta, int tolerance, double magnitude) {
//        MiniPID drivePID = new MiniPID(0.05, 0, 0);//i: 0.006 d: 0.06
//        MiniPID twistPID = new MiniPID(0.025, 0.005, 0.03);
//        drivePID.setOutputLimits(1);
//        twistPID.setOutputLimits(1);
//        thetaDifference = targetTheta - thetaDEG;
//        twist = - twistPID.getOutput(thetaDEG, targetTheta);
//        double rawDriveAngle = Math.toDegrees(Math.atan2(xTarget - xBlue, yTarget - yBlue));
//        driveAngle = rawDriveAngle - thetaDEG;
//        magnitude = Math.min(1.0, Math.abs(drivePID.getOutput(distanceToTarget/5000, 0))*2);
//        if (Math.abs(distanceToTarget) < 10000) {
//            magnitude = Math.max(0.1, Math.min(1.0, Math.abs(drivePID.getOutput(distanceToTarget/5000, 0))));
//        }
//        if (xBlue > xTarget) {
//            distanceToTarget = - Math.sqrt(Math.pow(xBlue - xTarget, 2) + Math.pow(yBlue - yTarget, 2));
//        } else {
//            distanceToTarget = Math.sqrt(Math.pow(xBlue - xTarget, 2) + Math.pow(yBlue - yTarget, 2));
//        }
//        drive = -(Math.cos(Math.toRadians(driveAngle)) * magnitude);
//        strafe = Math.sin(Math.toRadians(driveAngle)) * magnitude;
//
//        driveByVector(drive, strafe, twist, 1);
//        RobotLog.d(String.format("BlueX: %f BlueY: %f Theta: %f Angle: %f Drive: %f Strafe: %f Twist: %f", xBlue, yBlue, thetaDEG, driveAngle, drive, strafe, twist));
//        RobotLog.d(String.format("Distance: %f Magnitude: %f", distanceToTarget, magnitude));
//    }

//    public void savePosition() {
////        try {
////            odometryWriter = new FileWriter("/sdcard/FIRST/odometry positions.txt", false);
////        } catch (IOException e) {
////            throw new RuntimeException("odometry file writer open failed: " + e.toString());
////        }
////        try {
////            RobotLog.d("odometryWriter.write");
////            odometryWriter.write(xBlue + "\n");
////            odometryWriter.write(yBlue + "\n");
////            odometryWriter.write(thetaDEG + "\n");
////            odometryWriter.write(getAngle() + "\n");
////        } catch (IOException e) {
////            throw new RuntimeException("odometry file writer write failed: " + e.toString());
////        }
////        try {
////            RobotLog.d("odometryWriter.close");
////            odometryWriter.close();
////        } catch (IOException e) {
////            throw new RuntimeException("odometry file writer close failed: " + e.toString());
////        }
//        try {
//            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput("odometry positions.txt", Context.MODE_PRIVATE));
//
//            // write each configuration parameter as a string on its own line
//            outputStreamWriter.write(xBlue + "\n");
//            outputStreamWriter.write(yBlue + "\n");
//            outputStreamWriter.write(thetaDEG + "\n");
//            outputStreamWriter.write(getAngle() + "\n");
//
//            outputStreamWriter.close();
//        }
//        catch (IOException e) {
//            opMode.telemetry.addData("Exception", "Configuration file write failed: " + e.toString());
//        }
//
//    }

    public void readPosition() {
        try {
            InputStream inputStream = context.openFileInput("odometry positions.txt");
            if ( inputStream != null ) {
                InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);

                xBlue = Double.parseDouble(bufferedReader.readLine());
                opMode.telemetry.addData("X:", xBlue);
                yBlue = Double.parseDouble(bufferedReader.readLine());
                opMode.telemetry.addData("Y:", yBlue);
                opMode.telemetry.update();
                RobotLog.d(String.format("odometry bodoo: %.2f, %.2f", xBlue, yBlue));
                thetaDEG = Double.parseDouble(bufferedReader.readLine());
                savedStartAngle = Double.parseDouble(bufferedReader.readLine());
                thetaDEG = savedStartAngle;

                inputStream.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
