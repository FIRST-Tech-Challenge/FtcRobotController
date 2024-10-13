package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.stormbots.MiniPID;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class GyroBot extends FourWheelDriveBot {

    BNO055IMU imu;
    public double startAngle = 0;
    double power = 0.1;
    private double headingOffset = 0;

    public void driveFront(double power){

    }
    public GyroBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;

//        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingTag = "IMU";

        imu = hwMap.get(BNO055IMU.class, "imu 1");
        imu.initialize(parameters);

        if (!isAuto) {
            headingOffset = GyroHolder.getHeading();
        }
    }

    public void driveByHandFieldCentric(double left_stick_x1, double left_stick_y1, double right_stick_x1, boolean button1, double left_stick_x2, double left_stick_y2, double right_stick_x2, boolean button2) {

        double drive  = - left_stick_y1 - left_stick_y2;
        double strafe = left_stick_x1 + left_stick_x2;
        double twist  = right_stick_x1 + right_stick_x2;

        timeSinceToggle5 = System.currentTimeMillis() - lastToggleDone5;
        if (button1 || button2) {
//            if (isSlow) {
//                driveMultiplier = 0.85;
//                isSlow = false;
//                opMode.telemetry.addData("SLOW", driveMultiplier);
//                lastToggleDone5 = System.currentTimeMillis();
//                //RobotLog.d("robot not slow");
//            } else if (!isSlow) {
//                driveMultiplier = 0.95;
//                isSlow = true;
//                opMode.telemetry.addData("FAST", driveMultiplier);
//                lastToggleDone5 = System.currentTimeMillis();
//                //RobotLog.d("robot slow");
//            }
//            opMode.telemetry.update();
            //RobotLog.d("stick button pressed");
            driveMultiplier = 1;
            //opMode.telemetry.addData("FAST", driveMultiplier);
        } else {
            driveMultiplier = 0.7;
            //opMode.telemetry.addData("SLOW", driveMultiplier);
        }

        double angle = -getDeltaAngle() + 180;
//        opMode.telemetry.addData("raw angle: ", getAngle());
//        opMode.telemetry.addData("math: ", angle);

        double drive2 = Math.min(1.0, strafe*Math.sin(Math.toRadians(angle)) + drive*Math.cos(Math.toRadians(angle)));
        double strafe2 = Math.min(1.0, strafe*Math.cos(Math.toRadians(angle)) - drive*Math.sin(Math.toRadians(angle)));

        opMode.telemetry.update();
        driveByVector(drive2, strafe2, twist, driveMultiplier);
    }

    public void resetAngle(boolean button) {
        if (button) {
            startAngle = getAngle();
        }
    }

    public double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

//        opMode.telemetry.addData("Angle", angles.firstAngle - startAngle);
//        opMode.telemetry.addData("Raw angle", angles.firstAngle);
//        opMode.telemetry.addData("Start angle", startAngle);
//        opMode.telemetry.update();

        return angles.firstAngle + headingOffset;
    }

    public double getDeltaAngle() {

        double angle = getAngle();
        //RobotLog.d(String.format("Delta Angle : %.3f from %.3f", deltaAngle, angle));

        return angle - startAngle;
    }

    public void goBacktoStartAngle() {

        int direction;
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double delta = getDeltaAngle();

        while (Math.abs(delta) > 2 && this.opMode.opModeIsActive()) {
            onLoop(50, "goBacktoStartAngle");
            if (delta < 0) {
                // turn clockwize
                direction = -1;
            } else {
                // turn CC wize
                direction = 1;
            }
            leftFront.setPower(power * direction);
            rightFront.setPower(-power * direction * highRPMToLowRPM);
            leftRear.setPower(power * direction * highRPMToLowRPM);
            rightRear.setPower(-power * direction);

            delta = getDeltaAngle();

        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

    }

    public void goBacktoStartAnglePID() {

        MiniPID pid = new MiniPID(0.008, 0.00005, 0.0045);
        pid.setOutputLimits(0.5);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double angle;
        angle = getAngle();
        double power = pid.getOutput(angle, startAngle);
        while (Math.abs(power) > 0.1 && this.opMode.opModeIsActive()) {
            onLoop(50,"goBacktoStartAnglePID");
            RobotLog.d(String.format("PID(source: %.3f, target: %.3f) = power: %.3f", angle, startAngle, power));
            leftFront.setPower(-power);
            rightFront.setPower(power * highRPMToLowRPM);
            leftRear.setPower(-power * highRPMToLowRPM);
            rightRear.setPower(power);
            angle = getAngle();
            power = pid.getOutput(angle, startAngle);
        };
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void goToAngle(double targetAngle, double power) {
        int direction;
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double delta = getAngle() - targetAngle;

        while (Math.abs(delta) > 1 && this.opMode.opModeIsActive()) {
            onLoop(50, "goToAngle");
            if (delta < 0) {
                // turn clockwize
                direction = -1;
            } else {
                // turn CC wize
                direction = 1;
            }
            leftFront.setPower(power * direction);
            rightFront.setPower(-power * direction * highRPMToLowRPM);
            leftRear.setPower(power * direction * highRPMToLowRPM);
            rightRear.setPower(-power * direction);

            delta = getAngle() - targetAngle;

        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

    }

    public void goToAnglePID(double targetAngle) {

        MiniPID pid = new MiniPID(0.008, 0.00005, 0.0045);
        pid.setOutputLimits(1);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double angle;
        angle = getAngle();
        double power = pid.getOutput(angle, targetAngle);
        RobotLog.d(String.format("PID(source: %.3f, target: %.3f) = power: %.3f", angle, targetAngle, power));
        while (Math.abs(power) > 0.1 && this.opMode.opModeIsActive()) {
            onLoop(50, "goToAnglePID");
            RobotLog.d(String.format("PID(source: %.3f, target: %.3f) = power: %.3f", angle, targetAngle, power));
            leftFront.setPower(-power);
            rightFront.setPower(power * highRPMToLowRPM);
            leftRear.setPower(-power * highRPMToLowRPM);
            rightRear.setPower(power);
            angle = getAngle();
            power = pid.getOutput(angle, targetAngle);
        };
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void lineUpShot(boolean button, int towerGoalY, double currentY, double hypotenuse) {
        if (button) {
            double adjacent = Math.abs(towerGoalY - currentY);

//            RobotLog.d(String.format("adjacent: %.2f", adjacent));
//            RobotLog.d(String.format("hypotenuse: %.2f", hypotenuse));
//            RobotLog.d(String.format("target angle: %.2f", Math.acos((adjacent / hypotenuse))));
            double targetAngle = (Math.toDegrees(Math.acos(adjacent / hypotenuse)) * -1) + 5;

            goToAnglePID(targetAngle);

        }
    }

    public void fullRotate(double power) {
        int direction;
        int targetAngle = 5;
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(power);
        rightFront.setPower(-power * highRPMToLowRPM);
        leftRear.setPower(power * highRPMToLowRPM);
        rightRear.setPower(-power);
        onLoop(8000, "180 degrees");

        double delta = getAngle() - targetAngle;

        while (Math.abs(delta) > 1 && this.opMode.opModeIsActive()) {
            onLoop(30, "goBacktoStartAngle");
            if (delta < 0) {
                // turn clockwize
                direction = -1;
            } else {
                // turn CC wize
                direction = 1;
            }
            leftFront.setPower(power * direction);
            rightFront.setPower(-power * direction * highRPMToLowRPM);
            leftRear.setPower(power * direction * highRPMToLowRPM);
            rightRear.setPower(-power * direction);

            delta = getAngle() - targetAngle;

        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

    }

    public void driveStraightByGyro(int direction, double distance, double maxPower, boolean useCurrentAngle, double currentAngle, boolean brake) {
        if (direction != DIRECTION_FORWARD && direction != DIRECTION_BACKWARD && direction != DIRECTION_LEFT && direction != DIRECTION_RIGHT){
            String msg = String.format("Unaccepted direction value (%d) for driveStraightByGyro()", direction);
            print(msg);
            return;
        }
        double originalAngle;
        if (useCurrentAngle) {
            originalAngle = currentAngle;
        } else {
            originalAngle = startAngle;
        }

        // distance (in mm) = revolution * pi * diameter (100 mm)
        int distanceTicks = (int) (distance * CENTIMETER_TO_DRIVING_MOTOR_CONVERSION_RATE);
        int startingPosition = leftFront.getCurrentPosition();
        MiniPID pid = new MiniPID(0.02, 0, 0);
        pid.setOutputLimits(maxPower);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double angle;
        angle = getAngle();
        double adjustPower = pid.getOutput(angle, originalAngle);
        int currentPosition = leftFront.getCurrentPosition();
        while (Math.abs(currentPosition - startingPosition) < distanceTicks && this.opMode.opModeIsActive()) {
            onLoop(30, "gyro drive 1");
            RobotLog.d(String.format("driveStraightByGyro : Current: %d - Start:%d > 10 => power: %.3f  +/- PID(source: %.3f, target: %.3f) = adjustPower: %.3f", currentPosition, startingPosition, maxPower, angle, originalAngle, adjustPower));
            switch (direction){
                case DIRECTION_FORWARD:
                    leftFront.setPower(maxPower - adjustPower);
                    rightFront.setPower(maxPower + adjustPower * highRPMToLowRPM);
                    leftRear.setPower(maxPower - adjustPower * highRPMToLowRPM);
                    rightRear.setPower(maxPower + adjustPower);
                    break;
                case DIRECTION_BACKWARD:
                    leftFront.setPower(- maxPower - adjustPower);
                    rightFront.setPower(- maxPower + adjustPower * highRPMToLowRPM);
                    leftRear.setPower(- maxPower - adjustPower * highRPMToLowRPM);
                    rightRear.setPower(- maxPower + adjustPower);
                    break;
                case DIRECTION_LEFT:
                    leftFront.setPower(- maxPower - adjustPower);
                    rightFront.setPower(+ maxPower + adjustPower * highRPMToLowRPM);
                    leftRear.setPower(+ maxPower - adjustPower * highRPMToLowRPM);
                    rightRear.setPower(- maxPower + adjustPower);
                    break;
                case DIRECTION_RIGHT:
                    leftFront.setPower(+ maxPower - adjustPower);
                    rightFront.setPower(- maxPower + adjustPower * highRPMToLowRPM);
                    leftRear.setPower(- maxPower - adjustPower * highRPMToLowRPM);
                    rightRear.setPower(+ maxPower + adjustPower);
                    break;
            }
            onLoop(30, "gyro drive 2");
            angle = getAngle();
            adjustPower = pid.getOutput(angle, originalAngle);
            currentPosition = leftFront.getCurrentPosition();
        }
        if (brake) {
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        //sleep(100, "after gyro wait");
    }

//    public void driveByGyroWithEncodersVertical(int direction, double distance, double maxPower, boolean useCurrentAngle, boolean decelerate) {
//        driveByGyroWithEncodersVertical(direction, distance, maxPower, useCurrentAngle, decelerate, 500);
//    }
//
//    public void driveByGyroWithEncodersVertical(int direction, double distance, double maxPower, boolean useCurrentAngle, boolean decelerate, int wait) {
//        if (direction != DIRECTION_FORWARD && direction != DIRECTION_BACKWARD && direction != DIRECTION_LEFT && direction != DIRECTION_RIGHT){
//            String msg = String.format("Unaccepted direction value (%d) for driveStraightByGyro()", direction);
//            print(msg);
//            return;
//        }
//        double originalAngle;
//        if (useCurrentAngle) {
//            originalAngle = getAngle();
//        } else {
//            originalAngle = 0;
//        }
//
//        // distance (in mm) = revolution * pi * diameter (100 mm)
//        int distanceTicks = (int) distance;
//        int startingPosition = leftFront.getCurrentPosition();
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
//        int currentPosition = leftFront.getCurrentPosition();
//        while (Math.abs(currentPosition - startingPosition) < distanceTicks && this.opMode.opModeIsActive()) {
//            onLoop(60, "gyro drive 1");
//            RobotLog.d(String.format("driveStraightByGyro : Current: %d - Start:%d > 10 => power: %.3f  +/- PID(source: %.3f, target: %.3f) = adjustPower: %.3f", currentPosition, startingPosition, maxPower, angle, originalAngle, adjustPower));
//            if (Math.abs(currentPosition - startingPosition) > distanceTicks - (40000 * increment) && decelerate) {
//                powerMultiplier = powerMultiplier * increment;
//                increment -= 0.1;
//                RobotLog.d(String.format("Current Position: %d Powermultiplier: %.1f Increment: %.1f", currentPosition, powerMultiplier, increment));
//            }
//            switch (direction){
//                case DIRECTION_FORWARD:
//                    leftFront.setPower((maxPower - adjustPower) * powerMultiplier);
//                    rightFront.setPower((maxPower + adjustPower) * powerMultiplier * highRPMToLowRPM);
//                    leftRear.setPower((maxPower - adjustPower) * powerMultiplier * highRPMToLowRPM);
//                    rightRear.setPower((maxPower + adjustPower) * powerMultiplier);
//                    break;
//                case DIRECTION_BACKWARD:
//                    leftFront.setPower((- maxPower - adjustPower) * powerMultiplier);
//                    rightFront.setPower((- maxPower + adjustPower) * powerMultiplier * highRPMToLowRPM);
//                    leftRear.setPower((- maxPower - adjustPower) * powerMultiplier * highRPMToLowRPM);
//                    rightRear.setPower((- maxPower + adjustPower) * powerMultiplier);
//                    break;
//            }
//            //onLoop(30, "gyro drive 2");
//            angle = getAngle();
//            adjustPower = pid.getOutput(angle, originalAngle);
//            currentPosition = leftFront.getCurrentPosition();
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
//    public void driveWithEncodersVertical(int direction, double distance, double maxPower, boolean decelerate) {
//        if (direction != DIRECTION_FORWARD && direction != DIRECTION_BACKWARD && direction != DIRECTION_LEFT && direction != DIRECTION_RIGHT){
//            String msg = String.format("Unaccepted direction value (%d) for driveStraightByGyro()", direction);
//            print(msg);
//            return;
//        }
//
//        // distance (in mm) = revolution * pi * diameter (100 mm)
//        int distanceTicks = (int) distance;
//        int startingPosition = leftFront.getCurrentPosition();
//
//        double powerMultiplier = 1;
//        double increment = 0.8;
//
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        int currentPosition = leftFront.getCurrentPosition();
//        while (Math.abs(currentPosition - startingPosition) < distanceTicks && this.opMode.opModeIsActive()) {
//            onLoop(60, "gyro drive 1");
//            if (Math.abs(currentPosition - startingPosition) > distanceTicks - (40000 * increment) && decelerate) {
//                powerMultiplier = powerMultiplier * increment;
//                increment -= 0.1;
//                RobotLog.d(String.format("Current Position: %d Powermultiplier: %.1f Increment: %.1f", currentPosition, powerMultiplier, increment));
//            }
//            switch (direction){
//                case DIRECTION_FORWARD:
//                    leftFront.setPower((maxPower) * powerMultiplier);
//                    rightFront.setPower((maxPower) * powerMultiplier * highRPMToLowRPM);
//                    leftRear.setPower((maxPower) * powerMultiplier * highRPMToLowRPM);
//                    rightRear.setPower((maxPower) * powerMultiplier);
//                    break;
//                case DIRECTION_BACKWARD:
//                    leftFront.setPower((- maxPower) * powerMultiplier);
//                    rightFront.setPower((- maxPower) * powerMultiplier * highRPMToLowRPM);
//                    leftRear.setPower((- maxPower) * powerMultiplier * highRPMToLowRPM);
//                    rightRear.setPower((- maxPower) * powerMultiplier);
//                    break;
//            }
//            currentPosition = leftFront.getCurrentPosition();
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

    protected void onTick() {
//        opMode.telemetry.addData("saved angle:", headingOffset);
//        opMode.telemetry.addData("angle:", getAngle());
//        opMode.telemetry.addData("delta angle:", getDeltaAngle());
        super.onTick();
    }
}
