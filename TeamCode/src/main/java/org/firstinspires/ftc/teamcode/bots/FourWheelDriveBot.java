// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor.

package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class FourWheelDriveBot extends BotBot{

    // Gobilda 435 rpm DC motor : Encoder Countable Events Per Revolution (Output Shaft) : 383.6 * 2 (2:1 bevel gear ratio)
    // Gobilda 312 rpm DC motor : Encoder Countable Events Per Revolution (Output Shaft) : 383.6 * 2 (1:1 bevel gear ratio)
    public static final double DRIVING_MOTOR_TICK_COUNT = 767;
    public static final double CENTIMETER_TO_DRIVING_MOTOR_CONVERSION_RATE = 66.666;
    public static final int DIRECTION_FORWARD = 1;
    public static final int DIRECTION_BACKWARD = 2;
    public static final int DIRECTION_LEFT = 3;
    public static final int DIRECTION_RIGHT = 4;
    public static final int DIRECTION_RQUARTER = 5;
    public static final int DIRECTION_LQUARTER = 6;

    public static final double highRPMToLowRPM = 1;//0.71724137931034482758620689655171

    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightRear = null;

    boolean isSlow = false;

    long timeSinceToggle5 = 0;
    long lastToggleDone5 = 0;
    double driveMultiplier = 0.5;

    //HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();
    private Orientation angles;
    private boolean arcadeMode = false;
    private double headingOffset = 0.0;

    public FourWheelDriveBot(LinearOpMode opMode)  {
        super(opMode);
    }

    // manual drive
    private double getRawHeading() {
        return angles.firstAngle;
    }
    public double getHeading() {
        return (getRawHeading() - headingOffset) % (2.0 * Math.PI);
    }
    public void resetHeading() {
        headingOffset = getRawHeading();
    }

    private static double maxAbs(double... xs) {
        double ret = Double.MIN_VALUE;
        for (double x : xs) {
            if (Math.abs(x) > ret) {
                ret = Math.abs(x);
            }
        }
        return ret;
    }

    //    public void driveByHand(double _lf, double _lr, double _rf, double _rr) {
    public void driveByHand(double left_stick_x1, double left_stick_y1, double right_stick_x1, boolean button1, double left_stick_x2, double left_stick_y2, double right_stick_x2, boolean button2) {


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
            driveMultiplier = 0.60;
            opMode.telemetry.addData("FAST", driveMultiplier);
        } else {
            driveMultiplier = 0.075;
            opMode.telemetry.addData("SLOW", driveMultiplier);
        }
        opMode.telemetry.update();
        driveByVector(drive, strafe, twist, driveMultiplier);
    }

    public void driveByHandConeCentric(double right_stick_x) {
        if (right_stick_x > 0) {
            driveByVector(0, 0.2, -0.2, 1);
        } else if (right_stick_x < 0) {
            driveByVector(0, -0.2, 0.2, 1);
        }
    }

    public void driveByVector(double drive, double strafe, double twist, double multiplier) {
        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }
        //RobotLog.d(String.format("multiplier: %f speeds 0: %f", multiplier, speeds[0]));
        // apply the calculated values to the motors.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setPower(speeds[0] * multiplier);
        rightFront.setPower(speeds[1] * multiplier * highRPMToLowRPM);
        leftRear.setPower(speeds[2] * multiplier * highRPMToLowRPM);
        rightRear.setPower(speeds[3] * multiplier);
    }

    public void readControllerValues(double left_stick_x, double left_stick_y, double right_stick_x) {
        opMode.telemetry.addData("Left Stick X", left_stick_x);
        opMode.telemetry.addData("Left Stick Y", left_stick_y);
        opMode.telemetry.addData("Right Stick X", right_stick_x);
        opMode.telemetry.update();
    }

    public void print(String message){
        String caption = "4WD";
        opMode.telemetry.addData(caption, message);
        opMode.telemetry.update();
    }

    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        leftRear = hwMap.get(DcMotorEx.class, "leftRear");
        rightRear = hwMap.get(DcMotorEx.class, "rightRear");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        print("Resetting Encoders");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to indicate successful Encoder reset
//        print(String.format("Starting at leftFront: %7d, rightFront:%7d, leftRear:%7d, rightRear:%7d",
//                leftFront.getCurrentPosition(),
//                rightFront.getCurrentPosition(),
//                leftRear.getCurrentPosition(),
//                rightRear.getCurrentPosition()));
    }

    protected void onTick(){
        //driveByHand(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_button);
        super.onTick();
    }

    public void testOneMotor(DcMotor motor, double speed, int direction){
        // reset the timeout time and start motion.
        runtime.reset();

        double timeoutS = 5.0;
        // make 3 turn
        int target = motor.getCurrentPosition() + (int)DRIVING_MOTOR_TICK_COUNT * 3 * direction;
        print(String.format("Start %s @ %7d", motor.getDeviceName(), motor.getCurrentPosition()));

        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);

        while (opMode.opModeIsActive() && (runtime.seconds() < timeoutS) && motor.isBusy()) {
            // Display it for the driver.
            print(String.format("Running %s to %7d: @ %7d", motor.getDeviceName(), target, motor.getCurrentPosition()));
        }
        // Stop all motion;
        motor.setPower(0);

        print(String.format("Completed! %s @ %7d", motor.getDeviceName(), motor.getCurrentPosition()));

        sleep(3000);
    }
    public void driveStraightByDistance(int direction, double distance){
        // default max power 0.5
        driveStraightByDistance(direction, distance, 0.5);
    }

    public void driveStraightByDistance(int direction, double distance, double maxPower){
        // distance (in cm) * conversion rate = distance (in ticks)
        int target = (int)(distance * CENTIMETER_TO_DRIVING_MOTOR_CONVERSION_RATE);
//        int startingPosition = leftFront.getCurrentPosition();
//        int realTarget;
        switch (direction){
            case DIRECTION_FORWARD:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() + target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() + target);
                break;
            case DIRECTION_BACKWARD:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() - target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() - target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() - target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() - target);
                break;
            case DIRECTION_LEFT:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() - target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() + target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() - target);
                break;
            case DIRECTION_RIGHT:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() - target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() - target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() + target);
                break;
            case DIRECTION_RQUARTER:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() - target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() + target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() - target);
                break;
            case DIRECTION_LQUARTER:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() - target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() - target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() + target);
                break;
            default:
                String msg = String.format("Unaccepted direction value (%d) for driveStraightByDistance()", direction);
                print(msg);
        }

        double power = maxPower;

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
        RobotLog.d(String.format("Set direction and power!"));
        RobotLog.d(String.format("Target: %d", target));

        while (opMode.opModeIsActive() && rightFront.isBusy()) {
            onLoop(50, "Driving straight by distance");
            RobotLog.d(String.format("rightFront current pos: %d, target pos: %d", rightFront.getCurrentPosition(), rightFront.getTargetPosition()));
        }
        RobotLog.d(String.format("Stopping all motion!"));
        // Stop all motion;
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        print(String.format("Arrive target : %7d @ leftFront: %7d, rightFront:%7d, leftRear:%7d, rightRear:%7d",
                target,
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftRear.getCurrentPosition(),
                rightRear.getCurrentPosition()));
    }

    public void driveStraightByTime(int direction, int time, double maxPower){
        long startTime = System.currentTimeMillis();
        long timeSince = System.currentTimeMillis() - startTime;

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        switch (direction){
            case DIRECTION_FORWARD:
                leftFront.setPower(maxPower);
                rightFront.setPower(maxPower);
                leftRear.setPower(maxPower);
                rightRear.setPower(maxPower);
                break;
            case DIRECTION_BACKWARD:
                leftFront.setPower(-maxPower);
                rightFront.setPower(-maxPower);
                leftRear.setPower(-maxPower);
                rightRear.setPower(-maxPower);
                break;
            case DIRECTION_LEFT:
                leftFront.setPower(-maxPower);
                rightFront.setPower(maxPower);
                leftRear.setPower(maxPower);
                rightRear.setPower(-maxPower);
                break;
            case DIRECTION_RIGHT:
                leftFront.setPower(maxPower);
                rightFront.setPower(-maxPower);
                leftRear.setPower(-maxPower);
                rightRear.setPower(maxPower);
                break;
            case DIRECTION_RQUARTER:
                leftFront.setPower(maxPower);
                rightFront.setPower(-maxPower);
                leftRear.setPower(maxPower);
                rightRear.setPower(-maxPower);
                break;
            case DIRECTION_LQUARTER:
                leftFront.setPower(-maxPower);
                rightFront.setPower(maxPower);
                leftRear.setPower(-maxPower);
                rightRear.setPower(maxPower);
                break;
            default:
        }

        while (opMode.opModeIsActive() && timeSince < time) {
            timeSince = System.currentTimeMillis() - startTime;
            onLoop(0, "Driving straight by distance");
        }
        RobotLog.d("Stopping all motion!");
        // Stop all motion;
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void driveCurveByDistance(int direction, double distance, double curvePower,double maxPower) {
        if (direction != DIRECTION_FORWARD && direction != DIRECTION_BACKWARD && direction != DIRECTION_LEFT && direction != DIRECTION_RIGHT){
            String msg = String.format("Unaccepted direction value (%d) for driveCurveByDistance()", direction);
            print(msg);
            return;
        }
        // distance (in cm) * conversion rate = distance (in ticks)
        int distanceTicks = (int)(distance * CENTIMETER_TO_DRIVING_MOTOR_CONVERSION_RATE);
        int currentPosition;
        int startingPosition;
        if (curvePower < 0) {
            startingPosition = leftFront.getCurrentPosition();
        } else {
            startingPosition = leftRear.getCurrentPosition();
        }
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (curvePower < 0) {
            currentPosition = leftFront.getCurrentPosition();
        } else {
            currentPosition = leftRear.getCurrentPosition();
        }
        while (opMode.opModeIsActive() && Math.abs(currentPosition - startingPosition) < distanceTicks) {
            RobotLog.d(String.format("driveCurveByDistance : Current: %d - Start:%d > 10 => power: %.3f , curvePower: %.3f", currentPosition, startingPosition, maxPower, curvePower));
            switch (direction){
                case DIRECTION_FORWARD:
                    leftFront.setPower(maxPower - curvePower);
                    rightFront.setPower(maxPower + curvePower);
                    leftRear.setPower(maxPower - curvePower);
                    rightRear.setPower(maxPower + curvePower);
                    break;
                case DIRECTION_BACKWARD:
                    leftFront.setPower(- maxPower - curvePower);
                    rightFront.setPower(- maxPower + curvePower);
                    leftRear.setPower(- maxPower - curvePower);
                    rightRear.setPower(- maxPower + curvePower);
                    break;
                case DIRECTION_LEFT:
                    leftFront.setPower(- maxPower - curvePower);
                    rightFront.setPower(+ maxPower + curvePower);
                    leftRear.setPower(+ maxPower - curvePower);
                    rightRear.setPower(- maxPower + curvePower);
                    break;
                case DIRECTION_RIGHT:
                    leftFront.setPower(+ maxPower - curvePower);
                    rightFront.setPower(- maxPower + curvePower);
                    leftRear.setPower(- maxPower - curvePower);
                    rightRear.setPower(+ maxPower + curvePower);
                    break;
            }
            sleep(50);
            if (curvePower > 0) {
                currentPosition = leftFront.getCurrentPosition();
            } else {
                currentPosition = leftRear.getCurrentPosition();
            }
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void driveByDistanceWithAcceleration(int direction, double distance, double maxPower, int accelerationSteps){
        // distance (in mm) = revolution * pi * diameter (100 mm)
        int target = (int)(distance / 3.1415 / 100 * DRIVING_MOTOR_TICK_COUNT);
        int startingPosition = leftFront.getCurrentPosition();
        double accelerationDelta = maxPower/accelerationSteps;
        int accelerationInterval = 100;
        int realTarget;
        switch (direction){
            case DIRECTION_FORWARD:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
                realTarget = leftFront.getCurrentPosition() + target;
                rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() + target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() + target);
                break;
            case DIRECTION_BACKWARD:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() - target);
                realTarget = leftFront.getCurrentPosition() - target;
                rightFront.setTargetPosition(rightFront.getCurrentPosition() - target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() - target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() - target);
                break;
            case DIRECTION_LEFT:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() - target);
                realTarget = leftFront.getCurrentPosition() - target;
                rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() + target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() - target);
                break;
            case DIRECTION_RIGHT:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
                realTarget = leftFront.getCurrentPosition() + target;
                rightFront.setTargetPosition(rightFront.getCurrentPosition() - target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() - target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() + target);
                break;
            default:
                realTarget = leftFront.getCurrentPosition() - target;
                String msg = String.format("Unaccepted direction value (%d) for driveStraightByDistance()", direction);
                print(msg);
        }

        double power = maxPower;

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setPower(accelerationDelta);
        rightFront.setPower(accelerationDelta);
        leftRear.setPower(accelerationDelta);
        rightRear.setPower(accelerationDelta);
        int step = 1;
        RobotLog.d("Let's go : Target: %d AccelerationDelta: %.2f CurrentPosition: %d Step: %d", realTarget, accelerationDelta, leftFront.getCurrentPosition(), step);
        while (opMode.opModeIsActive() && leftFront.isBusy()) {
            double distToDecelerate = Math.min(Math.abs(leftFront.getCurrentPosition() - startingPosition), accelerationSteps * accelerationInterval);
            RobotLog.d("In loop : CurrentPosition: %d Step: %d DistToDecelerate: %.2f", leftFront.getCurrentPosition(), step, distToDecelerate);

            if (Math.abs(leftFront.getCurrentPosition() - realTarget) > distToDecelerate &&
                    step < accelerationSteps &&
                    Math.abs(leftFront.getCurrentPosition() - startingPosition) > step * accelerationInterval) {
                RobotLog.d("Step up CurrentPosition: %d Step: %d DistToDecelerate: %.2f", leftFront.getCurrentPosition(), step, distToDecelerate);
                step++;
                leftFront.setPower(Math.max(accelerationDelta * step, maxPower));
                rightFront.setPower(Math.max(accelerationDelta * step, maxPower));
                leftRear.setPower(Math.max(accelerationDelta * step, maxPower));
                rightRear.setPower(Math.max(accelerationDelta * step, maxPower));
            }

            if (Math.abs(leftFront.getCurrentPosition() - realTarget) < distToDecelerate &&
                    step > 1 && Math.abs(leftFront.getCurrentPosition() - realTarget) < step * accelerationInterval) {
                RobotLog.d("Step down CurrentPosition: %d Step: %d DistToDecelerate: %.2f", leftFront.getCurrentPosition(), step, distToDecelerate);
                step = (int)Math.floor(Math.abs(leftFront.getCurrentPosition() - realTarget) / accelerationInterval);
                leftFront.setPower(Math.max(accelerationDelta * step, 0.1));
                rightFront.setPower(Math.max(accelerationDelta * step, 0.1));
                leftRear.setPower(Math.max(accelerationDelta * step, 0.1));
                rightRear.setPower(Math.max(accelerationDelta * step, 0.1));
            }
        }
        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        print(String.format("Arrive target : %7d @ leftFront: %7d, rightFront:%7d, leftRear:%7d, rightRear:%7d",
                realTarget,
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftRear.getCurrentPosition(),
                rightRear.getCurrentPosition()));
    }

}

