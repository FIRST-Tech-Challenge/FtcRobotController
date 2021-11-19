package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.arthur;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Mecanum", group="Linear OpMode")
@Disabled
public class Mecanum_TeleOp extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private final int TICKS_PER_ROTATION = 537;
    private final double WHEEL_RADIUS = 0.025; // in meters
    private final double LENGTH_LIFTER = 0.95; // in meters
//    private final double WHEEL_DISTANCE = 0.30; // in meters
    private final double WHEEL_DIAMETER = WHEEL_RADIUS * 2 * Math.PI; // in meters

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private DcMotor lifter;

    private double speedRate;
    private GamepadState gamepad;

    public static final double SPEED_RATE_INTERVAL = 0.1;

    public Action decreaseSpeedRate = () -> decreaseSpeed(SPEED_RATE_INTERVAL);

    public Action increaseSpeedRate = () -> increaseSpeed(SPEED_RATE_INTERVAL);

    private boolean floating = true;

    public Action toggleZeroPowerBehavior = () -> {
        if (floating) {
            setFloatingBehavior();
        } else {
            setBrakeBehavior();
        }
    };

    public Action liftUp = () -> {
        int ticks = lifter.getCurrentPosition();
        telemetry.addData("ticks", ticks);
        double revs = (double)ticks / TICKS_PER_ROTATION;
        double distance = revs * WHEEL_DIAMETER;
        telemetry.addData("distance", distance);
        if (distance < LENGTH_LIFTER) {
            lifter.setPower(0.5);
        } else {
            lifter.setPower(0);
        }
    };

    public Action liftDown = () -> {
        int ticks = lifter.getCurrentPosition();
        telemetry.addData("ticks", ticks);
        double revs = (double)ticks / TICKS_PER_ROTATION;
        double distance = revs * WHEEL_DIAMETER;
        telemetry.addData("distance", distance);
        if (distance > 0) {
            lifter.setPower(-0.5);
        } else {
            lifter.setPower(0);
        }
    };

    private void setFloatingBehavior() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("FLOATING");
        floating = false;
    }

    private void setBrakeBehavior() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("BRAKE");
        floating = true;
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        while (opModeIsActive()) {
            execute();
        }

        finish();
    }

    public void execute() {
        runtime.reset();

        gamepad.onTriggerOnce("left", decreaseSpeedRate);
        gamepad.onTriggerOnce("right", increaseSpeedRate);

        gamepad.onButtonOnce("x", toggleZeroPowerBehavior);

        gamepad.onButtonOnce("y", liftUp);
        gamepad.onButtonOnce("a", liftDown);

        double leftFrontPower = gamepad.calcLeftFrontPower(speedRate);
        double rightFrontPower = gamepad.calcRightFrontPower(speedRate);
        double leftBackPower = gamepad.calcLeftBackPower(speedRate);
        double rightBackPower = gamepad.calcRightBackPower(speedRate);

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        printData();
        telemetry.update();
    }

    public void modifySpeed(double s) {
        if (s < 0) {
            decreaseSpeed(Math.abs(s));
        } else {
            increaseSpeed(s);
        }
    }

    private void decreaseSpeed(double n) {
        double decreased = speedRate - n;
        if (decreased >= 0) {
            speedRate = decreased;
        }
    }

    private void increaseSpeed(double n) {
        double increased = speedRate + n;
        if (increased <= 1) {
            speedRate = increased;
        }
    }

    // ODOMETRY STUFF
//    private void updateTicks() {
//        updateLeftTicks();
//        updateRightTicks();
//        updateCenterTicks();
//    }

//    private void updateLeftTicks() {
//        lEncoderPos = leftEncoder.getCurrentPosition();
//    }
//
//    private double getLeftTicks() {
//        return leftEncoder.getCurrentPosition() - lEncoderPos;
//    }
//
//    private void updateRightTicks() {
//        rEncoderPos = rightEncoder.getCurrentPosition();
//    }
//
//    private double getRightTicks() {
//        return rightEncoder.getCurrentPosition() - rEncoderPos;
//    }
//
//    private void updateCenterTicks() {
//        cEncoderPos = centerEncoder.getCurrentPosition();
//    }
//
//    private double getCenterTicks() {
//        return centerEncoder.getCurrentPosition() - cEncoderPos;
//    }
//
//    private void updatePosition() {
//        double dLeft = (getLeftTicks() / TICKS_PER_ROTATION) * Math.PI * 2 * WHEEL_RADIUS;
//        double dRight = (getRightTicks() / TICKS_PER_ROTATION) * Math.PI * 2 * WHEEL_RADIUS;
//        double dCenter = (getCenterTicks() / TICKS_PER_ROTATION) * Math.PI * 2 * WHEEL_RADIUS;
//
//        x += (dLeft + dRight) / 2.0 * Math.cos(theta);
//        y += (dLeft + dRight) / 2.0 * Math.sin(theta);
//        theta += (dLeft - dRight) / WHEEL_DISTANCE;
//
//        updateTicks();
//    }

    private void printData() {
        telemetry.addData("speed rate", "%f%%", speedRate*100);
        telemetry.addData("LF power", leftFront.getPower());
        telemetry.addData("RF power", rightFront.getPower());
        telemetry.addData("LB power", leftBack.getPower());
        telemetry.addData("RB power", rightBack.getPower());
    }

    private static final double DEFAULT_SPEED_RATE = 0.5;

    private void initialize() {
        gamepad = new GamepadState(gamepad1);
        leftFront = hardwareMap.get(DcMotor.class, "LF");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack = hardwareMap.get(DcMotor.class, "LB");
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        lifter = hardwareMap.get(DcMotor.class, "Slide");
        lifter.setDirection(DcMotor.Direction.FORWARD);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Odometry wheels
//        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftTracking");
//        leftEncoder.setDirection(DcMotorEx.Direction.FORWARD);
//        leftEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightTracking");
//        rightEncoder.setDirection(DcMotorEx.Direction.FORWARD);
//        rightEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        centerEncoder = hardwareMap.get(DcMotorEx.class, "backTracking");
//        centerEncoder.setDirection(DcMotorEx.Direction.FORWARD);
//        centerEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        speedRate = DEFAULT_SPEED_RATE;

        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    private void finish() {
        leftFront.close();
        rightFront.close();
        leftBack.close();
        rightBack.close();

        telemetry.addData("status", "finished");
        telemetry.addData("runtime", runtime.toString());
        telemetry.update();
    }

}
