package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.arthur;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Basic: BeeLine", group="Linear OpMode")
public class Mecanum_TeleOp extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private final int TICKS_PER_ROTATION = 800;
    private final double WHEEL_RADIUS = 0.025; // in meters
    private final double WHEEL_DISTANCE = 0.30; // in meters

    Orientation angles;

    private DcMotor leftMotors;
    private DcMotor rightMotors;
    private DcMotorEx leftEncoder;
    private double lEncoderPos = 0;
    private DcMotorEx rightEncoder;
    private double rEncoderPos = 0;
    private DcMotorEx centerEncoder;
    private double cEncoderPos = 0;

    private double speedRate;
    private double x;
    private double y;
    private double theta;
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

    private void setFloatingBehavior() {
        leftMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telemetry.addLine("FLOATING");
        floating = false;
    }

    private void setBrakeBehavior() {
        leftMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        double leftPower = gamepad.calcLeftPower(speedRate);
        double rightPower = gamepad.calcRightPower(speedRate);

        leftMotors.setPower(leftPower);
        rightMotors.setPower(rightPower);

        updatePosition();

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

    private void updateTicks() {
        updateLeftTicks();
        updateRightTicks();
        updateCenterTicks();
    }

    private void updateLeftTicks() {
        lEncoderPos = leftEncoder.getCurrentPosition();
    }

    private double getLeftTicks() {
        return leftEncoder.getCurrentPosition() - lEncoderPos;
    }

    private void updateRightTicks() {
        rEncoderPos = rightEncoder.getCurrentPosition();
    }

    private double getRightTicks() {
        return rightEncoder.getCurrentPosition() - rEncoderPos;
    }

    private void updateCenterTicks() {
        cEncoderPos = centerEncoder.getCurrentPosition();
    }

    private double getCenterTicks() {
        return centerEncoder.getCurrentPosition() - cEncoderPos;
    }

    private void updatePosition() {
        double dLeft = (getLeftTicks() / TICKS_PER_ROTATION) * Math.PI * 2 * WHEEL_RADIUS;
        double dRight = (getRightTicks() / TICKS_PER_ROTATION) * Math.PI * 2 * WHEEL_RADIUS;
        double dCenter = (getCenterTicks() / TICKS_PER_ROTATION) * Math.PI * 2 * WHEEL_RADIUS;

        x += (dLeft + dRight) / 2.0 * Math.cos(theta);
        y += (dLeft + dRight) / 2.0 * Math.sin(theta);
        theta += (dLeft - dRight) / WHEEL_DISTANCE;

        updateTicks();
    }

    private void printData() {
        telemetry.addData("speed rate", "%f%%", speedRate*100);
        telemetry.addData("left motors power", leftMotors.getPower());
        telemetry.addData("right motors power", rightMotors.getPower());
        telemetry.addData("left motors encoder value", leftMotors.getCurrentPosition());
        telemetry.addData("right motors encoder value", rightMotors.getCurrentPosition());
    }

    private static final double DEFAULT_SPEED_RATE = 0.5;

    private void initialize() {
        gamepad = new GamepadState(gamepad1);
        leftMotors = hardwareMap.get(DcMotor.class, "leftMotors");
        leftMotors.setDirection(DcMotor.Direction.FORWARD);
        leftMotors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotors = hardwareMap.get(DcMotor.class, "rightMotors");
        rightMotors.setDirection(DcMotor.Direction.REVERSE);
        rightMotors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftTracking");
        leftEncoder.setDirection(DcMotorEx.Direction.FORWARD);
        leftEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightTracking");
        rightEncoder.setDirection(DcMotorEx.Direction.FORWARD);
        rightEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        centerEncoder = hardwareMap.get(DcMotorEx.class, "backTracking");
        centerEncoder.setDirection(DcMotorEx.Direction.FORWARD);
        centerEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        speedRate = DEFAULT_SPEED_RATE;

        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    private void finish() {
        leftMotors.close();
        rightMotors.close();

        telemetry.addData("status", "finished");
        telemetry.addData("runtime", runtime.toString());
        telemetry.update();
    }

}
