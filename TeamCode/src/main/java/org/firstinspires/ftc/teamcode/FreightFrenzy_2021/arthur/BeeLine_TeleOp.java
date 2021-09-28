package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.arthur;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: BeeLine OpMode", group="Linear OpMode")
public class BeeLine_TeleOp extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotors;
    private DcMotor rightMotors;

    private double speedRate;
    private GamepadState gamepad;

    public static final double SPEED_RATE_INTERVAL = 0.1;

    public Action decreaseSpeedRate = new Action() {
        @Override
        public void run() {
            decreaseSpeed(SPEED_RATE_INTERVAL);
        }
    };

    public Action increaseSpeedRate = new Action() {
        @Override
        public void run() {
            increaseSpeed(SPEED_RATE_INTERVAL);
        }
    };

    private boolean floating = true;

    public Action toggleZeroPowerBehavior = new Action() {
        @Override
        public void run() {
            if (floating) {
                setFloatingBehavior();
            } else {
                setBrakeBehavior();
            }
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

        leftMotors.setPower(gamepad.calcLeftPower(speedRate));
        rightMotors.setPower(gamepad.calcRightPower(speedRate));

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

