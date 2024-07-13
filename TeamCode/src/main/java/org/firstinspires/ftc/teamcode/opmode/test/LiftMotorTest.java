package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.hardware_data.GoBilda435DcMotorData;

@Config
@TeleOp(name = "LiftMotorTest", group = "Test")

public class LiftMotorTest extends LinearOpMode {
    private DcMotorEx liftMotorL;
    private DcMotorEx liftMotorR;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;
    public static double maxVelocity = GoBilda435DcMotorData.maxTicksPerSec;
    public static int retractPos = 50;
    public static int deployPos = 300;
    public static int maxPos = 1000;
    private final int minPos = 200;
    public static int purplePlacementPos = 100;
    public static final int defaultPosition = 250;
    public static int targetPos = defaultPosition;
    public static double defaultPowerFactor = 0.5;
    public static double powerFactor = defaultPowerFactor;
    public static boolean busy;
    public static double power = 0.0;
    public static int currentPos;

    private PIDFController pidf = new PIDFController(kP, kI, kD, kF);

    private double maxPower = 0.5;

    public LiftMotorTest() {
    }

    @Override
    public void runOpMode() {
        liftMotorL = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftMotorR = hardwareMap.get(DcMotorEx.class, "liftRight");

        liftMotorL.setDirection(DcMotor.Direction.FORWARD);
        liftMotorR.setDirection(DcMotor.Direction.REVERSE);

        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double leftTrigger = 0.0;
        double rightTrigger = 0.0;

        logTelemetry();
        waitForStart();

        while (opModeIsActive()) {

            leftTrigger = gamepad1.left_trigger;
            rightTrigger = gamepad1.right_trigger;
            if (leftTrigger > 0.3) {
                liftMotorL.setPower(-power);
                liftMotorR.setPower(-power);

            } else if (rightTrigger > 0.3) {
                liftMotorL.setPower(power);
                liftMotorR.setPower(power);
            } else
            {
                liftMotorL.setPower(0.0);
                liftMotorR.setPower(0.0);
            }

            if (gamepad1.right_bumper) {
                power = power + 0.005;
            } else if (gamepad1.left_bumper) {
                power = power - 0.005;
            }

            logTelemetry();
            sleep(50);
        }
    }

    public void logTelemetry() {
        telemetry.addData("power: ", power);
        telemetry.addData("power*maxPower: ", power * maxPower);
        telemetry.addData("right bumper = power up: ", 0);
        telemetry.addData("left bumper = power down: ", 0);
        telemetry.update();
    }


    public void update() {
        pidf.setD(kD);
        pidf.setF(kF);
        pidf.setI(kI);
        pidf.setP(kP);
        setPIDFMotorPower();
        logPosition();
    }

    public void up(double powerFactor) {
        this.powerFactor = powerFactor;
        if (!atTop()) {
            targetPos++;
        }
        logPosition();
    }

    public void down(double powerFactor) {
        this.powerFactor = powerFactor;
        if (!atBottom()) {
            targetPos--;
        }
        logPosition();
    }

    public void goToPurplePlacementPosition() {
        setTargetPos(purplePlacementPos);
        while (isBusy()) {
            update();
            telemetry.addData("Moving to Purple Placement Position: ", purplePlacementPos);
            logPosition();
        }
    }

    private boolean atTop() {
        if (liftMotorL.getCurrentPosition() >= maxPos) {
            return true;
        } else {
            return false;
        }
    }

    private boolean isBusy() {
        return !pidf.atSetPoint();
    }

    private boolean atBottom() {
        if (liftMotorL.getCurrentPosition() <= minPos) {
            return true;
        } else {
            return false;
        }
    }

    public void setTargetPos(int targetPos) {
        if (targetPos >= minPos && targetPos <= maxPos) {
            this.targetPos = targetPos;
        }
    }

    private int avgCurrentPos() {
        int currentPosL, currentPosR = 0;
        currentPosL = liftMotorL.getCurrentPosition();
        currentPosR = liftMotorR.getCurrentPosition();
        currentPos = (int) ((currentPosL + currentPosR) / 2.0);
        return (currentPos);
    }

    private void setPIDFMotorPower() {
        //if (!pidf.atSetPoint()) {
        power = pidf.calculate(avgCurrentPos(), targetPos) * powerFactor;
        //}

        liftMotorL.setPower(power);
        liftMotorR.setPower(power);
    }

    private void logPosition() {
        telemetry.addData("PositionL:  ", liftMotorL.getCurrentPosition());
        telemetry.addData("PositionR:  ", liftMotorR.getCurrentPosition());
        telemetry.addData("Target:  ", targetPos);
        telemetry.addData("Power target: ", power);
        telemetry.addData("PowerL:  ", liftMotorL.getPower());
        telemetry.addData("PowerR:  ", liftMotorR.getPower());
        telemetry.update();
    }
}
