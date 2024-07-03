package org.firstinspires.ftc.masters.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.masters.Intake;

@Config
@TeleOp(name="Allia's Test", group="test")
public class AlliasTest extends LinearOpMode {

    private PIDController controller;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    public static double p = 0.012, i = 0, d = 0.0001;

    double x = 0;
    double y = 0;
    double rx = 0;

    DcMotor rightFrontMotor = null;
    DcMotor leftFrontMotor = null;
    DcMotor rightBackMotor = null;
    DcMotor leftBackMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightBackMotor = hardwareMap.dcMotor.get("backRight");
        leftBackMotor = hardwareMap.dcMotor.get("backLeft");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        Intake intake = new Intake(hardwareMap, telemetry);
        intake.init();

        boolean buttonPushed = false;
        boolean intakeStackButtonPushed = false;

        waitForStart();

        while (opModeIsActive()) {
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            if (Math.abs(y) < 0.2){
                y = 0;
            }
            if (Math.abs(x) < 0.2){
                x = 0;
            }

            double leftFrontPower = y + x + rx;
            double leftBackPower = y - x + rx;
            double rightFrontPower = y - x - rx;
            double rightBackPower = y + x - rx;

            if (Math.abs(leftFrontPower) > 1 || Math.abs(leftBackPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightBackPower) > 1) {
                double max;
                max = Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(rightBackPower));

                leftFrontPower /= max;
                leftBackPower /= max;
                rightFrontPower /= max;
                rightBackPower /= max;
            }

            leftFrontMotor.setPower(leftFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightFrontMotor.setPower(rightFrontPower);
            rightBackMotor.setPower(rightBackPower);

            if (gamepad1.right_trigger >= 0.1 && !buttonPushed){
                intake.check();
                buttonPushed = true;
            }
            if (gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1) {
                buttonPushed = false;
            }
            if (gamepad1.left_trigger >= 0.1 && !buttonPushed){
                intake.reverseCheck();
                buttonPushed = true;
            }

            if (gamepad2.x && !intakeStackButtonPushed){
                intake.lowerIntake();
                intakeStackButtonPushed = true;
            }
            if (gamepad2.b && !intakeStackButtonPushed){
                intake.raiseIntake();
                intakeStackButtonPushed = true;
            }
            if (!gamepad2.x && !gamepad2.b){
                intakeStackButtonPushed = false;
            }

            if (gamepad2.a){
                intake.setIntakeHeight(0);
            }
            if (gamepad2.y){
                intake.setIntakeHeight(5);
            }

            intake.transferCheck();
        }
    }
}
