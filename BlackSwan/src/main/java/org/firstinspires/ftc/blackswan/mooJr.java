package org.firstinspires.ftc.blackswan;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "mooJr")
public class mooJr extends LinearOpMode {

    private PIDController controller;

    public static double p = 0.012, i = 0, d = 0.0001;
    public static double f = 0.1;

    public static int target = 0;

    private final double ticks_in_degree = 1425 / 180.0;

    public static int armQuickPosition = 0;

    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        DcMotor slideLeft = hardwareMap.dcMotor.get("slideLeft");
        DcMotor slideRight = hardwareMap.dcMotor.get("slideRight");
        DcMotor clawUp = hardwareMap.dcMotor.get("clawUp");

        Servo turnClaw = hardwareMap.servo.get("turnClaw");
        Servo closeClaw = hardwareMap.servo.get("closeClaw");
        Servo planeLauncher = hardwareMap.servo.get("planeLauncher");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        clawUp.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        double servoPosition = 0.5;

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        planeLauncher.setPosition(0.95);
        turnClaw.setPosition(0.69);
        telemetry.addData("claw up", clawUp.getCurrentPosition());
        telemetry.update();

        armQuickPosition = 0;

        waitForStart();

        frontLeft.setPower(.5);
        frontRight.setPower(.5);
        backLeft.setPower(.5);
        backRight.setPower(.5);

        ElapsedTime elapsedTime= new ElapsedTime();
        while (opModeIsActive() && elapsedTime.seconds()<2){

        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        }
    }