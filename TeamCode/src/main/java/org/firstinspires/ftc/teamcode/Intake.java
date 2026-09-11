package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Intake extends OpMode {
    private DcMotor rightFront, leftFront, rightBack, leftBack, intake;
    private Servo rightIntake, leftIntake;
    private double power;
    private double theta;
    private double sin;
    private double cos;
    private double max;
    private double turn;

    private double rightFrontPower, leftFrontPower, rightBackPower, leftBackPower;

    @Override
    public void init() {
        hardwareInit();
    }

    @Override
    public void loop() {
        power = (Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x) + (gamepad1.left_stick_y * gamepad1.left_stick_y));
        theta = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        sin = Math.sin(theta - Math.PI / 4);
        cos = Math.cos(theta - Math.PI / 4);
        max = Math.max(Math.abs(sin), Math.abs(cos));
        turn = gamepad1.right_stick_x; //setting the variable

        leftFrontPower = (power * sin / max - turn);
        rightFrontPower = (power * cos / max + turn);
        leftBackPower = (power * cos / max - turn);
        rightBackPower = (power * sin / max + turn);

        if((power + Math.abs(turn)) > 1) {
            leftFrontPower /= power + turn;
            leftBackPower /= power + turn;
            rightFrontPower /= power + turn;
            rightBackPower /= power + turn;
        }

        //two motors need to be negative
        intake.setPower(gamepad1.dpad_up? -0.5 : 0);
        rightFront.setPower(-rightFrontPower);
        leftBack.setPower(-leftBackPower);
        rightBack.setPower(rightBackPower);

        rightIntake.setPosition(gamepad1.right_bumper? 5: 0.5);
        leftIntake.setPosition(gamepad1.left_bumper? 5:0.5);
    }

    public void hardwareInit() {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        intake = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");

        rightIntake = hardwareMap.get(Servo.class,"rightintake");
        leftIntake = hardwareMap.get(Servo.class,"leftIntake");

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
