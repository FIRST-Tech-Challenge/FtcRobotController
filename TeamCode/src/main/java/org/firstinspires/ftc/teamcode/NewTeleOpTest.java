package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp()
public class NewTeleOpTest extends LinearOpMode {
    private DcMotor m3;
    private DcMotor m0;
    private DcMotor m2;
    private DcMotor m1;
    private Servo baseServo;
    private CRServo armServo;
    private Servo clawServo;
    double open = .95;
    double closed = .87;
    double back = -.5;
    double forward = .5;


    @Override
    public void runOpMode() {
        m3 = hardwareMap.get(DcMotor.class, "m3");
        m0 = hardwareMap.get(DcMotor.class, "m0");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m1 = hardwareMap.get(DcMotor.class, "m1");
        baseServo = hardwareMap.get(Servo.class, "baseServo");
        armServo = hardwareMap.get(CRServo.class, "armServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        m3.setDirection(DcMotorSimple.Direction.REVERSE);
        m0.setDirection(DcMotorSimple.Direction.REVERSE);
        m2.setDirection(DcMotorSimple.Direction.REVERSE);
        m1.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.left_stick_y > .1) {
                m3.setPower(1);
                m0.setPower(-1);
                m2.setPower(1);
                m1.setPower(-1);
            }
            else {
                m3.setPower(0);
                m0.setPower(0);
                m2.setPower(0);
                m1.setPower(0);
            }

            if(-gamepad1.left_stick_y > .1) {
                m3.setPower(-1);
                m0.setPower(1);
                m2.setPower(-1);
                m1.setPower(1);
            }
            else {
                m3.setPower(0);
                m0.setPower(0);
                m2.setPower(0);
                m1.setPower(0);
            }
            if(gamepad1.left_stick_x >.4) {
                m3.setPower(-1);
                m0.setPower(-1);
                m2.setPower(1);
                m1.setPower(1);
            }
            else {
                m3.setPower(0);
                m0.setPower(0);
                m2.setPower(0);
                m1.setPower(0);
            }
            if(-gamepad1.left_stick_x >.4) {
                m3.setPower(1);
                m0.setPower(1);
                m2.setPower(-1);
                m1.setPower(-1);
            }
            else {
                m3.setPower(0);
                m0.setPower(0);
                m2.setPower(0);
                m1.setPower(0);
            }
            if(-gamepad1.right_stick_x >.4) {
                m3.setPower(0.5);
                m0.setPower(0.5);
                m2.setPower(0.5);
                m1.setPower(0.5);
            }
            else {
                m3.setPower(0);
                m0.setPower(0);
                m2.setPower(0);
                m1.setPower(0);
            }
            if(gamepad1.right_stick_x >.4) {
                m3.setPower(-0.5);
                m0.setPower(-0.5);
                m2.setPower(-0.5);
                m1.setPower(-0.5);
            }
            else {
                m3.setPower(0);
                m0.setPower(0);
                m2.setPower(0);
                m1.setPower(0);
            }
            if (gamepad1.right_bumper) {
                armServo.setPower(forward);
            }
            else {
                armServo.setPower(0);
            }
            while (gamepad1.right_trigger > .1) {
                armServo.setPower(back);
            }

            if(gamepad1.a) {
                clawServo.setPosition(open);
            }

            if (gamepad1.b) {
                clawServo.setPosition(closed);
            }
            baseServo.setPosition(.15);

        }
    }
}