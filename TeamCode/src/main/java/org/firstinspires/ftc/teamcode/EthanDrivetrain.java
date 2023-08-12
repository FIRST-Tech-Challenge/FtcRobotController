package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class EthanDrivetrain extends LinearOpMode {




    @Override
    public void runOpMode() {
        DcMotor lFront = hardwareMap.dcMotor.get("Left front");
        DcMotor rFront = hardwareMap.dcMotor.get("Right front");
        DcMotor lBack = hardwareMap.dcMotor.get("Left back");
        DcMotor rBack = hardwareMap.dcMotor.get("Right back");
        DcMotor arm = hardwareMap.dcMotor.get("arm");

        Servo lServo = hardwareMap.servo.get("lServo");
        Servo rServo = hardwareMap.servo.get("rServo");



        waitForStart();



        lServo.setDirection(Servo.Direction.REVERSE);

        rBack.setDirection(DcMotorSimple.Direction.FORWARD);
        lBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rFront.setDirection(DcMotorSimple.Direction.FORWARD);
        lFront.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {



            //forward & backward
            double forwardBackward = gamepad1.left_stick_y * -0.25;
            /*lFront.setPower(gamepad1.left_stick_y*-0.5);
            rFront.setPower(gamepad1.left_stick_y*-0.5);
            lBack.setPower(gamepad1.left_stick_y*-0.5);
            rBack.setPower(gamepad1.left_stick_y*-0.5);
               */
            //turning
            double turning = gamepad1.right_stick_x * 0.25;
            /*lFront.setPower(gamepad1.right_stick_x*0.5);
            rFront.setPower(gamepad1.right_stick_x*-0.5);
            lBack.setPower(gamepad1.right_stick_x*0.5);
            rBack.setPower(gamepad1.right_stick_x*-0.5);
            */
            //mecanuming
            double mecanuming = gamepad1.left_stick_x * 0.25;

            //arm up and down
            double armPower = gamepad1.right_stick_y * -0.25;
            //Everything
            lFront.setPower(forwardBackward + turning + mecanuming);
            rFront.setPower(forwardBackward - turning - mecanuming);
            lBack.setPower(forwardBackward + turning - mecanuming);
            rBack.setPower(forwardBackward - turning + mecanuming);
            arm.setPower(armPower);

            if (gamepad1.left_bumper) {
                lServo.setPosition(1);
                rServo.setPosition(1);
            }
            if (gamepad1.right_bumper) {
                lServo.setPosition(0);
                rServo.setPosition(0);
            }



        }
    }
}
//aifhesuigfwgdgyfyfhfjydsjkhfjkshekjf