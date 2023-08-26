package org.firstinspires.ftc.teamcode.Ethan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class EthanDrive extends LinearOpMode {



    @Override
    public void runOpMode() {
        DcMotor lFront = hardwareMap.dcMotor.get("Left front");
        DcMotor rFront = hardwareMap.dcMotor.get("Right front");
        DcMotor lBack = hardwareMap.dcMotor.get("Left back");
        DcMotor rBack = hardwareMap.dcMotor.get("Right back");
        DcMotor arm = hardwareMap.dcMotor.get("arm");

        Servo lServo = hardwareMap.servo.get("lServo");
        Servo rServo = hardwareMap.servo.get("rServo");

        // chloe was here

        waitForStart();



        lServo.setDirection(Servo.Direction.REVERSE);

        rBack.setDirection(DcMotorSimple.Direction.FORWARD);
        lBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rFront.setDirection(DcMotorSimple.Direction.FORWARD);
        lFront.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        lFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                lServo.setPosition(0);
                rServo.setPosition(1);
            }
            if (gamepad1.right_bumper) {
                lServo.setPosition(1);
                rServo.setPosition(0);
            }

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
            //Arm Power
            arm.setPower(armPower);

            double fLeftPower = forwardBackward + turning + mecanuming;
            double fRightPower = forwardBackward - turning - mecanuming;
            double bLeftPower = forwardBackward + turning - mecanuming;
            double bRightPower = forwardBackward - turning + mecanuming;
            
            double maxPower = maxAbsValueDouble(fLeftPower, fRightPower, bLeftPower, bRightPower);

            if (Math.abs(maxPower) > 1) {


                fLeftPower /= Math.abs(maxPower);
                fRightPower /= Math.abs(maxPower);
                bLeftPower /= Math.abs(maxPower);
                bRightPower /= Math.abs(maxPower);
            }

            lFront.setPower(fLeftPower);
            rFront.setPower(fRightPower);
            lBack.setPower(bLeftPower);
            rBack.setPower(bRightPower);

            telemetry.addLine(String.valueOf(lFront.getCurrentPosition()));
            telemetry.addLine(String.valueOf(lBack.getCurrentPosition()));
            telemetry.addLine(String.valueOf(rFront.getCurrentPosition()));
            telemetry.addLine(String.valueOf(rBack.getCurrentPosition()));

            telemetry.update();


        }
    }

    private double maxAbsValueDouble(double a, double... others) {

        double max = a;


        for (double next : others) {
            if (Math.abs(next) > Math.abs(max)) {
                max = next;
            }

        }

        return max;
    }

}