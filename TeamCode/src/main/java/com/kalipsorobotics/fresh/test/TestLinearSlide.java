package com.kalipsorobotics.fresh.test;

import com.kalipsorobotics.fresh.OpModeUtilities;
import com.kalipsorobotics.fresh.mechanism.OuttakeSlide;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class TestLinearSlide extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        //OuttakeSlide linearSlide = new OuttakeSlide(opModeUtilities);

        DcMotor linearSlide = hardwareMap.get(DcMotor.class,"linearSlide");
        DcMotor linearSlideTwo = hardwareMap.get(DcMotor.class,"linearSlideTwo");
        Servo armPivot = hardwareMap.get(Servo.class, "armPivot");
        Servo claw = hardwareMap.get(Servo.class, "claw");

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double lsStayUpPower = 0.1;
        double armPivotPos = 0.5;
        double clawPos = 0.9;
        double highError;
        double errorToZero;

        boolean leftBumperPressed = false;

        waitForStart();
        while (opModeIsActive()) {
//            if (gamepad1.a) {
//                linearSlide.setPower(1);
//            } else if (gamepad1.b) {
//                linearSlide.setPower(-1);
//            } else {
//                linearSlide.setPower(0);
//            }

            linearSlide.setPower((0.75*gamepad1.left_stick_y) - lsStayUpPower);
            linearSlideTwo.setPower((0.75*gamepad1.left_stick_y) - lsStayUpPower);

            if(gamepad1.right_stick_y < 0) {
                armPivotPos += 0.0095;
            } else if (gamepad1.right_stick_y > 0) {
                armPivotPos -= 0.0095;
            }

            if(gamepad1.a) {
                clawPos = 0.75;
            } else if (gamepad1.y) {
                clawPos = 0.9;
            }

            if(gamepad1.x) {
                armPivotPos = 0.13;
            } else if (gamepad1.b){
                armPivotPos = 0.5;
            }

            if(armPivotPos < 0) {
                armPivotPos = 0;
            } else if (armPivotPos > 0.5) {
                armPivotPos = 0.5;
            }

            if(clawPos < 0) {
                clawPos = 0;
            } else if (clawPos > 1) {
                clawPos = 1;
            }

//            highError = -950 - linearSlide.getCurrentPosition();
//            if(-linearSlide.getCurrentPosition()>650 && -linearSlide.getCurrentPosition()<1250) {
//                if(Math.abs(gamepad1.left_stick_y ) < 0.1) {
//                    linearSlide.setPower(Range.clip(0.03 * highError, -0.85, 0.85));
//                }
//            }

            if(Math.abs(linearSlide.getCurrentPosition()) < 15) {
                linearSlide.setPower(0);
                linearSlideTwo.setPower(0);
            }

            if(gamepad1.left_bumper) {
                leftBumperPressed = true;
            }

            errorToZero = 25 - linearSlide.getCurrentPosition();

            if(leftBumperPressed) {
                linearSlide.setPower(Range.clip(0.03 * errorToZero, -0.9, 0.9));
                linearSlideTwo.setPower(Range.clip(0.03 * errorToZero, -0.9, 0.9));
                if(linearSlide.getCurrentPosition() < -700 && linearSlide.getCurrentPosition() > -800) {
                    clawPos = 0.75;
                }

                if (errorToZero < 25) {
                    leftBumperPressed = false;
                }
            }

            telemetry.addData("lienar slides position", linearSlide.getCurrentPosition());
            telemetry.addData("arm pivot position", armPivot.getPosition());
            telemetry.addData("claw position", claw.getPosition());
            telemetry.update();

            armPivot.setPosition(armPivotPos);
            claw.setPosition(clawPos);


            //0.75
            //0.9

            //0.5, 0.13
            //-800
        }
    }
}
