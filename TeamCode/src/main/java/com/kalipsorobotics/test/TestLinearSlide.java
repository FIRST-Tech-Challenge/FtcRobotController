package com.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.modules.Outtake;

@TeleOp
public class TestLinearSlide extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        //OuttakeSlide linearSlide = new OuttakeSlide(opModeUtilities);

        Outtake outtake2024 = new Outtake(opModeUtilities);

        double lsStayUpPower = 0.1; //power LS motors need to be at to keep the slides up instead of dropping down
        double armPivotPos = 0.5;
        double clawPos = 0.9;
        double highError;
        double errorToZero;

        boolean leftBumperPressed = false;

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                outtake2024.linearSlide1.setPower(1);
                outtake2024.linearSlide2.setPower(1);
            } else if (gamepad1.b) {
                outtake2024.linearSlide1.setPower(-1);
                outtake2024.linearSlide2.setPower(-1);
            } else {
                outtake2024.linearSlide1.setPower(0);
                outtake2024.linearSlide2.setPower(0);
            }

            //linearSlide.setPower((0.75*gamepad1.left_stick_y) - lsStayUpPower);
            //linearSlideTwo.setPower((0.75*gamepad1.left_stick_y) - lsStayUpPower);

            if(gamepad1.right_stick_y < 0) {
                armPivotPos += 0.0095;
            } else if (gamepad1.right_stick_y > 0) {
                armPivotPos -= 0.0095;
            }

            //open and close positions for CLAW
            if(gamepad1.a) {
                clawPos = 0.75;
            } else if (gamepad1.y) {
                clawPos = 0.9;
            }

            //up and down positions for OUTTAKE PIVOT
            if(gamepad1.x) {
                armPivotPos = 0.13;
            } else if (gamepad1.b){
                armPivotPos = 0.5;
            }

            //servo positions are from 0 to 1
            if(armPivotPos < 0) {
                armPivotPos = 0;
            } else if (armPivotPos > 0.5) {
                armPivotPos = 0.5;
            }

            //servo positions are from 0 to 1
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

            //make sure linear slides don't crash all the way down; if position is low, turn off power on motors
            if(Math.abs(outtake2024.linearSlide1.getCurrentPosition()) < 15) {
                outtake2024.linearSlide1.setPower(0);
                outtake2024.linearSlide2.setPower(0);
            }

            if(gamepad1.left_bumper) {
                leftBumperPressed = true;
            }

            errorToZero = 25 - outtake2024.linearSlide1.getCurrentPosition();

            //REALLY basic one-button outtake stuff
            if(leftBumperPressed) {
                outtake2024.linearSlide1.setPower(Range.clip(0.03 * errorToZero, -0.9, 0.9));
                outtake2024.linearSlide2.setPower(Range.clip(0.03 * errorToZero, -0.9, 0.9));
                if(outtake2024.linearSlide1.getCurrentPosition() < -700 && outtake2024.linearSlide1.getCurrentPosition() > -800) {
                    clawPos = 0.75;
                }

                if (errorToZero < 25) {
                    leftBumperPressed = false;
                }
            }

            telemetry.addData("lienar slides position", outtake2024.linearSlide1.getCurrentPosition());
            telemetry.addData("arm pivot position", outtake2024.armPivot.getPosition());
            telemetry.addData("claw position", outtake2024.claw.getPosition());
            telemetry.update();

            outtake2024.armPivot.setPosition(armPivotPos);
            outtake2024.claw.setPosition(clawPos);


            //0.75
            //0.9

            //0.5, 0.13
            //-800
        }
    }
}
