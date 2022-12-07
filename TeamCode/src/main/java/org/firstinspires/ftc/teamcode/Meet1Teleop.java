package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.motorBL;
import static org.firstinspires.ftc.teamcode.Variables.motorBR;
import static org.firstinspires.ftc.teamcode.Variables.motorFL;
import static org.firstinspires.ftc.teamcode.Variables.motorFR;
import static org.firstinspires.ftc.teamcode.Variables.motorSlide;
import static org.firstinspires.ftc.teamcode.Variables.servoGrabberThing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Meet1Teleop", group = "A")
public class Meet1Teleop extends DriveMethods {

   


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initMotorsBlue();
        motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double leftY;
        double leftX;
        double rightX;
        double speedDiv = 2;
        // Can we deleat Clamp & Relase Pos?
        // LOOK IN VARIABLES FOR GRIBBER POSISITIONS, SEE NUMBER ON GRIBBER
        double clampPosition = 0.19;
        //double clampPosition = 0.76;
        double releasePosition = 0.5;
        //double releasePosition = 0.66;
        double aggressiveness = 3000;
        double holdingPower = 0.05;
        int slideTarget = 0;
        int slideDifference = 0;
        int targetHeight = 0;
        double sPosition = motorSlide.getCurrentPosition();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            sPosition = motorSlide.getCurrentPosition();

            //update doubles
            leftY = -gamepad1.left_stick_y;
            leftX = gamepad1.left_stick_x;
            rightX = gamepad1.right_stick_x;


            if (gamepad2.x) {
                clawClamp();
            }
            if (gamepad2.a) {
                clawRelease();
            }

            if (!gamepad1.right_bumper) {
                motorFL.setPower((leftY + leftX + rightX) / speedDiv);
                motorBL.setPower((leftY - leftX + rightX) / speedDiv);
                motorFR.setPower((leftY - leftX - rightX) / speedDiv);
                motorBR.setPower((leftY + leftX - rightX) / speedDiv);
            } else {
                motorFL.setPower(0);
                motorBL.setPower(0);
                motorFR.setPower(0);
                motorBR.setPower(0);
            }
            if (gamepad1.a) {
                speedDiv = 2;
            }
            if(gamepad1.b) {
                speedDiv = 4;
            }
            if(gamepad2.dpad_up || gamepad2.dpad_down){
                if(gamepad2.dpad_up) {
                    targetHeight++;
                    if (targetHeight > 4) {
                        targetHeight = 4;
                    }
                    sleep(150);
                }
                if (gamepad2.dpad_down) {
                    targetHeight--;
                    if (targetHeight < 0) {
                        targetHeight = 0;
                    }
                    sleep(150);
                }
                switch (targetHeight) {
                        case 0:
                            slideTarget = 0;
                            aggressiveness = 2000;
                            holdingPower = 0.0;
                            break;
                        case 1:
                            slideTarget = 200;
                            aggressiveness = 1200;
                            holdingPower = 0.06;
                            break;
                        case 2:
                            slideTarget = 1800;
                            aggressiveness = 2000;
                            holdingPower = 0.18;
                            break;
                        case 3:
                            slideTarget = 2950;
                            aggressiveness = 2000;
                            holdingPower = 0.18;
                            break;
                        case 4:
                            slideTarget = 4200;
                            aggressiveness = 2000;
                            holdingPower = 0.18;
                            break;

                    }
                }

            //Change the target height based on the height of the linear slide at the time.

            if(gamepad2.right_bumper){
                targetHeight = 4;
                sleep(50);
            }
            if(gamepad2.left_bumper){
                targetHeight = 0;
                sleep(50);

            }


            if(gamepad2.left_stick_y != 0){
                    slideTarget += (int) -gamepad2.left_stick_y * 25;
                    aggressiveness = 1250;
                    sleep(50);
                if (sPosition<300 && sPosition>0){
                    targetHeight = 1;
                }
                if (sPosition<1900 && sPosition>300){
                    targetHeight = 2;
                }
                if (sPosition<3150 && sPosition>1900){
                    targetHeight = 3;
                }
                if (sPosition>3150 && sPosition<4300){
                    targetHeight = 4;
                }
            }
            if(slideTarget<0){
                slideTarget=0;
            }
            if(slideTarget>4400){
                slideTarget = 4400;
            }

//            if(motorSlide.getCurrentPosition()<150){
//                aggressiveness = 1750;
//            }

            if(slideTarget == 0 && motorSlide.getCurrentPosition() < 150 && motorSlide.getCurrentPosition() >= 50){
                aggressiveness = 700;
                holdingPower = 0;
            }

            if(slideTarget == 0 && motorSlide.getCurrentPosition() < 50){
                aggressiveness = 400;
                holdingPower = 0;
            }


            slideDifference = (slideTarget - Math.abs(motorSlide.getCurrentPosition()));

            motorSlide.setPower(((slideDifference / aggressiveness) + holdingPower));

            telemetry.addLine(slideDifference + "..difference");
            telemetry.addLine(Math.abs(motorSlide.getCurrentPosition()) + "..position");
            telemetry.addLine(slideTarget + "..target");
            telemetry.addLine(((slideDifference / aggressiveness) + holdingPower) + "..power");
            telemetry.addLine("Target Height: " + targetHeight);

//            motorSlide.setPower(holdingPower);



            // Show the elapsed game time and wheel power.
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.addLine("ClampPosition: " + clampPosition);
            telemetry.addLine("ReleasePosition: " + releasePosition);
            telemetry.addLine("SpeedDiv: " + speedDiv);
            telemetry.addLine("linear slide position " + motorSlide.getCurrentPosition());
            telemetry.addLine("left_stick_y_2: " + gamepad2.left_stick_y);
            telemetry.update();
        }
    }
}
