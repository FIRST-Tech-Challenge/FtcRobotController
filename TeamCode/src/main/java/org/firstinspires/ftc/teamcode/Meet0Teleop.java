package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Meet0Teleop", group = "A")
public class Meet0Teleop extends DriveMethods {

   


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initMotorsBlue();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double leftY;
        double leftX;
        double rightX;
        double speedDiv = 2;
        double clampPosition = 0.76;
        double releasePosition =0.66;
        double aggressiveness = 3000;
        double holdingPower = 0.05;
        int slideTarget = 0;
        int slideDifference = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //update doubles
            leftY = -gamepad1.left_stick_y;
            leftX = gamepad1.left_stick_x;
            rightX = gamepad1.right_stick_x;


            if (gamepad2.x) {
                servoGrabberThing.setPosition(clampPosition);
            }
            if (gamepad2.a) {
                servoGrabberThing.setPosition(releasePosition);
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

            if(gamepad2.dpad_up){
                slideTarget = 1950;
                aggressiveness = 2000;
                holdingPower = 0.05;
            }
            if(gamepad2.dpad_down){
                slideTarget = 0;
                aggressiveness = 2750;
                holdingPower = 0.0;
            }

            if(gamepad2.left_stick_y != 0){
                slideTarget += (int)-gamepad2.left_stick_y*25;
                aggressiveness = 2500;
                sleep(50);
            }

            slideDifference = (slideTarget - Math.abs(motorSlide.getCurrentPosition()));

            motorSlide.setPower(((slideDifference / aggressiveness) + holdingPower));

            telemetry.addLine(slideDifference + "..difference");
            telemetry.addLine(Math.abs(motorSlide.getCurrentPosition()) + "..position");
            telemetry.addLine(slideTarget + "..target");
            telemetry.addLine(((slideDifference / aggressiveness) + holdingPower) + "..power");

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
