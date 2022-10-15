package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.motorBL;
import static org.firstinspires.ftc.teamcode.Variables.motorBR;
import static org.firstinspires.ftc.teamcode.Variables.motorFL;
import static org.firstinspires.ftc.teamcode.Variables.motorFR;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Meet0Teleop", group = "A")
public class MeetZeroTeleop extends DriveMethods {

    DcMotor motorLinearSlide;
    Servo servoGrabberThing;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initMotorsBlue();
        motorLinearSlide = hardwareMap.get(DcMotor.class,"motorLS");
        servoGrabberThing = hardwareMap.get(Servo.class, "grabber");


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double leftY;
        double leftX;
        double rightX;
        double speedDiv = 2;
        double clampPosition = 0.76;
        double releasePosition =0.66;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //update doubles
            leftY = -gamepad1.left_stick_y;
            leftX = gamepad1.left_stick_x;
            rightX = gamepad1.right_stick_x;

            motorLinearSlide.setPower(gamepad2.left_stick_y/4);


//            if(gamepad2.dpad_up){
//                clampPosition = clampPosition + 0.03;
//                sleep(200);
//            }
//            if(gamepad2.dpad_down){
//                clampPosition = clampPosition - 0.03;
//                sleep(200);
//            }
//            if(gamepad2.dpad_right){
//                releasePosition = releasePosition + 0.03;
//                sleep(200);
//            }
//            if(gamepad2.dpad_left){
//                releasePosition = releasePosition - 0.03;
//                sleep(200);
//            }

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


            // Show the elapsed game time and wheel power.
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.addLine("ClampPosition: " + clampPosition);
            telemetry.addLine("ReleasePosition: " + releasePosition);
            telemetry.addLine("SpeedDiv: " + speedDiv);
            telemetry.update();
        }
    }
}
