package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Impasta;

@TeleOp
public class ImpastaRobotCentricTeleOp extends LinearOpMode {
    // Declaring hardware variables
    private DcMotor fl, fr, bl, br, leftSlide, rightSlide, Winch, Intake;
    private Servo out1, out2, launchPlane, aimLauncher;
    private CRServo DRV4BL, DRV4BR;
    private double up, down, current;
    Impasta impasta;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initializing motors, servos, and sensors
        fl = hardwareMap.dcMotor.get("m0"); //Drivebase // ControlHub 2
        fr = hardwareMap.dcMotor.get("m1"); //Drivebase // ExpansionHub 2
        bl = hardwareMap.dcMotor.get("m2"); //Drivebase // ControlHub 3
        br = hardwareMap.dcMotor.get("m3"); //Drivebase // ExpansionHub 3

        leftSlide = hardwareMap.dcMotor.get("Left Slide"); //Slides ControlHub 1
        rightSlide = hardwareMap.dcMotor.get("Right Slide"); //Slides ExpansionHub 1

        Winch = hardwareMap.dcMotor.get("Winch"); //Climbing ControlHub 0
        Intake = hardwareMap.dcMotor.get("Intake"); //Pixel // Intake ExpansionHub 0

        DRV4BL = hardwareMap.crservo.get("V4BL"); //Virtual Four Bar Servos // Left Side
        DRV4BR = hardwareMap.crservo.get("V4BR"); //Virtual Four Bar Servos //Right Side

        launchPlane = hardwareMap.servo.get("lP");
//        aimLauncher = hardwareMap.servo.get("aL");

        out1 = hardwareMap.servo.get("OL"); //Outtake
        out2 = hardwareMap.servo.get("OR"); //Outtake

        // Creating an instance of the Impasta class
        impasta = new Impasta(fl, fr, bl, br, leftSlide, rightSlide, Intake);

        impasta.reset();
        boolean reset = true;

        boolean DRV4BReset = false;

        telemetry.addLine("Initialization Done, pos reset: " + reset + " DRV4B reset: " + DRV4BReset);
        telemetry.update();

        // Waiting for the start button to be pressed
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            /** gamepad1                                                                                */
            // Driving the robot based on gamepad input
            impasta.driveBaseRobot(gamepad1.left_stick_y, -gamepad1.left_stick_x * 1.1, -gamepad1.right_stick_x);

            // Controlling intake based on trigger input
            impasta.intake(gamepad1.left_trigger - gamepad1.right_trigger);

            // Resetting IMU yaw angle if left bumper is pressed
            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                impasta.reset();
            }

            /** gamepad2                                                                                */
            // Controlling slides with gamepad input
            impasta.setSlidesPower(-gamepad2.left_stick_y);
            telemetry.addData("PowerForSlides", -gamepad2.left_stick_y);

            telemetry.addLine("SlidePos: " + -leftSlide.getCurrentPosition());
            telemetry.update();

//            //TODO Test Airplane Aim
            if (gamepad2.triangle) {
                launchPlane.setPosition(0);
            } else if (gamepad2.square) {
                launchPlane.setPosition(1);
            }

            // Controlling the Resting/Scoring state of the Virtual Four Bar
            if (gamepad2.circle) {
                // Rotate the CR servos clockwise as long as the button is pressed
                while (gamepad2.circle) {
                    DRV4BL.setPower(1.0);  // Adjust the power as needed
                    DRV4BR.setPower(1.0);  // Adjust the power as needed
                }
                // Stop the servos when the button is released
                DRV4BL.setPower(0);
                DRV4BR.setPower(0);

            } else if (gamepad2.cross) {
                // Rotate the CR servos counterclockwise as long as the button is pressed
                while (gamepad2.cross) {
                    DRV4BL.setPower(-1.0);  // Adjust the power as needed
                    DRV4BR.setPower(-1.0);  // Adjust the power as needed
                }
                // Stop the servos when the button is released
                DRV4BL.setPower(0);
                DRV4BR.setPower(0);

            }

            // Outtake switches between scoring and rest position based on button press //Swap later
            if (gamepad2.left_trigger > 0.3) {
                out1.setPosition(0.75); // left //lower
            } else {
                out1.setPosition(0.55); // left //raise
            }

            if (gamepad2.right_trigger > 0.3) {
                out2.setPosition(0.5); // right //lower
            } else {
                out2.setPosition(0.6); // right //raise
            }
        }
    }
}
