package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Wrist And Claw Test No Extend")
public class WristAndClawTest extends LinearOpMode {

    private ElapsedTime IntakeClawTime = new ElapsedTime();

    private double TouchPadInput = .5;

    private boolean IntakeClawClosed = false;
    private boolean OuttakeActive = false;
    @Override
    public void runOpMode() throws InterruptedException {

        Servo IntakeClaw = hardwareMap.servo.get("Intake Claw");              // Chub Port 0 // O Button
        Servo RightIntakeWrist = hardwareMap.servo.get("Right Intake Wrist"); // Chub Port 1 // Increments Using Dpad Side Buttons?
        Servo LeftIntakeWrist = hardwareMap.servo.get("Left Intake Wrist");   // Chub Port 2 // Ideally Stick Controlled
        Servo RightIntakeV4B = hardwareMap.servo.get("Right Intake V4B");     // Chub Port 3 // Preset To Swing Out With X
        Servo LeftIntakeV4B = hardwareMap.servo.get("Left Intake V4B");       // Chub Port 4 // --------------------------


        IntakeClaw.setPosition(0);    // Closes Intake Claw
        LeftIntakeWrist.setPosition(TouchPadInput);    // Sets the intake wrist to the starting position // Left is 0 Right is 1
        RightIntakeWrist.setPosition(TouchPadInput);   // Sets the intake wrist to the starting position // Left is 0 Right is 1

        LeftIntakeV4B.setPosition(0);   // Sets the intake virtual four bar to the starting position
        RightIntakeV4B.setPosition(0);  // Sets the intake virtual four bar to the starting position

        waitForStart();

        while (opModeIsActive()){
            // Closes the intake claw
            if (gamepad2.b && !OuttakeActive && IntakeClawTime.seconds() >= .3 && !IntakeClawClosed){ // If the B button was pressed, Outtake is retracted, It has been more than .3 seconds since the intake claw has been used and the intake claw is open
                IntakeClawTime.reset();     // Reset the timer since the intake claw was just used
                IntakeClaw.setPosition(1);  // Closes the intake claw
                IntakeClawClosed = true;    // Since the intake claw was closed we change this to stay accurate
            }
            // Opens the intake claw
            else if (gamepad2.b && !OuttakeActive && IntakeClawTime.seconds() >= .3 && IntakeClawClosed){  // If the B button was pressed, Outtake is retracted, It has been more than .3 seconds since the intake claw has been used and the intake claw is closed
                IntakeClawTime.reset();     // Reset the timer since the intake claw was just used
                IntakeClaw.setPosition(0);  // Opens the intake claw
                IntakeClawClosed = false;   // Since the intake claw was opened we change this to stay accurate
            }

            //**************************** INTAKE WRIST ************************************************************
            // Intake Wrist
            if (gamepad1.touchpad_finger_1){   // The intake is extended past 200 ticks and a finger is on the touchpad
                TouchPadInput = (gamepad1.touchpad_finger_1_x + 1) / 2;     // This is taking a range from -1 - 1 and converting it to a range of 0 - 1 and saving it to a variable
            }
            else if (gamepad2.y){
                TouchPadInput = .5;
            }
            LeftIntakeWrist.setPosition(TouchPadInput);
            RightIntakeWrist.setPosition(TouchPadInput);

            telemetry.addData("Touch Pad Input", TouchPadInput)
                     .addData("Left Wrist Position", LeftIntakeWrist.getPosition())
                     .addData("Right Wrist Position", RightIntakeWrist.getPosition())
                     .addData("Claw Closed?", IntakeClawClosed);
                     telemetry.update();
        }
    }
}
