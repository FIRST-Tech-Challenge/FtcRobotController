package org.firstinspires.ftc.teamcode.Mechanisms.Intake.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Test Autonomous Intake", group = "Autonomous")
public class TuneIntake2 extends LinearOpMode {

    public static double speed = 1;
    public static double intakeDuration = 2.0;  // How long the intake motor runs (in seconds)

    @Override
    public void runOpMode() {
        // Get the hardware
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "lfm");
        Servo intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        // Wait for the start button
        waitForStart();

        while (opModeIsActive()) {
            while(gamepad1.circle){
                intake.setPower(0.5);
            }
            if(gamepad1.cross){
                intake.setPower(0);
            }
        }


        // Run the intake motor and set the servo for a certain duration (autonomous action)
//        if (opModeIsActive()) {
//            intake.setPower(speed);        // Run the intake forward
//            intakeServo.setPosition(0.5);  // Set the servo to the "engage" position
//            sleep((long)(intakeDuration * 1000));  // Run for the specified duration (in milliseconds)
//
//            intake.setPower(0);            // Stop the intake motor
//            intakeServo.setPosition(0);    // Reset the servo position (disengage)
//        }
    }
}
