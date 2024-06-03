package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Made by Pearl on May 31, 2024 following FTC documentation
 *  https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/creating_op_modes/Creating-and-Running-an-Op-Mode-%28Android-Studio%29.html
 */

// This is called an anotation, which tells the mode of the Operation Mode

@TeleOp // Happens with a controler and a Driver
//@Automous // Makes the roboot run by itself

// Creates the OpMode
public class TestJavaOpMode extends LinearOpMode {
    
    // Primatives and references
    private Gyroscope imu;
    private DcMotor motorTest;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private Servo servoTest;

    // Every OpMode of type LinearOpMode must implenent this method
    @Override
    public void runOpMode() {
        // All the stuff here runs one and only once

        // Hardware map args must tach the reference defined the device in the config file
        imu = hardwareMap.get(DcMotor.class, "imu");
        motorTest = hardwareMap.get(DigitalChannel.class, "motorTest");
        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColor");

        // Sends message
        telemetry.addData("Status", "Initalized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver prsses STOP)
        double tgtPower = 0;
        while (opModeIsActive()) {
            tgtPower =  -this.gamepad1.left_stick_y;
            motorTest.setPower(tgtPower);
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", motorTest.getPower());


            // First is index, and second is message
            telemetry.addData("Status", "Running");
            telemetry.update();

        }

    }

}