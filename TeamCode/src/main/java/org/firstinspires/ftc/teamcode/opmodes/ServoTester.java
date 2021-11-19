package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.CRServo;

/**
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 */

@TeleOp(name = "Servo Tester", group = "Test")
public class ServoTester extends LinearOpMode {

    // Define class members
    private double position1 = 0.55; // Start at halfway position so was .5
    private double position2 = 0.5; // Start at halfway position so was .5
    private final double position3 = 0.5; // Start at halfway position so was .5
    private final double position4 = 0.5; // Start at halfway position so was .5


    private final double INCREMENT = 0.01;
    private final int DELAY = 100;//was 100

    @Override
    public void runOpMode() {

//      CRServo servo = hardwareMap.get(CRServo.class, "hand");
        Servo shooter = hardwareMap.get(Servo.class, "hopper");

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();


        telemetry.addData(">", "Servo Position: ");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                // Keep stepping up until we hit the max value.
                position1 += INCREMENT;
            } else if (gamepad1.dpad_down) {
                // Keep stepping down until we hit the min value.
                position1 -= INCREMENT;
            }

            if (gamepad1.y) {
                // Keep stepping up until we hit the max value.
                position2 += INCREMENT;
            } else if (gamepad1.a) {
                // Keep stepping down until we hit the min value.
                position2 -= INCREMENT;
            }


            // Display the current value
            telemetry.addData("Shooter (dpad up and dpad down)", "%5.2f", position1);
            telemetry.addData("Lifter Position (Y and A)", "%5.2f", position2);


            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the servo to the new position and pause;
            shooter.setPosition(position1);


            sleep(DELAY);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}