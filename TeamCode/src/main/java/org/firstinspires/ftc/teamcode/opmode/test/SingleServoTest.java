package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name = "Single Servo Test", group = "Test")
public class SingleServoTest extends LinearOpMode {

    static final double INCREMENT   = 0.005;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   40;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    ServoImplEx servo;
    double  position = .5; // Start at halfway position

    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servo = hardwareMap.get(ServoImplEx.class, "launcher");
        servo.setDirection(Servo.Direction.FORWARD);


        // Wait for the start button
        telemetry.addData(">", "Press Start to test Servo." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){

            // slew the servo, according to the rampUp (direction) variable.
            if (gamepad1.dpad_up) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT;
                if (position <= MAX_POS) {
                    servo.setPosition(position);
                }

            }
            else if (gamepad1.dpad_down) {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT;
                if (position >= MIN_POS) {
                    servo.setPosition(position);
                } else {
                    position += INCREMENT;
                }
            }

            if (gamepad1.triangle) {
                servo.setPosition(0);
            }
            else if (gamepad1.cross) {
                servo.setPosition(1);
            }

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", servo.getPosition());

            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}