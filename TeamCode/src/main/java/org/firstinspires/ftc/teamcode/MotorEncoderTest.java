package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This <b>should</b> make a 12V DC Motor spin
 * Name of DCMotor needs to be <code>spin</code>
 * @author Joshua Miller <22jmiller@xbhs.net>
 * @version 1.0.0
 */

@TeleOp
public class MotorEncoderTest extends LinearOpMode {
    // Lets you get the Elapsed time of the program (big shocker)
    private ElapsedTime runtime = new ElapsedTime();

    // DcMotor port to give powar
    private final String powerPortName = "power"; // The device name in the controller has to be "spin" Why? I said so and choose this name because I can
    private DcMotor powerMotor;

    // Servo to servo things
    private final String servoName = "servo";
    private Servo servo;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization
        {
            // Not too sure why we have this twice, but I currently do not have the controller with me and can not test the usefulness of this
            telemetry.addData("Status", "Initialized");

            // Setting up the motor power
            powerMotor = hardwareMap.get(DcMotor.class, powerPortName); // Gets it from hardwareMap (which I believe will know about all the hardware bits connected to the robot)
            powerMotor.setDirection(DcMotor.Direction.FORWARD); // Sets the direction of the motor (useful for when you want to reverse the way it thinks forward is (like on a drive train)
            powerMotor.setPower(1); // Sets the power as fast as possible

            // Set up servo
            servo = hardwareMap.get(Servo.class,servoName);

            // Tell the driver that initialization is complete.
            telemetry.addData("Status", "Initialized");
        }

        // Waiting for the start button
        {
            waitForStart(); // Waits for start, somewhere there is a play button to press
        }

        // Prep before going into loop
        {
            runtime.reset(); // Resets the runtime to when the program actually runs

        }

        // Code while running
        while (opModeIsActive()) { // Only runs until the stop button is pressed
            //servo.setPosition();

            // Tell about the current state
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Position", servo.getPosition());
            telemetry.update(); // Update the screen
        }
    }
}
