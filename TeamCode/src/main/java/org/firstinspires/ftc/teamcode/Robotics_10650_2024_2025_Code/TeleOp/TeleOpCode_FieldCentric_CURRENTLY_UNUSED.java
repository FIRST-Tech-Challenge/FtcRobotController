// Program created by: Danny and William
// Purpose: FTC Robot Software

package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.RobotInitialize;

@TeleOp (name = "TeleOp_FieldCentric")
@Disabled // This here until it is ready to be used to avoid
// confusion, remove after it is ready to be used
public class TeleOpCode_FieldCentric_CURRENTLY_UNUSED extends LinearOpMode {

    // Run the initialize function
    RobotInitialize robot;

    @Override
    public void runOpMode() throws InterruptedException {
// create and define the initialization variable
        robot = new RobotInitialize(this, false);

        // initialization of the control of the robot when start is pressed
        waitForStart();

        // loop while the program is running
        // waits for controller input then runs the associated code
        while(opModeIsActive()) {
            double heading = robot.gyroScope.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            // controller inputs that is inputted by the drive team
            controllerInput();
        }
    }

    public void controllerInput() {
        // Gamepad usages (two gamepads in use, one for driving and one for mechanisms):

        // Gamepad1 is used for driving (motor controls)
        // Gamepad2 is used for mechanism manipulation (moving servos)


        // Variables that store the different game pad movements for ease of reference later

        double strafePower; // (left stick x-axis movement)
        strafePower = Math.pow(gamepad1.left_stick_x,3) * 5000; // Min: -10000, Max: 10000
        //telemetry.addData("gamepad1.left_stick_x (strafing)", strafePower);
        double turnPower; // (right stick x-axis movement)
        turnPower = Math.pow(gamepad1.right_stick_x,3) * 5000; // Min: -10000, Max: 10000
        //telemetry.addData("gamepad1.right_stick_x (turning)", turnPower);
        double straightMovementPower; // (left stick y-axis movement)
//      straightMovementPower = 10000*(gamepad1.left_stick_y*gamepad1.left_stick_y*gamepad1.left_stick_y);
// Min: -10000, Max: 10000
        straightMovementPower = Math.pow(gamepad1.left_stick_y, 3) * 10000;
        //telemetry.addData("gamepad1.left_stick_y (straight movement)", strafePower);

        //double strafePower; // (left stick x-axis movement)
        strafePower = gamepad1.left_stick_x * 10000;
        telemetry.addData("gamepad1.left_stick_x (strafing)", strafePower);
        //double turnPower; // (right stick x-axis movement)
        turnPower = gamepad1.right_stick_x * 10000;
        telemetry.addData("gamepad1.right_stick_x (turning)", turnPower);
        //double straightMovementPower; // (left stick y-axis movement)
        straightMovementPower = gamepad1.left_stick_y * 10000;
        telemetry.addData("gamepad1.left_stick_y (straight movement)", strafePower);

        // Set velocity of the motors
        // Forward and backward movement (left stick y-axis movement)
        // Left and right turning (right stick x-axis movement)
        // Strafing left and right (left stick x-axis movement)

        robot.fLeft.setVelocity(strafePower - straightMovementPower + turnPower); // Overall
        // negative value
        robot.fRight.setVelocity(-strafePower - straightMovementPower - turnPower); // Overall
        // positive value
        robot.bLeft.setVelocity(strafePower + straightMovementPower - turnPower); // Overall
        // positive value
        robot.bRight.setVelocity(-strafePower + straightMovementPower + turnPower); // Overall
        // negative value

        // Makes the pitch servo go all the way up
        // Make sure claw is fully closed before lifting up (set up conditional for this)
//        if (gamepad1.triangle && (Lclaw.getPosition() <= 0.2 && Lclaw.getPosition() >= 0.1) &&
//                (Rclaw.getPosition() >= 0.8 && Rclaw.getPosition() <= 0.9)) {
//            pitch.setPosition(1);
//        }

        // Is supposed to make the pitch servo touch the ground (it keeps going too far down right now)
        if (gamepad1.cross) {
            //pitch.setPosition(0.6);
        }

        // Makes the Lclaw and Rclaw servos open (fix right claw)
        if (gamepad1.circle) {
            // 0.6 is maximum value of Lclaw (opening all the way to the left)
            // Lclaw.setPosition(0.6); (Temporarily disabled but still working)
            // The range value works as intended for this servo

            //Rclaw.setPosition(0.4);

            // For some reason the Rclaw needs a very specific setPosition
            // between 0.4-0.5 [0.4?]? As Rclaw setPosition value approaches 1 the claw closes
            // inwards; As Rclaw setPosition value goes away from 1 (0.9 or below)
            // the claw opens outwards (negative values are not necessary)
        }

        // Makes the Lclaw and Rclaw servos close (fix the right claw)
        if (gamepad1.square) {
            // Lclaw goes right as setPosition decreases
            // Lclaw goes left as setPosition increases
            // Lclaw.setPosition(0.5); (Temporarily disabled but still working)
            //Rclaw.setPosition(0.9);
            // Fix the right claw
        }

        // Prints to the robot driver station screen
        telemetry.update();
    }
}