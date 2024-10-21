// Program created by: Danny and William
// Purpose: FTC Robot Software

package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "TeleOp_RobotCentric")
public class TeleOpCode_RobotCentric extends LinearOpMode {

    // Run the initialize function
    RobotInitialize robot;

    @Override
    public void runOpMode() throws InterruptedException {
// create and define the initialization variable
        robot = new RobotInitialize(this);

        // initialization of the control of the robot when start is pressed
        waitForStart();

        // loop while the program is running
        // waits for controller input then runs the associated code
        while(opModeIsActive()) {
            // controller inputs that is inputted by the drive team
            controllerInput();
        }
    }

    public void controllerInput() {
        // Gamepad usages (two gamepads in use, one for driving and one for mechanisms):

        // Gamepad1 is used for driving (motor controls)
        // Gamepad2 is used for mechanism manipulation (moving servos and the lift motors)


        // Variables that store the different game pad movements for ease of reference later
        // Gamepad1 configuration
        {
            double strafeVelocity; // (left stick x-axis movement)
            strafeVelocity = Math.pow(gamepad1.left_stick_x, 3) * 5000; // Min: -10000, Max: 10000
            //telemetry.addData("gamepad1.left_stick_x (strafing)", strafePower);
            double turnVelocity; // (right stick x-axis movement)
            turnVelocity = Math.pow(gamepad1.right_stick_x, 3) * 5000; // Min: -10000, Max: 10000
            //telemetry.addData("gamepad1.right_stick_x (turning)", turnPower);
            double straightMovementVelocity; // (left stick y-axis movement)
//      straightMovementPower = 10000*(gamepad1.left_stick_y*gamepad1.left_stick_y*gamepad1.left_stick_y);
// Min: -10000, Max: 10000
            straightMovementVelocity = Math.pow(gamepad1.left_stick_y, 3) * 10000;
            //telemetry.addData("gamepad1.left_stick_y (straight movement)", strafePower);
            //Gamepad1 controls the drivetrain
            if (gamepad1.circle) {
                straightMovementVelocity = Math.pow(gamepad1.left_stick_y, 3) * 1000;
                turnVelocity = Math.pow(gamepad1.right_stick_x, 3) * 1000;
                strafeVelocity = Math.pow(gamepad1.left_stick_x, 3) * 1000;
            }
        /*if(gamepad1.y){
            //testing upper bound of lift
            robot.liftExtender.setPower(.25);
            telemetry.addData("position", robot.liftExtender.getCurrentPosition());
            telemetry.update();
        }*/

            if (gamepad1.right_trigger != 0) {
                // makes the robot drivetrain go forward
                straightMovementVelocity = Math.pow(gamepad1.right_trigger, 3) * 5000;
            }
            if (gamepad1.b) {
                // slow down
                straightMovementVelocity = Math.pow(gamepad1.right_trigger, 3) * 1000;
            }
            if (gamepad1.a) {
                // speed boost
                straightMovementVelocity = Math.pow(gamepad1.right_trigger, 3) * 10000;
            }

            // Set velocity of the motors (drivetrain)
            // Forward and backward movement (left stick y-axis movement)
            // Left and right turning (right stick x-axis movement)
            // Strafing left and right (left stick x-axis movement)
            {
                robot.fLeft.setVelocity(strafeVelocity - straightMovementVelocity + turnVelocity); // Overall
                // negative value
                robot.fRight.setVelocity(-strafeVelocity - straightMovementVelocity - turnVelocity); // Overall
                // positive value
                robot.bLeft.setVelocity(strafeVelocity + straightMovementVelocity - turnVelocity); // Overall
                // positive value
                robot.bRight.setVelocity(-strafeVelocity + straightMovementVelocity + turnVelocity); // Overall
                // negative value
            }
        }

        // Gamepad2 configuration
        {
            {
//            double liftPower = (gamepad2.right_stick_y);// Extends and retracts the lift
//            if (Math.abs(liftPower)>.2){
//                robot.liftExtender.setPower(liftPower);
//            }
//            double pitchPower = (gamepad2.left_stick_y);// Extends and retracts the lift
//            if (Math.abs(pitchPower)>.2){
//                robot.liftPitch.setPower(pitchPower);
//            }
//            int pitchPower;
//            pitchPower = Math.round(gamepad2.left_stick_y);
//            //telemetry.addData("pitchPower", pitchPower);
        }

            // Makes the lift go backwards
            if (gamepad2.square) {
                robot.liftPitch(0, 0.2);
                telemetry.addData("Pitchpos", robot.liftPitch.getCurrentPosition());
            }

            // Makes the lift go forward
            if (gamepad2.circle) {
                robot.liftPitch(200, 0.2);
                telemetry.addData("Pitchpos", robot.liftPitch.getCurrentPosition());

            }

//            if (gamepad2.triangle) {
//                robot.liftExtender(2700, 0.3);
//            }
//
//            if (gamepad2.cross) {
//                robot.liftExtender(0, 0.3);
//            }

            // Makes the robot lift move based on the control stick input
            if (Math.abs(gamepad2.left_stick_x)>0.2) {
                robot.liftPitch.setVelocity(400*gamepad2.left_stick_x);
                telemetry.addData("Pitchpos", robot.liftPitch.getCurrentPosition());

            } else{
                robot.liftPitch.setVelocity(0);
            }//1300
            //2700
            //find positon for extension
            if (Math.abs(gamepad2.right_stick_y)>0.2&&robot.liftPitch.getCurrentPosition()<2780&&
                    robot.liftPitch.getCurrentPosition()>0) {
                robot.liftExtender.setVelocity(-400*gamepad2.right_stick_y);
                telemetry.addData("extenderhpos", robot.liftExtender.getCurrentPosition());
            } else{
                robot.liftExtender.setVelocity(0);
            }


            if (gamepad2.right_trigger != 0) {
                robot.intakeToggle(0.5);
            } else {
                robot.intakeToggle(0);
            }

            if (gamepad2.left_trigger != 0) {
                robot.intakeToggle(-1);
            } else {
                robot.intakeToggle(0);
            }

            if (gamepad2.dpad_left) {
                //roll turns 90degrees
                robot.pitch.setPosition(0.08);
            }

            if (gamepad2.dpad_right) {
                //roll turns right
                robot.pitch.setPosition(.1278);
            }
            if(gamepad2.dpad_up){
               //pitch moves up
                robot.clawRoll.setPosition(0.1867);
            }
            if(gamepad2.dpad_down){
                //pitch moves down
                robot.clawRoll.setPosition(0.125);
            }

            // Set power of the motors for the lift and the servos
            // Right stick-y is lift extend
            // FIXME: robot.liftExtender((int)(Math.round()*200), 0.1);
            // FIXME: robot.liftPitch(pitchPower*200, 0.1);
            System.out.println(robot);

        }

        // accelerationAdditive is 1428.57
        // The intended result is that when the control sticks are not stationary the speed slowly
        // increases until it gets to the max value of 10000 or -10000

        /*
        if encoder value too diff from initial, then increase speed
        as change in position increases, speed increases

        diff int = currentpos-initpos
        speed

        */

//        ElapsedTime accelerationTime = new ElapsedTime();
//
//        if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0|| gamepad1.right_stick_x != 0) {
//            for (int i = 0; i < 7; i++) {
//                strafePower = gamepad1.left_stick_x * 1428.57;
//                turnPower = gamepad1.right_stick_x * 1428.57;
//                straightMovementPower = gamepad1.left_stick_y * 1428.57;
//            }
//        }

        // Prints to the robot driver station screen
        telemetry.update();
    }
}