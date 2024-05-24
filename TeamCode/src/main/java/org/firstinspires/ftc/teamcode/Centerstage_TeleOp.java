package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// This file is the main TeleOp file.

@TeleOp(name = "Centerstage TeleOp", group = "LinearOpMode")
public class Centerstage_TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {


        // Initiates the robots system and subsystems!
        Gobbler gobbler = new Gobbler(hardwareMap);

        ElapsedTime intakeToggleTime = new ElapsedTime();
        ElapsedTime droneToggleTime = new ElapsedTime();
        ElapsedTime trapToggleTime = new ElapsedTime();
        ElapsedTime directionToggleTime = new ElapsedTime();
        ElapsedTime hangServoTime = new ElapsedTime();
        ElapsedTime intakeDirectionToggle = new ElapsedTime();

        gobbler.outtake.closeMailbox();
        gobbler.planeHang.initServo();

        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            // This function controls the trapdoor.
            // The first input is the button used to control the trap door.
            // The second input is the time the function uses to space out inputs.
            gobbler.outtake.trapdoor(gamepad2.x, trapToggleTime);

            // This function controls the drone.
            // The first input is the button used to control the drone.
            // The second input is the time the function uses to space out inputs.
            gobbler.planeHang.launchDrone(gamepad2.y, droneToggleTime);

            // This function controls the hanging motors.
            // The first input is the button used to power the motors.
            // The second input is the button used to reverse the motors.
            gobbler.planeHang.hangMotors(gamepad2.dpad_up, gamepad2.dpad_down);

            // This function controls the hanging servo.
            // The first input is the button used to control the servo.
            // The second input is the time the function uses to space out inputs.
            gobbler.planeHang.hangServo(gamepad2.b, hangServoTime);

            // This function controls the intake and conveyor.
            // The first input is the button used to control the trap door.
            // The second input is the time the function uses to space out inputs.
            gobbler.intake.driveIntake(gamepad2.a, intakeToggleTime);
            gobbler.intake.reverseIntake(gamepad2.dpad_left, intakeDirectionToggle);

            // This controls the drive train using three double input methods.
            // The fourth input is a boolean for the direction toggle.
            // The last input is the time the function uses to space out inputs for the direction switch.
            gobbler.driveTrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.x, directionToggleTime);

            // This functions uses one double input to drive the lift.
            gobbler.outtake.driveLift(-gamepad2.left_stick_y);

            // Provides telemetry for all motors, servos, and sensors.
            telemetry.addData("Front Driving Motors (Left, Right)", "%4.2f, %4.2f",
                    gobbler.driveTrain.leftFrontDrive.getPower(),
                    gobbler.driveTrain.rightFrontDrive.getPower());
            telemetry.addData("Back Driving Motors (Left, Right)", "%4.2f, %4.2f",
                    gobbler.driveTrain.leftBackDrive.getPower(),
                    gobbler.driveTrain.rightBackDrive.getPower());
            telemetry.addData("Front Driving Motor Encoders (Left, Right)", "%d, %d",
                    gobbler.driveTrain.leftFrontDrive.getCurrentPosition(),
                    gobbler.driveTrain.rightFrontDrive.getCurrentPosition());
            telemetry.addData("Back Driving Motor Encoders (Left, Right)", "%d, %d",
                    gobbler.driveTrain.leftBackDrive.getCurrentPosition(),
                    gobbler.driveTrain.rightBackDrive.getCurrentPosition());
            telemetry.addData("Intake Motor Power/Encoder",
                   gobbler.intake.intakeMotor.getPower());
            telemetry.addData("Lift Motor Power", "%4.2f, %d",
                    gobbler.outtake.liftMotor.getPower(),
                    gobbler.outtake.getLiftMotorPos());
            telemetry.addData("Bottom Limit Status",
                    gobbler.outtake.getBottomLimitStatus());
            telemetry.addData("Mailbox Status",
                   gobbler.outtake.getMailboxStatus());
            telemetry.addData("Drone Status",
                    String.valueOf(gobbler.planeHang.droneToggle));
//            telemetry.addData("Distance Sensors (Left, Right)", "%4.2f, %4.2f",
//                    gobbler.driveTrain.getDistanceLeftValue(),
//                    gobbler.driveTrain.getDistanceRightValue());
            telemetry.update();
        }
    }
}