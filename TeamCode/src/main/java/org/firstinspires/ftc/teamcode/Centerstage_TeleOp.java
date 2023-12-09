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
        Robot robot = new Robot(hardwareMap);

        boolean intakeToggle = false;
        ElapsedTime intakeToggleTime = new ElapsedTime();

        boolean droneToggle = false;
        ElapsedTime droneToggleTime = new ElapsedTime();

        boolean trapToggle = false;
        ElapsedTime trapToggleTime = new ElapsedTime();

        boolean directionToggle = false;
        ElapsedTime directionToggleTime = new ElapsedTime();

        boolean droneLaunched = false;
        ElapsedTime droneTime = new ElapsedTime();

        robot.intake.intakeDown(true);

        waitForStart();
        robot.intake.intakeDown(true);
        while (opModeIsActive()) {


            // This helps control the intake, it toggles the intake.
            if (gamepad2.a && intakeToggleTime.time() > .75 && !intakeToggle) {
                intakeToggle = true;
                intakeToggleTime.reset(); }
            else if (gamepad2.a && intakeToggleTime.time() > .75 && intakeToggle) {
                intakeToggle = false;
                intakeToggleTime.reset();
            }

            // This function controls driving directions.
            if (gamepad1.x && directionToggleTime.time() > .75 && !directionToggle) {
                directionToggle = true;
                directionToggleTime.reset(); }
            else if (gamepad1.x && directionToggleTime.time() > .75 && directionToggle) {
                directionToggle = false;
                directionToggleTime.reset();
            }

            // This function controls the drone.
            if (gamepad2.y && droneToggleTime.time() > .75 && !droneToggle) {
                droneToggle = true;
                droneToggleTime.reset();
                robot.outtake.launchDrone(droneToggle);

            }
            else if (gamepad2.y && droneToggleTime.time() > .75 && droneToggle) {
                droneToggle = false;
                droneToggleTime.reset();
                robot.outtake.launchDrone(droneToggle);
            }

            // This function controls the trapdoor.
            if (gamepad2.x && trapToggleTime.time() > .25 && !trapToggle) {
                trapToggle = true;
                trapToggleTime.reset();
                robot.outtake.trapdoor(trapToggle);

            }
            else if (gamepad2.x && trapToggleTime.time() > .25 && trapToggle) {
                trapToggle = false;
                trapToggleTime.reset();
                robot.outtake.trapdoor(trapToggle);
            }


            // This controls the drive train using three double input methods:
            // It uses the last boolean input to reverse the controls:
            robot.driveTrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, directionToggle);

            // This functions uses one double input to drive the lift.
            robot.outtake.driveLift(gamepad2.left_stick_y);

            // This function uses a boolean to toggle the intakeMotors.
            robot.intake.driveIntake(intakeToggle);

            telemetry.addData("Front Driving Motors (Left, Right), Back Driving Motors (Left, Right)", "%4.2f, %4.2f",
                    robot.driveTrain.leftFrontDrive.getPower(),
                    robot.driveTrain.rightFrontDrive.getPower(),
                    robot.driveTrain.leftBackDrive.getPower(),
                    robot.driveTrain.rightBackDrive.getPower());
            telemetry.update();
        }
    }
}