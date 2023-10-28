package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// This file is the main TeleOp file.

@TeleOp(name = "Centerstage TeleOp", group = "LinearOpMode")
public class Centerstage_TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initiates Bessie!
        Robot robot = new Robot(hardwareMap);
        boolean toggle = false;
        ElapsedTime toggleTime = new ElapsedTime();

        waitForStart();
        while (opModeIsActive()) {

            // This controls the drive train using three input methods:
            // -1 to 1
            robot.driveTrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            robot.outtake.driveLift(gamepad2.left_stick_y);

            if (gamepad2.x && toggleTime.time() > .75 && !toggle) {
                toggle = true;
                toggleTime.reset(); }
            else if (gamepad2.x && toggleTime.time() > .75 && toggle) {
                toggle = false;
                toggleTime.reset();
            }

            robot.intake.driveIntake(toggle);

            // The next six lines give the power of each of the driving motors to the driver station:
            telemetry.addData("Front Driving Motors | Left, Right", "%4.2f, %4.2f",
                    robot.driveTrain.leftFrontDrive.getPower(),
                    robot.driveTrain.rightFrontDrive.getPower());
            telemetry.addData("Back Driving Motors | Left, Right", "%4.2f, %4.2f",
                    robot.driveTrain.leftBackDrive.getPower(),
                    robot.driveTrain.rightBackDrive.getPower());
            telemetry.update();
        }
    }
}