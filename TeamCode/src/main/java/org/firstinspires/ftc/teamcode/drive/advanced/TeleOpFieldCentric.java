package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive6340;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@Disabled
@TeleOp(group = "advanced")
public class TeleOpFieldCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        MecanumDrive6340 drive = new MecanumDrive6340(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();

            if (gamepad2.x) // X is intake system
                drive.intakeRings();
            if (gamepad2.y)
                drive.outakeRings();
            drive.update();
            if (gamepad2.b) {// B is stopping intake
                drive.intake.setPower(0);
                drive.indexer.setPower(0);
                drive.update();
            }
            /*
            shooterServo positions
            0 = LOAD
            1 = FIRE
             */
            if (gamepad2.right_trigger > 0.5) {// Right trigger starts shooter, releasing trigger stops it
                drive.shooter.setVelocity(1600);
                if (drive.shooter.getVelocity() > 1575) {
                    drive.shooterServo.setPosition(1);
                    sleep(1000);
                    drive.shooterServo.setPosition(0);
                    sleep(1000);
                    drive.update();

                }
            } else if (gamepad2.right_trigger < 0.5)
                drive.shooter.setVelocity(0);
            drive.shooterServo.setPosition(0);

            //drive.Arm(gamepad2.right_stick_y/2); // Right stick down pulls arm up, and vice versa

            if (gamepad2.right_bumper) //Right bumper grabs with arm servo
                drive.grabGoal();
            if (gamepad2.left_bumper) // Left bumper releases arm servo
                drive.releaseGoal();

            /*
            Arm Controller
             */
            drive.arm.setPower(-gamepad2.left_stick_y * .4);

            if (gamepad2.dpad_up) {
                drive.grabGoal();
                drive.deliverGoal();
                drive.update();
            }
            if (gamepad2.dpad_down) {
                drive.arm.setPower(-.02);
                sleep(500);
                drive.releaseGoal();
                drive.update();

            }


// Show the potentiometer’s voltage in telemetry
            telemetry.addData("Potentiometer voltage", drive.armPOT.getVoltage());
            telemetry.update();
            telemetry.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
