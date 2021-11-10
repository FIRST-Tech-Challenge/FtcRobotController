package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


import org.firstinspires.ftc.teamcode.drive.MecanumDrive6340;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
     /*   MecanumDrive6340 drive = new MecanumDrive6340(hardwareMap);
        PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(30, 0, 0, 13);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/
        waitForStart();
        MecanumDrive6340 drive = new MecanumDrive6340(hardwareMap);
        PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(30, 0, 0, 13);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y*.9,
                            -gamepad1.left_stick_x*.9,
                            -gamepad1.right_stick_x*.7
                    )
            );
            //TODO create button map

            drive.update();

            if (gamepad2.x) // X is intake system
                drive.intakeRings();
            if(gamepad2.y)
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
                drive.shooter.setVelocity(1625);
            if (drive.shooter.getVelocity()>1575) {
                drive.shooterServo.setPosition(1);
                sleep(1000);
                drive.shooterServo.setPosition(0);
                sleep(1000);
                drive.update();

            }}
            else if (gamepad2.right_trigger < 0.5)
                drive.shooter.setVelocity(0);
            drive.shooterServo.setPosition(0);

            if (gamepad2.left_trigger > 0.5) {// Right trigger starts shooter, releasing trigger stops it
                drive.shooter.setVelocity(1500);
                if (drive.shooter.getVelocity()>1450) {
                    drive.shooterServo.setPosition(1);
                    sleep(1000);
                    drive.shooterServo.setPosition(0);
                    sleep(1000);
                }}

                    drive.update();


            //drive.Arm(gamepad2.right_stick_y/2); // Right stick down pulls arm up, and vice versa

            if (gamepad2.right_bumper) //Right bumper grabs with arm servo
                drive.grabGoal();
            if (gamepad2.left_bumper) // Left bumper releases arm servo
                drive.releaseGoal();

            /*
            Arm Controller
             */
            drive.arm.setPower(-gamepad2.left_stick_y*.4);

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

        }
    }
}


