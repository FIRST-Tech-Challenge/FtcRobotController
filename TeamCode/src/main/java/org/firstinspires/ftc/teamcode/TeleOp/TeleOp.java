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

        waitForStart();

        MecanumDrive6340 drive = new MecanumDrive6340(hardwareMap);
        PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(30, 0, 0, 13);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int startPosition=0;
        int poistionOfArm =0;

        startPosition = drive.ArmMotor.getCurrentPosition();
        int endPosition = startPosition;
        int level1endPosition = startPosition + 76;
        int level2endPosition = startPosition + 104;
        int level3endPosition = startPosition + 176;

       // drive.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            if (gamepad2.dpad_right) // X is intake system
                drive.spinwheelright();
            if (gamepad2.dpad_left) // X is intake system
                drive.spinwheelleft();
            if (gamepad2.dpad_down) // X is intake system
                drive.spinwheelstop();
            if (gamepad2.y) // X is intake system
            {
                poistionOfArm = 1;
                endPosition = level3endPosition;

                drive.ArmMotor.setTargetPosition(startPosition);
                drive.ArmMotor.setPower(-1.0);
                while (drive.ArmMotor.getCurrentPosition() > startPosition) {
                    drive.ArmMotor.setPower(-1);
                }
                drive.ArmMotor.setPower(0);
            }
            if (gamepad2.b) // X is intake system
            {
                poistionOfArm = 1;
                endPosition = level2endPosition;

                drive.ArmMotor.setPower(-1);
                while (drive.ArmMotor.getCurrentPosition() > startPosition) {
                    drive.ArmMotor.setPower(-1);
                }
                drive.ArmMotor.setPower(0);
            }
            if (gamepad2.a) // X is intake system
            {
                poistionOfArm = 1;
                endPosition = level1endPosition;

                drive.ArmMotor.setPower(-1);
                while (drive.ArmMotor.getCurrentPosition() > startPosition) {
                    drive.ArmMotor.setPower(-1);
                }
                drive.ArmMotor.setPower(0);
            }
            if (gamepad2.x) // X is intake system
            {
                poistionOfArm = 1;
                endPosition = startPosition;
                drive.ArmMotor.setTargetPosition(startPosition);
                drive.ArmMotor.setPower(-1);
                while (drive.ArmMotor.getCurrentPosition() > startPosition) {
                    drive.ArmMotor.setPower(-1);
                }
                drive.ArmMotor.setPower(0);
            }
            if(drive.ArmMotor.getCurrentPosition()< endPosition) {
                drive.ArmMotor.setTargetPosition(endPosition);
                drive.ArmMotor.setPower(0.3);
                while (drive.ArmMotor.getCurrentPosition() < endPosition) {
                    drive.ArmMotor.setPower(0.3);
                }
                drive.ArmMotor.setPower(0.0);
            }
            drive.update();
            if (gamepad2.b) {// B is stopping intake

            }
            /*
            shooterServo positions
            0 = LOAD
            1 = FIRE
             */
            if (gamepad2.right_trigger > 0.5) {// Right trigger starts shooter, releasing trigger stops it
                }


            if (gamepad2.left_trigger > 0.5) {// Right trigger starts shooter, releasing trigger stops it
                }

                    drive.update();


            //drive.Arm(gamepad2.right_stick_y/2); // Right stick down pulls arm up, and vice versa

            if (gamepad2.right_bumper) {}//Right bumper grabs with arm servo

            if (gamepad2.left_bumper){} // Left bumper releases arm servo


            /*
            Arm Controller
             */
            drive.arm.setPower(-gamepad2.left_stick_y*.4);

            if (gamepad2.dpad_up) {

            }
       //     if (gamepad2.dpad_down) {


        //    }




// Show the potentiometer’s voltage in telemetry
            telemetry.addData(("PostionLevel:"),poistionOfArm);
            telemetry.addData(("startPosition:"),startPosition);
            telemetry.addData(("endPosition:"),endPosition);

            telemetry.addData("Position", drive.ArmMotor.getCurrentPosition());
            telemetry.update();
            telemetry.update();

        }
    }
}


