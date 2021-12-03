package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
public class TeleOpOneDriver extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        MecanumDrive6340 drive = new MecanumDrive6340(hardwareMap);

        PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(30, 0, 0, 13);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int armStartPosition = drive.ArmMotor.getCurrentPosition();
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
            if (gamepad1.right_trigger>0){
                drive.ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                drive.ArmMotor.setPower(gamepad1.right_trigger*0.35);
            }
            if (gamepad1.left_trigger>0){
                drive.ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                drive.ArmMotor.setPower(gamepad1.left_trigger*0.2);
            }
            if (drive.digitalTouch.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            }
            else {
                drive.stopIntakeBlocks();
            }

            if (gamepad1.dpad_right) // X is intake system
                drive.spinwheelright();
            if (gamepad1.dpad_left) // X is intake system
                drive.spinwheelleft();
            if (gamepad1.dpad_up) {
                drive.stopIntakeBlocks();
            }
            if (gamepad1.dpad_down) // X is intake system
                drive.spinwheelstop();
            if (gamepad1.y) // X is intake system
            {
             drive.ArmLifter(3,4);
            }
            if (gamepad1.b) // X is intake system
            {
                drive.ArmLifter(2,4);
            }
            if (gamepad1.a) // X is intake system
            {
                drive.ArmLifter(1,4);
            }
            if (gamepad1.x) // X is intake system
            {
                drive.ArmLifter(-1, 4);
            }
            if (gamepad1.right_bumper) // X is intake system
            {
                drive.inTakeblocks();
            }

            if (gamepad1.left_bumper) // X is intake system
            {
                drive.outTakeblocks();

            }
            drive.update();

            /*
            shooterServo positions
            0 = LOAD
            1 = FIRE
             */




                    drive.update();


            //drive.Arm(gamepad2.right_stick_y/2); // Right stick down pulls arm up, and vice versa


            /*
            Arm Controller
             */
          //  drive.arm.setPower(-gamepad2.left_stick_y*.4);


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


