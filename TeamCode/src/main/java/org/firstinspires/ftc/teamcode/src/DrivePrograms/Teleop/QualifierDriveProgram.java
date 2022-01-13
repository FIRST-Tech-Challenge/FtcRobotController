package org.firstinspires.ftc.teamcode.src.DrivePrograms.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains.TeleopDriveTrain;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.CarouselSpinner;

@TeleOp(name = "Qualifier Drive Program")
public class QualifierDriveProgram extends LinearOpMode {
    DcMotor intake;
    DcMotor slide;
    TeleopDriveTrain drivetrain;
    Servo bucketServo;
    CarouselSpinner spinner;

    boolean y_depressed = true;

    boolean y_depressed2 = true;
    double bucketServoPos = 0.7;

    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.dcMotor.get("linear_slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.dcMotor.get("intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bucketServo = hardwareMap.servo.get("bucket");

        drivetrain = new TeleopDriveTrain(hardwareMap, "front_right/vr", "front_left/vl", "back_right/h", "back_left");

        spinner = new CarouselSpinner(hardwareMap, "cs");


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (drivetrain.getFacingDirection()) {
                telemetry.addData("Facing", "Forward");
            } else {
                telemetry.addData("Facing", "Backward");
            }
            telemetry.update();

            //Declan's controls
            {
                drivetrain.setPowerFromGamepad(gamepad1);

                //Declan Speed Modifiers
                if (gamepad1.b) {
                    drivetrain.setDrivePowerMult(0.6);
                }
                if (gamepad1.y) {
                    drivetrain.setDrivePowerMult(1);

                }
                if (gamepad1.a) {
                    drivetrain.setDrivePowerMult(0.3);
                }

                //Declan gamepad y toggle
                if (!gamepad1.y) {
                    y_depressed = true;
                }

                if (gamepad1.y && y_depressed) {
                    drivetrain.flipFrontAndBack();
                    y_depressed = false;
                }
            }

            //Eli's controls
            {
                intake.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                bucketServo.setPosition(bucketServoPos);

                if (!gamepad2.y) {
                    y_depressed2 = true;
                }
                if (gamepad2.y && y_depressed2) {
                    y_depressed2 = false;
                    if (bucketServoPos == 0.7) {
                        bucketServoPos = 0.4;
                    } else {
                        bucketServoPos = 0.7;
                    }
                }
                if (gamepad2.x) {
                    spinner.setPowerBlueDirection();
                } else if (gamepad2.b) {
                    spinner.setPowerRedDirection();
                } else {
                    spinner.stop();
                }

                double linearSlidePower = -gamepad2.left_stick_y;
                if (linearSlidePower < 0) {
                    linearSlidePower = 0;
                }
                slide.setPower(linearSlidePower);

            }
        }
    }
}


