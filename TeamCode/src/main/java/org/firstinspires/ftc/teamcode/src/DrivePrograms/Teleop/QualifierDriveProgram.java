package org.firstinspires.ftc.teamcode.src.DrivePrograms.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains.TeleopDriveTrain;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;

@TeleOp(name = "Qualifier Drive Program")
public class QualifierDriveProgram extends LinearOpMode {
    DcMotor intake;
    LinearSlide slide;
    TeleopDriveTrain drivetrain;
    Servo bucketServo;
    CarouselSpinner spinner;

    boolean x_depressed = true;
    boolean y_depressed2 = true;

    boolean dPadUpDepressed = true;
    boolean dPadDownDepressed = true;
    int posToGoTo = 0;
    double bucketServoPos = 0.7;
    boolean posOn = false;

    public void runOpMode() throws InterruptedException {
        RobotVoltageSensor r = new RobotVoltageSensor(hardwareMap);
        slide = new LinearSlide(hardwareMap, "linear_slide", r, this::opModeIsActive, this::isStopRequested);

        intake = hardwareMap.dcMotor.get("intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bucketServo = hardwareMap.servo.get("bucket");

        drivetrain = new TeleopDriveTrain(hardwareMap, "front_right/vr", "front_left/vl", "back_right/h", "back_left");

        spinner = new CarouselSpinner(hardwareMap, "cs");
        slide.setMotorPower(1);
        Thread.sleep(250);
        slide.setMotorPower(0);

        telemetry.addData("Initialization", "finished");
        telemetry.update();
        waitForStart();
        //slide.resetEncoder();

        while (opModeIsActive() && !isStopRequested()) {
            if (drivetrain.getFacingDirection()) {
                telemetry.addData("Facing", "Forward");
            } else {
                telemetry.addData("Facing", "Backward");
            }

            if (posOn) {
                switch (posToGoTo) {
                    case 0:
                        telemetry.addData("Pos", LinearSlide.HeightLevel.BottomLevel);
                        break;
                    case 1:
                        telemetry.addData("Pos", LinearSlide.HeightLevel.MiddleLevel);
                        break;
                    case 2:
                        telemetry.addData("Pos", LinearSlide.HeightLevel.TopLevel);
                        break;
                }
            } else {
                telemetry.addData("Pos", "User Defined");
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
                if (!gamepad1.x) {
                    x_depressed = true;
                }

                if (gamepad1.x && x_depressed) {
                    drivetrain.flipFrontAndBack();
                    x_depressed = false;
                }
            }

            //Eli's controls
            {
                //Handles Linear Slide Control
                {
                    if (posOn) {
                        switch (posToGoTo) {
                            case 0:
                                slide.setTargetLevel(LinearSlide.HeightLevel.BottomLevel);
                                break;
                            case 1:
                                slide.setTargetLevel(LinearSlide.HeightLevel.MiddleLevel);
                                break;
                            case 2:
                                slide.setTargetLevel(LinearSlide.HeightLevel.TopLevel);
                                break;
                        }
                        slide.threadMain();
                    }


                    if (!gamepad2.dpad_down) {
                        dPadDownDepressed = true;
                    }
                    if (gamepad2.dpad_down && dPadDownDepressed) {
                        dPadDownDepressed = false;
                        posOn = true;
                        posToGoTo--;
                    }

                    if (!gamepad2.dpad_up) {
                        dPadUpDepressed = true;
                    }
                    if (gamepad2.dpad_up && dPadUpDepressed) {
                        dPadUpDepressed = false;
                        posOn = true;
                        posToGoTo++;
                    }

                    if (posToGoTo > 2) {
                        posToGoTo = 2;
                    }
                    if (posToGoTo < 0) {
                        posToGoTo = 0;
                    }


                    if ((gamepad2.left_stick_y) != 0) {
                        slide.setMotorPower(-gamepad2.left_stick_y);
                        posOn = false;
                        slide.setTargetHeight(slide.getEncoderCount());
                    } else {
                        slide.threadMain();
                    }
                }


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


            }
        }
    }
}


